"""Mission orchestration for prescription picking and AMR dispatch."""

import asyncio
import json
import threading
from datetime import datetime
from typing import Optional

from . import mission_state as ms
from . import prescription_state as ps
from . import robot_proxy
from . import ros_bridge
from . import pallet_pack as pp
from .db_schema import get_conn


class OrchestratorError(Exception):
    status = 'FAILED'


class RobotUnavailableError(OrchestratorError):
    status = 'FAILED_ROBOT'


class CameraUnavailableError(OrchestratorError):
    status = 'FAILED_CAMERA'


class OcrMismatchError(OrchestratorError):
    status = 'FAILED_OCR'


class MappingError(OrchestratorError):
    status = 'FAILED_MAPPING'


class MissionConflictError(OrchestratorError):
    status = 'FAILED_CONFLICT'


_lock = threading.Lock()
_state = {
    'running': False,
    'phase': 'IDLE',
    'status': 'IDLE',
    'prescription_id': None,
    'mission_id': None,
    'current_drawer': None,
    'current_drawer_index': 0,
    'drawers_queue': [],
    'labels_queue': [],
    'message': '',
    'error': '',
    'started_at': None,
    'updated_at': None,
    'retry_count': 0,
    'cancel_requested': False,
}
_task: Optional[asyncio.Task] = None


def _save_state_to_db() -> None:
    try:
        state_json = json.dumps(_state)
        with get_conn() as c:
            c.execute(
                '''INSERT INTO orchestrator_state (id, state_json, updated_at)
                   VALUES (1, ?, strftime('%Y-%m-%dT%H:%M:%S','now'))
                   ON CONFLICT(id) DO UPDATE SET state_json=excluded.state_json, updated_at=excluded.updated_at''',
                (state_json,),
            )
    except Exception as e:
        print(f"[Orchestrator] Error saving state to DB: {e}")


def load_state_from_db() -> None:
    try:
        with get_conn() as c:
            table_check = c.execute(
                "SELECT name FROM sqlite_master WHERE type='table' AND name='orchestrator_state'"
            ).fetchone()
            if not table_check:
                return
            row = c.execute('SELECT state_json FROM orchestrator_state WHERE id=1').fetchone()
            if row:
                loaded = json.loads(row['state_json'])
                if loaded.get('running'):
                    loaded['running'] = False
                    loaded['phase'] = 'FAILED_INTERRUPTED'
                    loaded['status'] = 'FAILED_INTERRUPTED'
                    loaded['message'] = '서버 재시작으로 인해 오케스트레이션이 중단되었습니다.'
                with _lock:
                    _state.update(loaded)
                
                # Restore mission_state queue
                drawers = loaded.get('drawers_queue', [])
                labels = loaded.get('labels_queue', [])
                current_idx = loaded.get('current_drawer_index', 0)
                if drawers:
                    ms.set_marker_queue(drawers, labels)
                    ms.set_marker_queue_index(current_idx)
    except Exception as e:
        print(f"[Orchestrator] Error loading state from DB: {e}")


def _now() -> str:
    return datetime.now().isoformat(timespec='seconds')


def _set(**updates) -> dict:
    with _lock:
        _state.update(updates)
        _state['updated_at'] = _now()
        current_state = dict(_state)
    _save_state_to_db()
    return current_state


def get_state() -> dict:
    with _lock:
        state = dict(_state)
    state['mission'] = ms.get_mission()
    state['marker_queue'] = ms.get_marker_queue()
    return state


def is_running() -> bool:
    with _lock:
        return bool(_state['running'])


def _ensure_idle_for_start() -> None:
    with _lock:
        if _state['running']:
            raise MissionConflictError('이미 오케스트레이션이 실행 중입니다.')


def _build_drawer_queue(pid: str) -> tuple[list[int], list[str], list[str]]:
    drawers, labels, missing = [], [], []
    with get_conn() as c:
        pres = c.execute('SELECT id FROM prescription WHERE code=?', (pid,)).fetchone()
        if not pres:
            return [], [], []
        rows = c.execute(
            '''SELECT pi.medicine_name, pi.medicine_id, pi.quantity
               FROM prescription_item pi
               WHERE pi.prescription_id = ?
               ORDER BY pi.sort_order, pi.id''',
            (pres['id'],),
        ).fetchall()
        for row in rows:
            name = row['medicine_name']
            # A prescription item represents one medicine type with a quantity.
            # The arm must run one full pick/place cycle for every physical box,
            # including repeated boxes from the same drawer.
            quantity = max(1, int(row['quantity'] or 1))
            if row['medicine_id'] is None:
                missing.extend([name] * quantity)
                continue
            slot = c.execute(
                'SELECT row_idx, col_idx FROM cabinet_slot WHERE medicine_id=?',
                (row['medicine_id'],),
            ).fetchone()
            if slot is None:
                missing.extend([name] * quantity)
                continue
            drawer = int(slot['row_idx']) * 2 + int(slot['col_idx']) + 1
            drawers.extend([drawer] * quantity)
            labels.extend([name] * quantity)
    return drawers, labels, missing


def _camera_ready() -> bool:
    try:
        from . import camera
        return camera.camera.available and camera.camera.error is None
    except Exception:
        return False


async def start(prescription_id: str, *, actor: str = 'admin') -> dict:
    _ensure_idle_for_start()
    _set(
        running=True,
        phase='STARTING',
        status='STARTING',
        prescription_id=prescription_id,
        mission_id=None,
        current_drawer=None,
        message='오케스트레이션 시작',
        error='',
        started_at=_now(),
        cancel_requested=False,
    )
    global _task
    _task = asyncio.create_task(_run(prescription_id, actor=actor))
    return get_state()


async def retry(actor: str = 'admin') -> dict:
    with _lock:
        pid = _state.get('prescription_id')
        running = _state.get('running')
        retry_count = int(_state.get('retry_count') or 0) + 1
    if running:
        raise MissionConflictError('실행 중에는 retry 할 수 없습니다.')
    if not pid:
        raise MissionConflictError('재시도할 처방이 없습니다.')
    _set(retry_count=retry_count)
    return await start(str(pid), actor=actor)


async def cancel(actor: str = 'admin', detail: str = '오케스트레이터 취소') -> dict:
    _set(cancel_requested=True, running=False, phase='CANCELLED', status='CANCELLED',
         message=detail)
    try:
        await robot_proxy.post('/api/motion/stop')
    except Exception:
        pass
    try:
        await ros_bridge.cancel_mission()
    except Exception:
        pass
    ms.cancel_current_mission(actor=actor, detail=detail)
    ms.add_audit(actor, 'ORCH_CANCEL', detail)
    return get_state()


async def monitor_loop() -> None:
    """Background loop for cross-module automatic transitions."""
    while True:
        await asyncio.sleep(0.5)
        with _lock:
            running = _state['running']
            phase = _state['phase']
        if running:
            continue

        mission = ms.get_mission()
        mission_status = mission.get('status', '')

        # 1) 적재 확인 완료 → AMR 출발
        if mission.get('can_dispatch') and mission_status == 'CONFIRMED':
            _set(
                running=True,
                phase='AMR_DISPATCHING',
                status='AMR_DISPATCHING',
                message='적재 확인 완료, AMR 출발',
                error='',
                mission_id=mission.get('mission_id'),
                prescription_id=mission.get('prescription_id'),
            )
            try:
                await _dispatch_amr(mission)
                _set(
                    running=False,
                    phase='AMR_DISPATCHED',
                    status='AMR_DISPATCHED',
                    message='AMR 출발 완료',
                )
            except Exception as exc:
                _fail(exc)

        # 2) 병동 도착 → 약재실 복귀 출발
        elif mission_status == 'ARRIVED' and phase == 'AMR_DISPATCHED':
            _set(
                running=True,
                phase='AMR_RETURNING',
                status='AMR_RETURNING',
                message='병동 도착 확인, AMR 약재실 복귀 시작',
            )
            try:
                result = await ros_bridge.return_to_base()
                if result.get('success'):
                    _set(running=False, phase='AMR_RETURNING',
                         message='AMR 약재실 복귀 중')
                else:
                    _set(running=False, phase='AMR_DISPATCHED',
                         message=f"복귀 명령 실패: {result.get('message', '')}")
            except Exception as exc:
                _fail(exc)

        # 3) 약재실 복귀 완료
        elif mission_status == 'COMPLETED' and phase == 'AMR_RETURNING':
            _set(
                running=False,
                phase='AMR_COMPLETED',
                status='AMR_COMPLETED',
                message='배송 완료, AMR 약재실 복귀 완료',
            )


async def _run(prescription_id: str, *, actor: str) -> None:
    try:
        prescription = ps.get(prescription_id)
        if not prescription:
            raise MissionConflictError('처방전을 찾을 수 없습니다.')
        if prescription['status'] != 'approved':
            raise MissionConflictError('승인된 처방만 오케스트레이션을 시작할 수 있습니다.')
        if not prescription.get('delivery_requested'):
            raise MissionConflictError('간호사 배송 요청이 먼저 필요합니다.')
        if not robot_proxy.is_online():
            raise RobotUnavailableError('로봇 ROS bridge가 준비되지 않았습니다.')
        if not _camera_ready():
            raise CameraUnavailableError('카메라가 준비되지 않았습니다.')

        drawers, labels, missing = _build_drawer_queue(prescription_id)
        if missing or not drawers:
            raise MappingError(
                '약품-서랍 매핑 누락: ' + (', '.join(missing) if missing else '전체')
            )

        mission = ms.new_mission(prescription['ward'], prescription_id)
        ps.set_status(prescription_id, 'awaiting_load_confirm')

        # Calculate and save pallet plan for this mission
        try:
            pallet_plan = pp.plan_for_mission(mission['mission_id'])
            if pallet_plan.get('ok'):
                ms.add_audit('system', 'PALLET_PLAN',
                             f'{mission["mission_id"]} → {pallet_plan["box_code"]} '
                             f'{pallet_plan["placed_count"]}슬롯 계획 완료')
            else:
                ms.add_audit('system', 'PALLET_PLAN_WARN',
                             f'{mission["mission_id"]} plan 실패: {pallet_plan.get("error")}')
        except Exception as exc:
            ms.add_audit('system', 'PALLET_PLAN_ERROR', f'plan_for_mission 예외: {exc}')
        
        # Save queues to the state for persistence and tracking
        _set(
            mission_id=mission.get('mission_id'),
            drawers_queue=drawers,
            labels_queue=labels,
            current_drawer_index=0,
            phase='PICKING',
            status='PICKING',
        )
        
        ms.set_marker_queue(drawers, labels)
        ms.add_audit(actor, 'ORCH_START',
                     f'{prescription_id} 큐={drawers} labels={labels}')

        for i, drawer in enumerate(drawers):
            with _lock:
                if _state.get('cancel_requested'):
                    raise MissionConflictError('사용자 취소')

            ms.set_marker_queue_index(i)
            _set(
                phase='PICKING',
                status='PICKING',
                current_drawer=drawer,
                current_drawer_index=i,
                message=f'서랍 {drawer} 피킹 시작 ({i+1}/{len(drawers)})',
            )

            arm = await robot_proxy.post('/api/motion/start', {'marker_id': drawer})
            if not arm.get('success'):
                raise RobotUnavailableError(arm.get('message', f'서랍 {drawer} 피킹 시작 실패'))

            _set(message=f'로봇팔 서랍 {drawer} 피킹 진행 중 ({i+1}/{len(drawers)})')

            await _wait_for_pick_result()

            _set(message=f'서랍 {drawer} 피킹 완료 ({i+1}/{len(drawers)})')
            await asyncio.sleep(1.0)

        # Advance the index to the end so get_marker_queue shows complete
        ms.set_marker_queue_index(len(drawers))
        _set(current_drawer_index=len(drawers))

        _set(
            running=False,
            phase='AWAITING_LOAD_CONFIRM',
            status='AWAITING_LOAD_CONFIRM',
            message='전체 피킹 완료. 약사/관리자 적재 확인 필요',
        )
        ms.add_audit('orchestrator', 'ORCH_AWAITING_LOAD_CONFIRM',
                     f'{mission.get("mission_id")} 적재 확인 대기')
    except Exception as exc:
        with _lock:
            cancelled = bool(_state.get('cancel_requested'))
        if cancelled:
            _set(
                running=False,
                phase='CANCELLED',
                status='CANCELLED',
                message='오케스트레이션 취소됨',
            )
            ms.add_audit('orchestrator', 'ORCH_CANCELLED', str(exc))
            return
        _fail(exc)


async def _wait_for_pick_result(timeout_sec: float = 300.0) -> None:
    deadline = asyncio.get_running_loop().time() + timeout_sec
    last_step = ''
    while asyncio.get_running_loop().time() < deadline:
        with _lock:
            if _state.get('cancel_requested'):
                raise MissionConflictError('사용자 취소')
        robot = robot_proxy.get_robot_state()
        step = str(robot.get('seq_step') or '')
        previous_step = last_step
        if step and step != last_step:
            last_step = step
            _set(message=f'로봇 단계: {step}')
        lowered = step.lower()
        if 'vision 실패' in step or 'failed' in lowered or '실패' in step:
            raise CameraUnavailableError(step)
        if step.startswith('ROLLBACK:'):
            raise OcrMismatchError(step)
        if step == 'IDLE' and previous_step and previous_step != 'IDLE':
            return
        await asyncio.sleep(0.5)
    raise TimeoutError('로봇팔 피킹 타임아웃')


async def _dispatch_amr(mission: dict) -> dict:
    target = mission.get('destination', '') or '간호스테이션'
    result = await ros_bridge.dispatch(target)
    if not result.get('success'):
        raise OrchestratorError(result.get('message', 'AMR 출발 실패'))
    ms.update_status(
        'DISPATCHED',
        actor='orchestrator',
        detail=f'AMR 출발 → 목적지: {target}',
    )
    ms.add_audit('orchestrator', 'ORCH_AMR_DISPATCH',
                 f'{target} 향해 출발')
    return result


def _fail(exc: Exception) -> None:
    status = getattr(exc, 'status', 'FAILED')
    message = str(exc)
    _set(running=False, phase=status, status=status, error=message, message=message)
    try:
        mission = ms.get_mission()
        if mission.get('mission_id') and mission.get('status') not in ('IDLE', 'COMPLETED'):
            ms.update_status(status, actor='orchestrator', detail=message)
    except Exception:
        pass
    ms.add_audit('orchestrator', 'ORCH_FAILED', f'{status}: {message}')
