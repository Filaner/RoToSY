"""Proxy robot control commands to RoToSY (localhost:8000)."""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

from .. import robot_proxy as proxy
from .. import mission_state as ms

router = APIRouter(prefix='/api/robot', tags=['robot'])


class ServoReq(BaseModel):
    enable: bool

class JogReq(BaseModel):
    joint_index: int
    speed: float

class MoveJReq(BaseModel):
    joints: list[float]
    velocity: float = 30.0
    acceleration: float = 60.0

class MoveLReq(BaseModel):
    pose: list[float]
    velocity: float = 50.0
    acceleration: float = 100.0

class TeachingReq(BaseModel):
    enable: bool

class MotionStartReq(BaseModel):
    marker_id: int

class JogCartReq(BaseModel):
    axis: str         # 'x' | 'y' | 'z'
    direction: int    # +1 | -1
    distance: float = 5.0  # mm


async def _proxy(path: str, body: dict | None = None) -> dict:
    result = await proxy.post(path, body)
    if not result.get('success', True):
        raise HTTPException(status_code=500, detail=result.get('message', 'RoToSY error'))
    return result


@router.post('/servo')
async def servo(req: ServoReq):
    r = await _proxy('/api/servo', {'enable': req.enable})
    ms.add_audit('admin', 'SERVO_' + ('ON' if req.enable else 'OFF'))
    return r

@router.post('/jog')
async def jog(req: JogReq):
    return await _proxy('/api/jog', {'joint_index': req.joint_index, 'speed': req.speed})

@router.post('/move_j')
async def move_j(req: MoveJReq):
    return await _proxy('/api/move_j', req.model_dump())

@router.post('/move_l')
async def move_l(req: MoveLReq):
    return await _proxy('/api/move_l', req.model_dump())

@router.post('/home')
async def home():
    r = await _proxy('/api/home')
    ms.add_audit('admin', 'ROBOT_HOME')
    return r

@router.post('/teaching')
async def teaching(req: TeachingReq):
    r = await _proxy('/api/teaching', {'enable': req.enable})
    ms.add_audit('admin', 'TEACHING_' + ('ON' if req.enable else 'OFF'))
    return r

@router.post('/estop')
async def estop():
    r = await _proxy('/api/estop')
    ms.add_audit('admin', 'ROBOT_ESTOP', '로봇 암 비상정지')
    ms.cancel_current_mission(actor='admin', detail='E-STOP: 미션 상태 강제 초기화')
    return r

@router.post('/recover')
async def recover():
    r = await _proxy('/api/recover')
    ms.add_audit('admin', 'ROBOT_RECOVER')
    return r


# ── Motion sequence (13-step) ────────────────────────────────────────────────

@router.post('/motion/start')
async def motion_start(req: MotionStartReq):
    """수동 호출 (옵션 X) — 서랍 인덱스(0~5) 모션 시퀀스 시작."""
    if req.marker_id < 0 or req.marker_id > 5:
        raise HTTPException(status_code=422, detail='drawer index must be 0..5')
    r = await _proxy('/api/motion/start', {'marker_id': req.marker_id})
    ms.add_audit('admin', 'MOTION_START', f'서랍 {req.marker_id + 1} (index {req.marker_id})')
    return r


@router.post('/motion/next')
async def motion_next():
    r = await _proxy('/api/motion/next')
    ms.add_audit('admin', 'MOTION_NEXT')
    return r


@router.post('/motion/stop')
async def motion_stop():
    r = await _proxy('/api/motion/stop')
    ms.add_audit('admin', 'MOTION_STOP')
    ms.cancel_current_mission(actor='admin', detail='시퀀스 정지: 미션 상태 초기화')
    return r


@router.post('/motion/reset')
async def motion_reset():
    """OCR 미일치 시 '원위치 복구' — motion_sequence 가 약품을 서랍에 되돌린다.
    (비상정지가 아니라 곱게 롤백하는 경로. _reset_cb 가 OCR 대기 중이면 rollback 결정으로 처리.)"""
    r = await _proxy('/api/motion/reset')
    ms.add_audit('admin', 'MOTION_OCR_ROLLBACK', 'OCR 미일치 → 원위치 복구 선택')
    return r


# ── Gripper / Magnet ─────────────────────────────────────────────────────────

@router.post('/gripper/on')
async def gripper_on():
    r = await _proxy('/api/gripper/on')
    ms.add_audit('admin', 'MAGNET_ON')
    return r


@router.post('/gripper/off')
async def gripper_off():
    r = await _proxy('/api/gripper/off')
    ms.add_audit('admin', 'MAGNET_OFF')
    return r


@router.get('/gripper/status')
async def gripper_status():
    async with __import__('httpx').AsyncClient(base_url=proxy.ROTOSY_BASE,
                                               timeout=__import__('httpx').Timeout(3.0)) as cli:
        try:
            r = await cli.get('/api/gripper/status')
            return r.json()
        except Exception as e:
            raise HTTPException(status_code=503, detail=str(e))


# ── Cartesian jog ────────────────────────────────────────────────────────────

@router.post('/jog_cart')
async def jog_cart(req: JogCartReq):
    return await _proxy('/api/jog_cart', req.model_dump())


# ── 자동 시퀀스 (옵션 Y) — 처방 기반 큐 자동 진행 ────────────────────────────

@router.post('/auto/start')
async def auto_start():
    """현재 미션의 마커 큐에서 첫 마커로 motion/start.
    큐 진행 인덱스는 mission_state가 보관."""
    marker = ms.queue_current_marker()
    if marker is None:
        raise HTTPException(status_code=400, detail='활성 미션의 마커 큐가 비어있습니다.')
    r = await _proxy('/api/motion/start', {'marker_id': marker})
    ms.add_audit('admin', 'AUTO_START', f'큐 시작 — 마커 {marker}')
    return {**r, 'marker_id': marker, 'queue': ms.get_marker_queue()}


@router.post('/auto/next')
async def auto_next():
    """큐 인덱스 전진 → 다음 마커로 motion/start.
    큐 끝이면 motion/stop 후 완료."""
    nxt = ms.queue_advance()
    if nxt is None:
        await _proxy('/api/motion/stop')
        ms.add_audit('admin', 'AUTO_DONE', '전 품목 픽업 완료')
        return {'success': True, 'done': True, 'queue': ms.get_marker_queue()}
    r = await _proxy('/api/motion/start', {'marker_id': nxt})
    ms.add_audit('admin', 'AUTO_NEXT', f'다음 마커 {nxt}')
    return {**r, 'marker_id': nxt, 'queue': ms.get_marker_queue()}


@router.post('/auto/stop')
async def auto_stop():
    """자동 시퀀스 중단 — motion/stop + 큐 리셋 및 미션 초기화."""
    r = await _proxy('/api/motion/stop')
    # ms.queue_clear()는 cancel_current_mission() 내부에서 자동 호출됨
    ms.cancel_current_mission(actor='admin', detail='자동 시퀀스 중단: 미션 초기화 (다시 시작 가능)')
    ms.add_audit('admin', 'AUTO_STOP', '자동 시퀀스 중단')
    return r
