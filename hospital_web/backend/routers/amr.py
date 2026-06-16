"""AMR and door control endpoints."""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

from .. import ros_bridge as ros
from .. import mission_state as ms
from .. import prescription_state as ps

router = APIRouter(prefix='/api/amr', tags=['amr'])


class DispatchReq(BaseModel):
    destination: str = ''


class GotoReq(BaseModel):
    x:     float
    y:     float
    theta: float = 0.0
    label: str   = ''


# ── 테스트용 직접 이동 (좌표를 그대로 Nav2 골로 전송) ─────────────────────────

@router.post('/test/goto')
async def test_goto(req: GotoReq):
    """raw (x, y, theta)로 로봇을 바로 이동시킨다. ward 조회/적재확인 절차 없이 동작."""
    result = await ros.goto(req.x, req.y, req.theta, req.label)
    if not result.get('success'):
        raise HTTPException(status_code=502, detail=result.get('message'))
    ms.add_audit('admin', 'AMR_TEST_GOTO',
                 f'({req.x:.2f}, {req.y:.2f}, θ={req.theta:.2f}) {req.label}')
    return result


@router.post('/test/stop')
async def test_stop():
    result = await ros.stop()
    ms.add_audit('admin', 'AMR_TEST_STOP')
    return result


@router.post('/dispatch')
async def dispatch(req: DispatchReq):
    mission = ms.get_mission()
    # 1. 적재 확인 안 됐으면 출발 불가
    if not mission['can_dispatch'] and mission['status'] != 'PICKED_UP':
        raise HTTPException(
            status_code=409,
            detail='약사와 관리자 모두 적재 확인을 완료해야 출발할 수 있습니다.'
        )

    target_destination = req.destination or mission['destination']
    actual_nav_target = target_destination

    # 2. 스테이션 경유 로직 (0616_todo 2번 반영)
    # 현재 상태가 CONFIRMED(적재 완료 직후)이면 1차 목적지는 무조건 '간호스테이션'
    if mission['status'] == 'CONFIRMED':
        actual_nav_target = '간호스테이션'
        detail_msg = f'1차 목적지: 간호스테이션 (최종: {target_destination})'
    # 간호사가 수령(PICKED_UP)한 뒤면 최종 목적지로 출발
    elif mission['status'] == 'PICKED_UP':
        detail_msg = f'2차(최종) 목적지: {target_destination}'
    else:
        detail_msg = f'목적지: {target_destination}'

    result = await ros.dispatch(actual_nav_target)
    if not result.get('success'):
        raise HTTPException(status_code=500, detail=result.get('message'))
    
    # 3. DB 업데이트 (상태는 DELIVERING이 아니라 DISPATCHED 사용)
    ms.update_status('DISPATCHED', actor='admin', detail=detail_msg)
    ms.add_audit('admin', 'AMR_DISPATCH', detail_msg)
    return result


@router.post('/cancel')
async def cancel():
    """AMR 미션 취소 — 처방 status 'approved'로 되돌리고 확인 플래그 리셋.
    재시작 시 관리자가 '조제 시작'을 다시 누르면 새 사이클로 진입."""
    result = await ros.cancel_mission()

    # 현재 미션이 가리키는 처방을 되돌림
    current = ms.get_mission()
    pid = current.get('prescription_id')
    if pid:
        p = ps.get(pid)
        if p and p['status'] in ('awaiting_load_confirm',):
            ps.set_status(pid, 'approved')

    # 미션 row IDLE + 확인 플래그 리셋
    ms.cancel_current_mission(actor='admin', detail='미션 취소')
    ms.add_audit('admin', 'AMR_CANCEL',
                 f'처방 {pid} 재시도 가능' if pid else '미션 없음')
    return result


@router.post('/return')
async def return_to_base():
    result = await ros.return_to_base()
    ms.add_audit('admin', 'AMR_RETURN')
    return result


@router.post('/door/open')
async def door_open():
    result = await ros.door_open()
    ms.add_audit('admin', 'DOOR_OPEN', '관리자 강제 개방')
    return result


@router.post('/door/close')
async def door_close():
    result = await ros.door_close()
    ms.add_audit('admin', 'DOOR_CLOSE')
    return result
