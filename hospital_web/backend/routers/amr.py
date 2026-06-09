"""AMR and door control endpoints."""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

from .. import ros_bridge as ros
from .. import mission_state as ms
from .. import prescription_state as ps

router = APIRouter(prefix='/api/amr', tags=['amr'])


class DispatchReq(BaseModel):
    destination: str = ''


@router.post('/dispatch')
async def dispatch(req: DispatchReq):
    mission = ms.get_mission()
    if not mission['can_dispatch']:
        raise HTTPException(
            status_code=409,
            detail='약사와 관리자 모두 적재 확인을 완료해야 출발할 수 있습니다.'
        )
    result = await ros.dispatch(req.destination or mission['destination'])
    if not result.get('success'):
        raise HTTPException(status_code=500, detail=result.get('message'))
    ms.update_status('DISPATCHED', actor='admin',
                     detail=f'목적지: {req.destination or mission["destination"]}')
    ms.add_audit('admin', 'AMR_DISPATCH', f'목적지: {req.destination or mission["destination"]}')
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
