"""Global E-Stop, mission management, confirmation, audit log."""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

from .. import robot_proxy as proxy
from .. import ros_bridge as ros
from .. import mission_state as ms

router = APIRouter(prefix='/api/system', tags=['system'])


class MissionReq(BaseModel):
    destination: str
    prescription_id: str = ''

class ConfirmReq(BaseModel):
    actor: str  # 'admin' | 'pharmacist'


@router.post('/estop_all')
async def global_estop():
    """Stop robot arm + AMR simultaneously."""
    robot_result = await proxy.post('/api/estop')
    amr_result   = await ros.cancel_mission()
    ms.add_audit('admin', 'GLOBAL_ESTOP', '전체 시스템 비상정지')
    return {
        'robot': robot_result,
        'amr':   amr_result,
    }


@router.post('/mission/new')
async def new_mission(req: MissionReq):
    data = ms.new_mission(req.destination, req.prescription_id)
    ms.add_audit('admin', 'MISSION_NEW',
                 f'{data["mission_id"]} → {req.destination}')
    return data


@router.post('/mission/confirm')
async def confirm_loading(req: ConfirmReq):
    if req.actor not in ('admin', 'pharmacist'):
        raise HTTPException(status_code=400, detail="actor must be 'admin' or 'pharmacist'")
    return ms.confirm_loading(req.actor)


@router.post('/mission/complete')
async def complete_mission():
    data = ms.update_status('COMPLETED', actor='admin', detail='약품 전달 완료')
    return data


@router.post('/mission/reset')
async def reset_mission():
    data = ms.update_status('IDLE', actor='admin', detail='미션 초기화')
    return data


@router.get('/mission')
async def get_mission():
    return ms.get_mission()


@router.get('/audit')
async def get_audit(limit: int = 100):
    return ms.get_audit_log(limit)
