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
    return r

@router.post('/recover')
async def recover():
    r = await _proxy('/api/recover')
    ms.add_audit('admin', 'ROBOT_RECOVER')
    return r
