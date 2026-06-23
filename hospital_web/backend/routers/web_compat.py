"""Compatibility routes for the old web_interface HTTP API."""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

from .. import robot_proxy as proxy

router = APIRouter(prefix='/api', tags=['web_interface_compat'])


class ServoRequest(BaseModel):
    enable: bool


class TeachingRequest(BaseModel):
    enable: bool


class JogRequest(BaseModel):
    joint_index: int
    speed: float


class MoveJRequest(BaseModel):
    joints: list[float]
    velocity: float = 30.0
    acceleration: float = 60.0


class MoveLRequest(BaseModel):
    pose: list[float]
    velocity: float = 50.0
    acceleration: float = 100.0


class CartJogRequest(BaseModel):
    axis: int
    speed: float


class SequenceStartRequest(BaseModel):
    marker_id: int


class FreqRequest(BaseModel):
    freq: int


class RunRequest(BaseModel):
    run: bool


async def _dispatch(path: str, body: dict | None = None) -> dict:
    result = await proxy.post(path, body)
    if not result.get('success', True):
        raise HTTPException(status_code=500, detail=result.get('message', 'Robot bridge error'))
    return result


@router.get('/health')
async def health() -> dict:
    return {
        'web_server': 'ok',
        'ros2_node': 'ok' if proxy.is_online() else 'not_initialized',
    }


@router.post('/servo')
async def servo(req: ServoRequest):
    return await _dispatch('/api/servo', req.model_dump())


@router.post('/jog')
async def jog(req: JogRequest):
    return await _dispatch('/api/jog', req.model_dump())


@router.post('/move_j')
async def move_j(req: MoveJRequest):
    return await _dispatch('/api/move_j', req.model_dump())


@router.post('/move_l')
async def move_l(req: MoveLRequest):
    return await _dispatch('/api/move_l', req.model_dump())


@router.post('/estop')
async def estop():
    return await _dispatch('/api/estop')


@router.post('/teaching')
async def teaching(req: TeachingRequest):
    return await _dispatch('/api/teaching', req.model_dump())


@router.post('/jog_cart')
async def jog_cart(req: CartJogRequest):
    if req.speed == 0:
        return {'success': True, 'message': 'stopped'}
    direction = 1 if req.speed > 0 else -1
    return await _dispatch(
        '/api/jog_cart',
        {'axis': req.axis, 'direction': direction, 'distance': abs(req.speed)},
    )


@router.post('/home')
async def home():
    return await _dispatch('/api/home')


@router.post('/motion/start')
async def motion_start(req: SequenceStartRequest):
    return await _dispatch('/api/motion/start', req.model_dump())


@router.post('/motion/next')
async def motion_next():
    return await _dispatch('/api/motion/next')


@router.post('/motion/stop')
async def motion_stop():
    return await _dispatch('/api/motion/stop')


@router.post('/motion/reset')
async def motion_reset():
    return await _dispatch('/api/motion/reset')


@router.post('/temp_motion/start')
async def temp_motion_start(req: SequenceStartRequest):
    return await _dispatch('/api/temp_motion/start', req.model_dump())


@router.post('/temp_motion/next')
async def temp_motion_next():
    return await _dispatch('/api/temp_motion/next')


@router.post('/temp_motion/stop')
async def temp_motion_stop():
    return await _dispatch('/api/temp_motion/stop')


@router.post('/recovery')
async def recovery():
    return await _dispatch('/api/recovery')


@router.post('/gripper/on')
async def gripper_on():
    return await _dispatch('/api/gripper/on')


@router.post('/gripper/off')
async def gripper_off():
    return await _dispatch('/api/gripper/off')


@router.get('/gripper/status')
async def gripper_status():
    return {'magnet_on': proxy.get_robot_state().get('magnet_on', False)}


@router.post('/inverter/freq')
async def inverter_freq(req: FreqRequest):
    return await _dispatch('/api/inverter/freq', req.model_dump())


@router.post('/inverter/run')
async def inverter_run(req: RunRequest):
    return await _dispatch('/api/inverter/run', req.model_dump())


@router.get('/inverter/status')
async def inverter_status():
    state = proxy.get_robot_state()
    return {
        'inverter_running': state.get('inverter_running', False),
        'inverter_freq': state.get('inverter_freq', 0),
    }
