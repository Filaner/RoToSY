"""
Control endpoints — write commands to the robot.
"""

import asyncio
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

from .. import ros_node as ros

router = APIRouter(tags=['control'])

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
    pose: list[float] # [x, y, z, rx, ry, rz]
    velocity: float = 50.0
    acceleration: float = 100.0

@router.post('/servo')
async def set_servo(req: ServoRequest) -> dict:
    """Enable or disable the robot's servo motors."""
    node = ros.get_node()
    if node is None:
        raise HTTPException(status_code=503, detail='ROS2 node not initialized')
    
    try:
        result = await node.call_servo(req.enable)
        if not result.get('success'):
            msg = result.get('message', 'Unknown Error')
            status_code = 409 if 'busy' in msg.lower() else 500
            raise HTTPException(status_code=status_code, detail=msg)
        return result
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f'Internal Error: {str(e)}')


@router.post('/jog')
async def jog(req: JogRequest) -> dict:
    """Jog a joint."""
    node = ros.get_node()
    if node is None:
        raise HTTPException(status_code=503, detail='ROS2 node not initialized')

    try:
        result = await node.call_jog(req.joint_index, req.speed)
        if not result.get('success'):
            msg = result.get('message', 'Unknown Error')
            status_code = 409 if 'busy' in msg.lower() else 500
            raise HTTPException(status_code=status_code, detail=msg)
        return result
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f'Internal Error: {str(e)}')

@router.post('/move_j')
async def move_j(req: MoveJRequest) -> dict:
    """Move robot in joint space."""
    node = ros.get_node()
    if node is None:
        raise HTTPException(status_code=503, detail='ROS2 node not initialized')
    
    try:
        result = await node.call_movej(req.joints, req.velocity, req.acceleration)
        if not result.get('success'):
            msg = result.get('message', 'Unknown Error')
            status_code = 409 if 'busy' in msg.lower() else 500
            raise HTTPException(status_code=status_code, detail=msg)
        return result
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f'Internal Error: {str(e)}')

@router.post('/move_l')
async def move_l(req: MoveLRequest) -> dict:
    """Move robot in task space."""
    node = ros.get_node()
    if node is None:
        raise HTTPException(status_code=503, detail='ROS2 node not initialized')
    
    try:
        result = await node.call_movel(req.pose, req.velocity, req.acceleration)
        if not result.get('success'):
            msg = result.get('message', 'Unknown Error')
            status_code = 409 if 'busy' in msg.lower() else 500
            raise HTTPException(status_code=status_code, detail=msg)
        return result
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f'Internal Error: {str(e)}')

@router.post('/estop')
async def emergency_stop() -> dict:
    """Emergency stop: immediately halt motion and disable servo."""
    node = ros.get_node()
    if node is None:
        raise HTTPException(status_code=503, detail='ROS2 node not initialized')
    try:
        result = await node.call_estop()
        return result
    except Exception as e:
        raise HTTPException(status_code=500, detail=f'Internal Error: {str(e)}')


@router.post('/teaching')
async def set_teaching(req: TeachingRequest) -> dict:
    """Enable or disable direct teaching (manual) mode."""
    node = ros.get_node()
    if node is None:
        raise HTTPException(status_code=503, detail='ROS2 node not initialized')

    try:
        result = await node.call_teaching(req.enable)
        if not result.get('success'):
            raise HTTPException(status_code=500, detail=result.get('message', 'Teaching toggle failed'))
        return result
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f'Internal Error: {str(e)}')


class CartJogRequest(BaseModel):
    axis: int    # 0=x 1=y 2=z 3=rx 4=ry 5=rz
    speed: float # 양수=+ 방향, 음수=- 방향, 크기=스텝(mm 또는 deg), 0=정지

_cart_jog: dict = {'active': False, 'axis': -1, 'direction': 0, 'step': 0.0}


async def _cart_jog_worker(node) -> None:
    try:
        while _cart_jog['active']:
            ok = await node.call_jog_cart_step(
                _cart_jog['axis'],
                _cart_jog['direction'],
                _cart_jog['step'],
            )
            if not ok:
                break
    finally:
        _cart_jog['active'] = False


@router.post('/jog_cart')
async def jog_cart(req: CartJogRequest) -> dict:
    """Cartesian TCP jog: 한 번 호출마다 step 크기만큼 이동 반복."""
    global _cart_jog
    node = ros.get_node()
    if node is None:
        raise HTTPException(status_code=503, detail='ROS2 node not initialized')

    if req.speed == 0:
        _cart_jog['active'] = False
        return {'success': True, 'message': 'stopped'}

    if not 0 <= req.axis <= 5:
        raise HTTPException(status_code=422, detail='axis must be 0–5')

    _cart_jog.update({
        'axis':      req.axis,
        'direction': 1 if req.speed > 0 else -1,
        'step':      abs(req.speed),
    })

    if not _cart_jog['active']:
        _cart_jog['active'] = True
        asyncio.create_task(_cart_jog_worker(node))

    return {'success': True}


@router.post('/home')
async def move_home() -> dict:
    """Move robot to home position."""
    node = ros.get_node()
    if node is None:
        raise HTTPException(status_code=503, detail='ROS2 node not initialized')
    
    try:
        result = await node.call_home()
        if not result.get('success'):
            msg = result.get('message', 'Unknown Error')
            status_code = 409 if 'busy' in msg.lower() else 500
            raise HTTPException(status_code=status_code, detail=msg)
        return result
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f'Internal Error: {str(e)}')


class SequenceStartRequest(BaseModel):
    marker_id: int

@router.post('/motion/start')
async def start_sequence(req: SequenceStartRequest) -> dict:
    """Start motion sequence for a marker."""
    node = ros.get_node()
    if node is None:
        raise HTTPException(status_code=503, detail='ROS2 node not initialized')
    node.start_sequence(req.marker_id)
    return {'success': True}

@router.post('/motion/next')
async def next_step() -> dict:
    """Trigger next step in the motion sequence."""
    node = ros.get_node()
    if node is None:
        raise HTTPException(status_code=503, detail='ROS2 node not initialized')
    node.next_step()
    return {'success': True}

@router.post('/motion/stop')
async def stop_sequence() -> dict:
    """Stop the current motion sequence."""
    node = ros.get_node()
    if node is None:
        raise HTTPException(status_code=503, detail='ROS2 node not initialized')
    node.stop_sequence()
    return {'success': True}

@router.post('/motion/reset')
async def reset_sequence() -> dict:
    """Stop the current sequence and return it to its initial state."""
    node = ros.get_node()
    if node is None:
        raise HTTPException(status_code=503, detail='ROS2 node not initialized')
    node.reset_sequence()
    return {'success': True}


@router.post('/temp_motion/start')
async def start_temp_sequence(req: SequenceStartRequest) -> dict:
    """Start temp sequence for a marker."""
    node = ros.get_node()
    if node is None:
        raise HTTPException(status_code=503, detail='ROS2 node not initialized')
    node.start_temp_sequence(req.marker_id)
    return {'success': True}

@router.post('/temp_motion/next')
async def next_temp_step() -> dict:
    """Trigger next step in the temp sequence."""
    node = ros.get_node()
    if node is None:
        raise HTTPException(status_code=503, detail='ROS2 node not initialized')
    node.next_temp_step()
    return {'success': True}

@router.post('/temp_motion/stop')
async def stop_temp_sequence() -> dict:
    """Stop the current temp sequence."""
    node = ros.get_node()
    if node is None:
        raise HTTPException(status_code=503, detail='ROS2 node not initialized')
    node.stop_temp_sequence()
    return {'success': True}


@router.post('/recovery')
async def toggle_recovery() -> dict:
    """안전복구 토글: 복구 모드 진입 또는 복구 완료."""
    node = ros.get_node()
    if node is None:
        raise HTTPException(status_code=503, detail='ROS2 node not initialized')
    try:
        state = node.get_state()
        if state.get('recovery_active'):
            result = await node.exit_recovery()
        else:
            result = await node.enter_recovery()
        if not result.get('success'):
            raise HTTPException(status_code=500, detail=result.get('message', 'Recovery failed'))
        return result
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f'Internal Error: {str(e)}')
