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

@router.post('/recover')
async def recover() -> dict:
    """Safety recovery: fault state → STANDBY → Servo ON."""
    node = ros.get_node()
    if node is None:
        raise HTTPException(status_code=503, detail='ROS2 node not initialized')
    try:
        # 1단계: 안전 상태 해제 (SAFE_STOP/SAFE_OFF → STANDBY)
        result = await node.call_recover()
        if not result.get('success'):
            raise HTTPException(status_code=500, detail=result.get('message', 'Recovery failed'))

        # 2단계: fault 상태 벗어날 때까지 폴링 (최대 10초)
        _FAULT_STATES = {3, 5, 6, 15}  # SAFE_OFF, SAFE_STOP, EMERGENCY_STOP, NOT_READY
        for _ in range(20):
            await asyncio.sleep(0.5)
            if node.get_state().get('robot_state', -1) not in _FAULT_STATES:
                break
        else:
            raise HTTPException(status_code=500,
                                detail='복구 타임아웃 — 로봇 상태 확인 필요 (티치펜던트 또는 E-Stop 버튼 확인)')

        # 3단계: 안정화 후 서보 ON
        await asyncio.sleep(0.5)
        servo_result = await node.call_servo(True)
        servo_ok = servo_result.get('success', False)

        return {
            'success': True,
            'message': f"안전모드 해제 완료 → 서보 {'ON' if servo_ok else 'ON 실패 (수동으로 Servo ON 버튼 누르세요)'}",
            'servo_enabled': servo_ok,
        }
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
