"""
Control endpoints — write commands to the robot.
"""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

from .. import ros_node as ros

router = APIRouter(tags=['control'])

class ServoRequest(BaseModel):
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
