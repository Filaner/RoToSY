"""
전자석 그리퍼 제어 엔드포인트.
"""

from fastapi import APIRouter, HTTPException
from .. import ros_node as ros

router = APIRouter(prefix='/api/gripper', tags=['gripper'])


@router.post('/on')
async def magnet_on() -> dict:
    node = ros.get_node()
    if node is None:
        raise HTTPException(status_code=503, detail='ROS2 노드 미연결')
    result = await node.call_magnet(True)
    if not result.get('success'):
        raise HTTPException(status_code=500, detail=result.get('message', '전자석 ON 실패'))
    return result


@router.post('/off')
async def magnet_off() -> dict:
    node = ros.get_node()
    if node is None:
        raise HTTPException(status_code=503, detail='ROS2 노드 미연결')
    result = await node.call_magnet(False)
    if not result.get('success'):
        raise HTTPException(status_code=500, detail=result.get('message', '전자석 OFF 실패'))
    return result


@router.get('/status')
def magnet_status() -> dict:
    node = ros.get_node()
    if node is None:
        return {'magnet_on': False}
    return {'magnet_on': node.get_state().get('magnet_on', False)}
