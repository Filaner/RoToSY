"""
Status endpoints — read-only robot state.

Future integration:
  - Vision team: add GET /api/vision/latest here (or a separate vision.py router)
  - DB team:     add GET /api/log/history here (or a separate db.py router)
"""

from fastapi import APIRouter, HTTPException

from .. import ros_node as ros

router = APIRouter(tags=['status'])


@router.get('/status')
async def get_status() -> dict:
    """Return the latest robot state snapshot."""
    node = ros.get_node()
    if node is None:
        raise HTTPException(status_code=503, detail='ROS2 node not initialized')
    return node.get_state()


@router.get('/health')
async def health() -> dict:
    """Liveness check for the web server and ROS2 node."""
    node = ros.get_node()
    return {
        'web_server': 'ok',
        'ros2_node':  'ok' if node is not None else 'not_initialized',
    }
