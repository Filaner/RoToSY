"""
Thin async proxy to RoToSY REST API (localhost:8000).
Robot state is polled at POLL_HZ and cached; commands are forwarded as-is.
"""

import asyncio
import threading
from typing import Optional

import httpx

ROTOSY_BASE = 'http://localhost:8000'
POLL_HZ     = 10
_TIMEOUT    = httpx.Timeout(3.0)

_lock:         threading.Lock = threading.Lock()
_robot_state:  dict           = {}
_online:       bool           = False


# ── State access ──────────────────────────────────────────────────────────────

def get_robot_state() -> dict:
    with _lock:
        return dict(_robot_state)


def is_online() -> bool:
    return _online


# ── Background poll loop ──────────────────────────────────────────────────────

async def poll_loop() -> None:
    global _online
    from . import ros_bridge  # vision_node heartbeat 갱신용 (지연 import로 순환참조 회피)
    cam_tick = 0
    async with httpx.AsyncClient(base_url=ROTOSY_BASE, timeout=_TIMEOUT) as client:
        while True:
            try:
                r = await client.get('/api/status')
                if r.status_code == 200:
                    with _lock:
                        _robot_state.update(r.json())
                    _online = True
                else:
                    _online = False
            except Exception:
                _online = False

            # vision_node = web_interface(:8000) 카메라(HTTP). ROS 토픽이 아니라
            # 게이트웨이 HTTP로 살아있나 확인 → 카메라 켜져있으면 계속 ONLINE.
            # 2Hz로만 체크(POLL_HZ=10 → 5사이클마다)해서 트래픽 절약.
            cam_tick = (cam_tick + 1) % 5
            if cam_tick == 0:
                try:
                    cr = await client.get('/camera/markers')
                    if cr.status_code == 200 and not cr.json().get('error'):
                        ros_bridge.update_node_seen('vision_node')
                except Exception:
                    pass

            await asyncio.sleep(1.0 / POLL_HZ)


# ── Command proxy ─────────────────────────────────────────────────────────────

async def post(path: str, body: Optional[dict] = None) -> dict:
    """Forward a POST command to RoToSY and return parsed JSON."""
    async with httpx.AsyncClient(base_url=ROTOSY_BASE, timeout=httpx.Timeout(30.0)) as client:
        try:
            r = await client.post(path, json=body or {})
            data = r.json()
            if r.status_code >= 400:
                detail = data.get('detail', str(data))
                return {'success': False, 'message': detail}
            return data
        except httpx.ConnectError:
            return {'success': False, 'message': 'RoToSY (localhost:8000) 연결 불가'}
        except Exception as e:
            return {'success': False, 'message': str(e)}
