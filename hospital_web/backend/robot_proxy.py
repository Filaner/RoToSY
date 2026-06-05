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
