"""
Hospital Web Gateway — Admin Dashboard backend.
Port: 8080  (RoToSY runs on 8000)

WebSocket /ws broadcasts combined state at 10 Hz:
  { robot, amr, door, mission, nodes, plc, arduino }
"""

import asyncio
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Set

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles

from . import robot_proxy as proxy
from . import ros_bridge   as ros
from . import mission_state as ms
from .routers import robot, amr, system as sys_router, prescription as presc_router, sensor as sensor_router, vision as vision_router, demo as demo_router
from . import sensor_db

STATIC_DIR   = Path(__file__).parent / 'static'
BROADCAST_HZ = 10


# ── WebSocket manager ─────────────────────────────────────────────────────────

class ConnectionManager:
    def __init__(self):
        self._clients: Set[WebSocket] = set()

    async def connect(self, ws: WebSocket):
        await ws.accept()
        self._clients.add(ws)

    def disconnect(self, ws: WebSocket):
        self._clients.discard(ws)

    async def broadcast(self, data: dict):
        dead: Set[WebSocket] = set()
        for client in self._clients:
            try:
                await client.send_json(data)
            except Exception:
                dead.add(client)
        self._clients -= dead

    @property
    def count(self) -> int:
        return len(self._clients)


manager = ConnectionManager()


def _build_state() -> dict:
    bridge   = ros.get_state()
    robot_st = proxy.get_robot_state()
    return {
        'robot':   robot_st,
        'robot_online': proxy.is_online(),
        'amr':     bridge['amr'],
        'door':    bridge['door'],
        'mission': ms.get_mission(),
        'nodes':   bridge['nodes'],
        'plc':     {'status': 'DISCONNECTED'},
        'arduino': bridge['arduino'],
    }


async def _broadcast_loop():
    interval = 1.0 / BROADCAST_HZ
    while True:
        if manager.count > 0:
            await manager.broadcast(_build_state())
        await asyncio.sleep(interval)


# ── Lifespan ──────────────────────────────────────────────────────────────────

@asynccontextmanager
async def lifespan(app: FastAPI):
    sensor_db.init_db()
    ros.init()
    broadcast_task = asyncio.create_task(_broadcast_loop())
    poll_task      = asyncio.create_task(proxy.poll_loop())
    yield
    broadcast_task.cancel()
    poll_task.cancel()
    ros.shutdown()


# ── App ───────────────────────────────────────────────────────────────────────

app = FastAPI(title='Hospital Web Gateway', version='1.0.0', lifespan=lifespan)

app.include_router(robot.router)
app.include_router(amr.router)
app.include_router(sys_router.router)
app.include_router(presc_router.router)
app.include_router(sensor_router.router)
app.include_router(vision_router.router)
app.include_router(demo_router.router)

if STATIC_DIR.exists():
    app.mount('/static', StaticFiles(directory=str(STATIC_DIR)), name='static')


@app.get('/', response_class=HTMLResponse)
async def admin_dashboard():
    return HTMLResponse((STATIC_DIR / 'admin.html').read_text())


@app.get('/pharmacist', response_class=HTMLResponse)
async def pharmacist_dashboard():
    return HTMLResponse((STATIC_DIR / 'pharmacist.html').read_text())


@app.get('/nurse', response_class=HTMLResponse)
async def nurse_dashboard():
    return HTMLResponse((STATIC_DIR / 'nurse.html').read_text())


@app.get('/patient', response_class=HTMLResponse)
async def patient_dashboard():
    return HTMLResponse((STATIC_DIR / 'patient.html').read_text())


@app.websocket('/ws')
async def websocket_endpoint(ws: WebSocket):
    await manager.connect(ws)
    try:
        while True:
            await ws.receive_text()
    except WebSocketDisconnect:
        manager.disconnect(ws)


def main():
    uvicorn.run('backend.main:app', host='0.0.0.0', port=8080, reload=False)
