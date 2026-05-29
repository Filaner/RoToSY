"""
FastAPI application entry point.

Architecture:
  - rclpy spins in a daemon thread (see ros_node.py)
  - /ws WebSocket streams robot state at 10 Hz to all connected browsers
  - /api/* REST endpoints for one-shot queries and future commands
  - Static files served from web_interface/static/
"""

import asyncio
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Set

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles

from . import ros_node as ros
from .routers import status as status_router
from .routers import control as control_router

STATIC_DIR = Path(__file__).parent / 'static'
BROADCAST_HZ = 10  # state push rate to WebSocket clients


# ── WebSocket connection manager ──────────────────────────────────────────────

class ConnectionManager:
    """Thread-safe set of active WebSocket connections."""

    def __init__(self):
        self._clients: Set[WebSocket] = set()

    async def connect(self, ws: WebSocket) -> None:
        await ws.accept()
        self._clients.add(ws)

    def disconnect(self, ws: WebSocket) -> None:
        self._clients.discard(ws)

    async def broadcast(self, data: dict) -> None:
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


async def _broadcast_loop() -> None:
    """Push robot state to all connected WebSocket clients at BROADCAST_HZ."""
    interval = 1.0 / BROADCAST_HZ
    while True:
        if manager.count > 0:
            node = ros.get_node()
            if node:
                await manager.broadcast(node.get_state())
        await asyncio.sleep(interval)


# ── Application lifecycle ─────────────────────────────────────────────────────

@asynccontextmanager
async def lifespan(app: FastAPI):
    ros.init_ros()
    broadcast_task = asyncio.create_task(_broadcast_loop())
    yield
    broadcast_task.cancel()
    ros.shutdown_ros()


# ── FastAPI app ───────────────────────────────────────────────────────────────

app = FastAPI(title='RoToSY Web Interface', version='0.1.0', lifespan=lifespan)

app.include_router(status_router.router, prefix='/api')
app.include_router(control_router.router, prefix='/api')

if STATIC_DIR.exists():
    app.mount('/static', StaticFiles(directory=str(STATIC_DIR)), name='static')


@app.get('/', response_class=HTMLResponse)
async def index() -> HTMLResponse:
    return HTMLResponse((STATIC_DIR / 'index.html').read_text())


@app.websocket('/ws')
async def websocket_endpoint(websocket: WebSocket) -> None:
    await manager.connect(websocket)
    try:
        while True:
            # Keep the connection alive; state is pushed by _broadcast_loop
            await websocket.receive_text()
    except WebSocketDisconnect:
        manager.disconnect(websocket)


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    uvicorn.run('web_interface.main:app', host='0.0.0.0', port=8000, reload=False)
