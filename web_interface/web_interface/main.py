"""
FastAPI application entry point.

Architecture:
  - rclpy spins in a daemon thread (see ros_node.py)
  - /ws WebSocket streams robot state at 10 Hz to all connected browsers
  - /api/* REST endpoints for one-shot queries and future commands
  - Static files served from web_interface/static/
"""

import asyncio
import fcntl
import logging
import os
import sys
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Set

# 폴링 엔드포인트 access 로그 무음 처리
_MUTED_PATHS = {
    '/camera/markers',
    '/camera/detections',
    '/camera/stream',
    '/api/manual_calib/touch/status',
    '/api/status',
}

class _MutePollingFilter(logging.Filter):
    def filter(self, record: logging.LogRecord) -> bool:
        msg = record.getMessage()
        return not any(p in msg for p in _MUTED_PATHS)

logging.getLogger('uvicorn.access').addFilter(_MutePollingFilter())

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles

from . import ros_node as ros
from . import camera as cam_module
from .routers import status as status_router
from .routers import control as control_router
from .routers import manual_calib as manual_calib_router
from .routers import gripper as gripper_router
from .routers import inverter as inverter_router

STATIC_DIR = Path(__file__).parent / 'static'
BROADCAST_HZ = 10  # state push rate to WebSocket clients
_INSTANCE_LOCK_FD = None


def _acquire_instance_lock() -> bool:
    """Prevent duplicate web bridges and duplicate ownership of the camera."""
    global _INSTANCE_LOCK_FD
    fd = os.open('/tmp/rotosy_web_server.lock', os.O_CREAT | os.O_RDWR, 0o644)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        os.close(fd)
        return False
    os.ftruncate(fd, 0)
    os.write(fd, str(os.getpid()).encode('ascii'))
    _INSTANCE_LOCK_FD = fd
    return True


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
    # rclpy.init() + DDS 디스커버리가 수 초 걸릴 수 있으므로 스레드에서 실행해
    # asyncio 이벤트 루프를 막지 않는다 → 웹페이지가 즉시 로드됨
    await asyncio.get_running_loop().run_in_executor(None, ros.init_ros)
    cam_module.camera.start()
    broadcast_task = asyncio.create_task(_broadcast_loop())
    yield
    broadcast_task.cancel()
    cam_module.camera.stop()
    ros.shutdown_ros()


# ── FastAPI app ───────────────────────────────────────────────────────────────

app = FastAPI(title='RoToSY Web Interface', version='0.1.0', lifespan=lifespan)

app.include_router(status_router.router, prefix='/api')
app.include_router(control_router.router, prefix='/api')
app.include_router(manual_calib_router.router)
app.include_router(gripper_router.router)
app.include_router(inverter_router.router)

if STATIC_DIR.exists():
    app.mount('/static', StaticFiles(directory=str(STATIC_DIR)), name='static')


@app.get('/', response_class=HTMLResponse)
async def index() -> HTMLResponse:
    return HTMLResponse((STATIC_DIR / 'index.html').read_text())


@app.get('/camera/stream')
async def camera_stream():
    """MJPEG stream from the RealSense camera."""
    from fastapi import HTTPException as _HTTPException
    if not cam_module.camera.available:
        raise _HTTPException(status_code=503, detail='pyrealsense2 not installed')

    async def generate():
        last_frame: bytes | None = None
        while True:
            frame = cam_module.camera.get_jpeg()
            if frame:
                last_frame = frame
            if last_frame:
                yield (
                    b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n'
                    + last_frame +
                    b'\r\n'
                )
            await asyncio.sleep(1 / 30)

    return StreamingResponse(
        generate(),
        media_type='multipart/x-mixed-replace; boundary=frame',
    )


@app.get('/camera/markers')
async def camera_markers() -> dict:
    """Return the latest detected ArUco marker positions."""
    return {
        'markers': cam_module.camera.get_aruco_markers(),
        'error':   cam_module.camera.error,
    }


@app.get('/camera/detections')
async def camera_detections() -> dict:
    """Return the latest stabilized YOLO medicine detections."""
    return {
        'detections': cam_module.camera.get_detections(),
        'model_loaded': cam_module.camera.detector_loaded,
        'error': cam_module.camera.detector_error,
    }


@app.get('/camera/snapshot')
async def camera_snapshot():
    """Return the latest camera frame as a JPEG image."""
    from fastapi import HTTPException as _HTTPException
    from fastapi.responses import Response as _Response
    frame = cam_module.camera.get_jpeg()
    if frame is None:
        raise _HTTPException(status_code=503, detail='카메라 프레임 없음')
    return _Response(content=frame, media_type='image/jpeg')



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
    if not _acquire_instance_lock():
        print(
            'web_server is already running; refusing duplicate instance.',
            file=sys.stderr,
        )
        return
    uvicorn.run('web_interface.main:app', host='0.0.0.0', port=8000, reload=False)
