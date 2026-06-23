"""
Hospital Web Gateway — Admin Dashboard backend.
Port: 8080

WebSocket /ws broadcasts combined state at 10 Hz:
  { robot, amr, door, mission, nodes, plc, arduino }
"""

import asyncio
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Set

import uvicorn
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles

from . import robot_proxy as proxy
from . import ros_bridge   as ros
from . import mission_state as ms
from . import camera as cam_module
from . import orchestrator
from .routers import (robot, amr, system as sys_router, prescription as presc_router,
                      sensor as sensor_router, vision as vision_router, demo as demo_router,
                      patient as patient_router, medicine as medicine_router, ocr as ocr_router,
                      manual_calib as manual_calib_router, orchestrator as orchestrator_router,
                      pallet as pallet_router, web_compat as web_compat_router)
from . import sensor_db
from . import db_schema

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
    # 'arduino' 필드는 DB(sensor_db)가 단일 소스. 시리얼 리더가 insert_reading()으로
    # drawer_sensors를 upsert하면 get_latest()가 최신값을 들고 옴.
    # DB에 아직 값이 없으면 ros_bridge mock으로 폴백.
    arduino = sensor_db.get_latest() or bridge['arduino']
    return {
        'robot':        robot_st,
        'robot_online': proxy.is_online(),
        'amr':          bridge['amr'],
        'door':         bridge['door'],
        'mission':      ms.get_mission(),
        'nodes':        bridge['nodes'],
        'plc':          {'status': 'DISCONNECTED'},
        'arduino':      arduino,
        'motion_step':  robot_st.get('seq_step', 'IDLE'),
        'magnet_on':    robot_st.get('magnet_on', False),
        'marker_queue': ms.get_marker_queue(),
        'orchestrator': orchestrator.get_state(),
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
    db_schema.init_schema()
    orchestrator.load_state_from_db()
    sensor_db.init_db()
    sensor_db.start_serial_reader()
    ros.init()
    cam_module.camera.start()
    broadcast_task = asyncio.create_task(_broadcast_loop())
    poll_task      = asyncio.create_task(proxy.poll_loop())
    orch_task      = asyncio.create_task(orchestrator.monitor_loop())
    yield
    broadcast_task.cancel()
    poll_task.cancel()
    orch_task.cancel()
    cam_module.camera.stop()
    sensor_db.stop_serial_reader()
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
app.include_router(patient_router.router)
app.include_router(medicine_router.router)
app.include_router(ocr_router.router)
app.include_router(manual_calib_router.router)
app.include_router(orchestrator_router.router)
app.include_router(pallet_router.router)
app.include_router(web_compat_router.router)

if STATIC_DIR.exists():
    app.mount('/static', StaticFiles(directory=str(STATIC_DIR)), name='static')


@app.get('/api/status')
async def robot_status():
    """Compatibility endpoint for the old web_interface robot state API."""
    return proxy.get_robot_state()


@app.get('/camera/stream')
async def camera_stream():
    """Compatibility MJPEG stream endpoint previously owned by web_interface."""
    if not cam_module.camera.available:
        raise HTTPException(status_code=503, detail='pyrealsense2 not installed')

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
    """Compatibility ArUco marker endpoint previously owned by web_interface."""
    return {
        'markers': cam_module.camera.get_aruco_markers(),
        'error': cam_module.camera.error,
    }


@app.get('/camera/detections')
async def camera_detections() -> dict:
    """Compatibility YOLO detections endpoint previously owned by web_interface."""
    return {
        'detections': cam_module.camera.get_detections(),
        'model_loaded': cam_module.camera.detector_loaded,
        'error': cam_module.camera.detector_error,
    }


@app.get('/camera/snapshot')
async def camera_snapshot():
    """Compatibility latest JPEG snapshot endpoint previously owned by web_interface."""
    frame = cam_module.camera.get_jpeg()
    if frame is None:
        raise HTTPException(status_code=503, detail='카메라 프레임 없음')
    return Response(content=frame, media_type='image/jpeg')


@app.get('/', response_class=HTMLResponse)
async def admin_dashboard():
    return HTMLResponse((STATIC_DIR / 'admin.html').read_text())


@app.get('/admin/test', response_class=HTMLResponse)
async def amr_test_page():
    return HTMLResponse((STATIC_DIR / 'amr_test.html').read_text())


@app.get('/pharmacist', response_class=HTMLResponse)
async def pharmacist_dashboard():
    return HTMLResponse((STATIC_DIR / 'pharmacist.html').read_text())


@app.get('/nurse', response_class=HTMLResponse)
async def nurse_dashboard():
    return HTMLResponse((STATIC_DIR / 'nurse.html').read_text())


@app.get('/patient', response_class=HTMLResponse)
async def patient_dashboard():
    return HTMLResponse((STATIC_DIR / 'patient.html').read_text())


@app.get('/pallet', response_class=HTMLResponse)
async def pallet_viewer():
    return HTMLResponse((STATIC_DIR / 'pallet.html').read_text())


@app.get('/test', response_class=HTMLResponse)
async def integration_test_page():
    return HTMLResponse((STATIC_DIR / 'test.html').read_text())


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
