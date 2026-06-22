"""Vision AI & camera stream API backed by hospital_web's camera manager."""

import asyncio
import datetime

from fastapi import APIRouter, HTTPException
from fastapi.responses import StreamingResponse

from .. import camera as cam_module
from .. import drawer_metadata

router = APIRouter(prefix='/api/vision', tags=['vision'])


@router.get('/drawers')
async def get_drawers():
    return {
        'grid': {'rows': 3, 'cols': 2},
        'drawers': drawer_metadata.get_all_drawers(),
    }


@router.get('/drawer/{drawer_id}')
async def get_drawer(drawer_id: str):
    info = drawer_metadata.get_drawer_by_id(drawer_id)
    if not info:
        raise HTTPException(status_code=404, detail='drawer not found')
    return info


@router.get('/aruco/{aruco_id}')
async def get_by_aruco(aruco_id: int):
    info = drawer_metadata.get_drawer_by_aruco(aruco_id)
    if not info:
        raise HTTPException(status_code=404, detail='aruco not found')
    return info


async def _placeholder_mjpeg():
    import io
    from PIL import Image, ImageDraw

    n = 0
    while True:
        n += 1
        img = Image.new('RGB', (640, 480), color=(200, 200, 200))
        draw = ImageDraw.Draw(img)
        draw.text((150, 200), 'RealSense D455', fill=(100, 100, 100), font=None)
        draw.text((125, 240), cam_module.camera.error or 'camera frame unavailable',
                  fill=(150, 150, 150), font=None)
        draw.text((220, 400), f'Frame: {n}', fill=(100, 100, 100), font=None)

        buf = io.BytesIO()
        img.save(buf, format='JPEG', quality=80)
        jpeg = buf.getvalue()
        yield (
            b'--frame\r\nContent-Type: image/jpeg\r\n'
            b'Content-Length: ' + str(len(jpeg)).encode() + b'\r\n\r\n'
            + jpeg + b'\r\n'
        )
        await asyncio.sleep(0.1)


@router.get('/stream')
async def camera_stream():
    async def generate():
        last_frame = None
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
            else:
                async for chunk in _placeholder_mjpeg():
                    yield chunk
                    if cam_module.camera.get_jpeg():
                        break

    return StreamingResponse(
        generate(),
        media_type='multipart/x-mixed-replace; boundary=frame',
    )


@router.get('/detections')
async def get_detections():
    error = cam_module.camera.detector_error or cam_module.camera.error
    return {
        'timestamp': datetime.datetime.now().isoformat(),
        'detections': cam_module.camera.get_detections(),
        'model_loaded': cam_module.camera.detector_loaded,
        'status': 'ONLINE' if cam_module.camera.detector_loaded and not error else 'OFFLINE',
        'error': error,
    }


@router.get('/markers')
async def get_markers():
    return {
        'timestamp': datetime.datetime.now().isoformat(),
        'markers': cam_module.camera.get_aruco_markers(),
        'error': cam_module.camera.error,
    }
