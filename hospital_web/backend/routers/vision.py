"""Vision AI & 카메라 스트림 API.

카메라/YOLO 는 web_interface(RoToSY, :8000) 가 단독 소유한다.
여기서는 그쪽 출력을 그대로 프록시해서 병원 웹(:8080)에 노출만 한다.
(robot_proxy 와 동일한 패턴 — hospital_web 은 RealSense 를 직접 열지 않는다.)
"""

import asyncio
import datetime

import httpx
from fastapi import APIRouter, HTTPException
from fastapi.responses import StreamingResponse

from .. import drawer_metadata
from .. import robot_proxy

router = APIRouter(prefix='/api/vision', tags=['vision'])

_ROTOSY_BASE = robot_proxy.ROTOSY_BASE   # 'http://localhost:8000'


# ── DB 기반 서랍 정보 (카메라 무관) ───────────────────────────────────────────

@router.get('/drawers')
async def get_drawers():
    """모든 서랍 위치 & ArUco 정보."""
    return {
        'grid': {'rows': 3, 'cols': 2},
        'drawers': drawer_metadata.get_all_drawers(),
    }


@router.get('/drawer/{drawer_id}')
async def get_drawer(drawer_id: str):
    """특정 서랍 정보."""
    info = drawer_metadata.get_drawer_by_id(drawer_id)
    if not info:
        raise HTTPException(status_code=404, detail='drawer not found')
    return info


@router.get('/aruco/{aruco_id}')
async def get_by_aruco(aruco_id: int):
    """ArUco 마커 ID로 서랍 조회."""
    info = drawer_metadata.get_drawer_by_aruco(aruco_id)
    if not info:
        raise HTTPException(status_code=404, detail='aruco not found')
    return info


# ── 카메라 스트림 (web_interface MJPEG relay) ─────────────────────────────────

async def _placeholder_mjpeg():
    """web_interface(:8000) 연결 불가 시 보여줄 대기 화면 (YOLO 박스 영상 대체)."""
    import io
    from PIL import Image, ImageDraw

    n = 0
    while True:
        n += 1
        img = Image.new('RGB', (640, 480), color=(200, 200, 200))
        draw = ImageDraw.Draw(img)
        draw.text((150, 200), 'RealSense D455', fill=(100, 100, 100), font=None)
        draw.text((110, 240), 'web_interface(:8000) 대기 중...', fill=(150, 150, 150), font=None)
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
    """web_interface 의 /camera/stream 을 그대로 중계. 끊기면 placeholder 로 fallback.

    web_interface 가 내보내는 프레임에는 이미 YOLO 검출 박스가 그려져 있으므로,
    이 스트림 하나로 '카메라 영상 + 약품 인식 결과'가 같이 보인다.
    """
    async def relay():
        try:
            async with httpx.AsyncClient(timeout=None) as cli:
                async with cli.stream('GET', f'{_ROTOSY_BASE}/camera/stream') as up:
                    if up.status_code != 200:
                        async for chunk in _placeholder_mjpeg():
                            yield chunk
                        return
                    async for chunk in up.aiter_bytes():
                        yield chunk
        except Exception:
            # web_interface 미기동/끊김 → 대기 화면으로 폴백
            async for chunk in _placeholder_mjpeg():
                yield chunk

    return StreamingResponse(
        relay(),
        media_type='multipart/x-mixed-replace; boundary=frame',
    )


# ── YOLO 약품 검출 결과 (web_interface /camera/detections proxy) ──────────────

@router.get('/detections')
async def get_detections():
    """web_interface 의 실제 YOLO 검출 결과를 프록시.

    반환 detections 는 리스트:
      [{class_name, confidence, bbox, depth_m, camera_position_m, base_position_m, ...}]
    """
    try:
        async with httpx.AsyncClient(base_url=_ROTOSY_BASE,
                                     timeout=httpx.Timeout(3.0)) as cli:
            r = await cli.get('/camera/detections')
            data = r.json()
        return {
            'timestamp':    datetime.datetime.now().isoformat(),
            'detections':   data.get('detections', []),
            'model_loaded': bool(data.get('model_loaded')),
            'status':       'ONLINE' if data.get('model_loaded') else 'OFFLINE',
            'error':        data.get('error'),
        }
    except Exception as exc:
        return {
            'timestamp':    datetime.datetime.now().isoformat(),
            'detections':   [],
            'model_loaded': False,
            'status':       'OFFLINE',
            'error':        f'web_interface(:8000) 연결 불가: {exc}',
        }
