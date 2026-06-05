"""Vision AI & 카메라 스트림 API."""

from fastapi import APIRouter, HTTPException
from fastapi.responses import StreamingResponse
import time

from .. import drawer_metadata

router = APIRouter(prefix='/api/vision', tags=['vision'])


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


# ── 카메라 스트림 (MJPEG, 현재는 플레이스홀더) ──────────────────────────────

_mock_frame_counter = 0


def _generate_placeholder_mjpeg():
    """카메라 연결 전까지 플레이스홀더 JPEG 생성."""
    global _mock_frame_counter
    import io
    from PIL import Image, ImageDraw

    while True:
        _mock_frame_counter += 1
        img = Image.new('RGB', (640, 480), color=(200, 200, 200))
        draw = ImageDraw.Draw(img)

        # 카메라 없음 표시
        draw.text((150, 200), 'RealSense D455', fill=(100, 100, 100), font=None)
        draw.text((140, 240), '카메라 연결 대기 중...', fill=(150, 150, 150), font=None)
        draw.text((200, 400), f'Frame: {_mock_frame_counter}', fill=(100, 100, 100), font=None)

        # JPEG로 인코딩
        buf = io.BytesIO()
        img.save(buf, format='JPEG', quality=80)
        buf.seek(0)
        jpeg_data = buf.getvalue()

        # MJPEG 경계선
        header = f'--frame\r\nContent-Type: image/jpeg\r\nContent-length: {len(jpeg_data)}\r\n\r\n'.encode()
        yield header + jpeg_data + b'\r\n'
        time.sleep(0.03)  # ~30 FPS


@router.get('/stream')
async def camera_stream():
    """MJPEG 카메라 스트림 (현재는 placeholder)."""
    return StreamingResponse(
        _generate_placeholder_mjpeg(),
        media_type='multipart/x-mixed-replace; boundary=frame'
    )


# ── Vision AI 결과 (현재는 mock) ──────────────────────────────────────────

_mock_detections = {
    'drawer_001': {'confidence': 0.95, 'aruco': 10, 'detected_at': None},
    'drawer_002': {'confidence': 0.88, 'aruco': 11, 'detected_at': None},
    'drawer_003': {'confidence': 0.92, 'aruco': 20, 'detected_at': None},
}


@router.get('/detections')
async def get_detections():
    """최근 Vision AI 객체 인식 결과 (ArUco 마커)."""
    return {
        'timestamp': __import__('datetime').datetime.now().isoformat(),
        'detections': _mock_detections,
        'status': 'OFFLINE',  # 카메라 연결 후 ONLINE으로 변경
    }


@router.post('/detections/update')
async def update_detections(data: dict):
    """외부 Vision 노드에서 결과 푸시."""
    global _mock_detections
    _mock_detections.update(data.get('detections', {}))
    return {'success': True, 'count': len(_mock_detections)}
