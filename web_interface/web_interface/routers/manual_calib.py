"""
수동 포인트 캘리브레이션.

워크플로:
  1. ID 0 마커가 link_6에 부착된 상태로 로봇 이동
  2. 웹 UI에서 현재 TCP XYZ 확인 후 입력 (또는 자동 입력)
  3. "캡처" → 카메라 P_cam 자동 기록
  4. 4개 이상 수집 후 "계산" → SVD로 T_base_camera 산출
  5. 잔차 확인 후 "저장"

P_base_marker = TCP_xyz + gripper_to_marker_xyz (yaml에서 자동 로드, 근사값)
"""

import asyncio
import random
import threading
from pathlib import Path

import numpy as np
import yaml
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

from .. import camera as cam_module
from .. import ros_node as ros

router = APIRouter(tags=['manual_calib'], prefix='/api/manual_calib')

_CALIB_PATH = Path('/home/cheol/RoToSY_ws/src/rotosy_calibration/config/camera_extrinsic.yaml')
_MIN_POINTS = 4

_lock   = threading.Lock()
_points: list = []  # [{label, p_tcp_mm, p_cam_m, tcp_orient_deg}]
_result: dict = {}

_auto_lock   = threading.Lock()
_auto_status: dict = {'running': False, 'done': True, 'progress': 0, 'total': 0, 'captured': 0, 'log': []}


# ── 유틸 ──────────────────────────────────────────────────────────────────────

def _load_gripper_to_marker_mm() -> np.ndarray:
    """camera_extrinsic.yaml에서 gripper_to_marker_xyz 로드 (mm 변환)."""
    try:
        data = yaml.safe_load(_CALIB_PATH.read_text())
        gm = data['hand_eye_result']['ros__parameters']['gripper_to_marker_xyz']
        return np.array(gm, dtype=float) * 1000.0
    except Exception:
        return np.zeros(3)


def _euler_to_rotation(rx_deg: float, ry_deg: float, rz_deg: float) -> np.ndarray:
    """DSR extrinsic XYZ → 회전 행렬.  R = Rz @ Ry @ Rx."""
    rx, ry, rz = np.radians(rx_deg), np.radians(ry_deg), np.radians(rz_deg)
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(rx), -np.sin(rx)],
                   [0, np.sin(rx),  np.cos(rx)]])
    Ry = np.array([[ np.cos(ry), 0, np.sin(ry)],
                   [0,           1, 0          ],
                   [-np.sin(ry), 0, np.cos(ry)]])
    Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                   [np.sin(rz),  np.cos(rz), 0],
                   [0,           0,          1]])
    return Rz @ Ry @ Rx


def _svd_rigid(P_cam: np.ndarray, P_base: np.ndarray):
    """P_base ≈ R @ P_cam + t (모두 미터 단위). SVD 최소자승."""
    c_c = P_cam.mean(0)
    c_b = P_base.mean(0)
    H   = (P_cam - c_c).T @ (P_base - c_b)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = Vt.T @ U.T
    t = c_b - R @ c_c
    residuals = [float(np.linalg.norm(R @ pc + t - pb))
                 for pc, pb in zip(P_cam, P_base)]
    return R, t, residuals


def _rot_to_quat(R: np.ndarray) -> list:
    trace = R[0,0] + R[1,1] + R[2,2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        return [(R[2,1]-R[1,2])*s, (R[0,2]-R[2,0])*s, (R[1,0]-R[0,1])*s, 0.25/s]
    if R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = 2.0 * np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
        return [0.25*s, (R[0,1]+R[1,0])/s, (R[0,2]+R[2,0])/s, (R[2,1]-R[1,2])/s]
    if R[1,1] > R[2,2]:
        s = 2.0 * np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
        return [(R[0,1]+R[1,0])/s, 0.25*s, (R[1,2]+R[2,1])/s, (R[0,2]-R[2,0])/s]
    s = 2.0 * np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
    return [(R[0,2]+R[2,0])/s, (R[1,2]+R[2,1])/s, 0.25*s, (R[1,0]-R[0,1])/s]


# ── 요청 모델 ─────────────────────────────────────────────────────────────────

class CaptureRequest(BaseModel):
    x_tcp_mm: float
    y_tcp_mm: float
    z_tcp_mm: float
    marker_id: int = 0
    label: str = ''


# ── 엔드포인트 ────────────────────────────────────────────────────────────────

@router.get('/status')
def get_status():
    with _lock:
        return {
            'count':        len(_points),
            'min_required': _MIN_POINTS,
            'points':       list(_points),
            'result':       dict(_result),
        }


@router.post('/capture')
def capture(req: CaptureRequest):
    """TCP XYZ(사용자 입력) + 카메라 마커 P_cam 캡처."""
    markers = cam_module.camera.get_aruco_markers()
    m = next((x for x in markers if x['id'] == req.marker_id), None)

    if m is None:
        raise HTTPException(status_code=404,
                            detail=f'Marker ID {req.marker_id} 미감지 — 마커가 카메라에 보이는지 확인')
    if m.get('x_cam_m') is None:
        raise HTTPException(status_code=422, detail='Depth 값 없음 — 카메라와의 거리 확인')

    # 현재 TCP 자세(rx, ry, rz)를 ROS 노드에서 읽어 저장 — compute 시 포인트별 gm_offset 변환에 사용
    orient = [0.0, 0.0, 0.0]
    node = ros.get_node()
    if node is not None:
        tcp_state = node.get_state().get('current_tcp', [])
        if len(tcp_state) >= 6:
            orient = [float(tcp_state[3]), float(tcp_state[4]), float(tcp_state[5])]

    entry = {
        'label':          req.label or f'pt{len(_points)+1}',
        'p_tcp_mm':       [req.x_tcp_mm, req.y_tcp_mm, req.z_tcp_mm],
        'p_cam_m':        [m['x_cam_m'], m['y_cam_m'], m['z_cam_m']],
        'tcp_orient_deg': orient,
    }
    with _lock:
        _points.append(entry)
        count = len(_points)

    return {'success': True, 'count': count, 'entry': entry}


@router.post('/capture_auto')
def capture_auto(marker_id: int = 0):
    """로봇 현재 TCP를 자동으로 읽어서 캡처 (입력 없이 원클릭)."""
    node = ros.get_node()
    if node is None:
        raise HTTPException(status_code=503, detail='ROS2 노드 미연결')

    tcp = node.get_state().get('current_tcp', [])
    if len(tcp) < 3 or all(v == 0.0 for v in tcp[:3]):
        raise HTTPException(status_code=422, detail='TCP 위치 미수신')

    markers = cam_module.camera.get_aruco_markers()
    m = next((x for x in markers if x['id'] == marker_id), None)
    if m is None:
        raise HTTPException(status_code=404,
                            detail=f'Marker ID {marker_id} 미감지')
    if m.get('x_cam_m') is None:
        raise HTTPException(status_code=422, detail='Depth 값 없음')

    entry = {
        'label':          f'pt{len(_points)+1}',
        'p_tcp_mm':       [float(tcp[0]), float(tcp[1]), float(tcp[2])],
        'p_cam_m':        [m['x_cam_m'], m['y_cam_m'], m['z_cam_m']],
        'tcp_orient_deg': [float(tcp[3]), float(tcp[4]), float(tcp[5])],
    }
    with _lock:
        _points.append(entry)
        count = len(_points)

    return {'success': True, 'count': count, 'entry': entry}


@router.delete('/point/{index}')
def delete_point(index: int):
    with _lock:
        if index < 0 or index >= len(_points):
            raise HTTPException(status_code=404, detail='인덱스 범위 초과')
        removed = _points.pop(index)
    return {'success': True, 'removed': removed}


@router.delete('/reset')
def reset_points():
    with _lock:
        _points.clear()
        _result.clear()
    return {'success': True}


class ComputeRequest(BaseModel):
    rx_deg:  float = 90.0
    ry_deg:  float = -90.0
    rz_deg:  float = 0.0
    # TCP → 마커 오프셋 (tool 좌표계, mm)
    # 마커→TCP 가 y=-55mm, z=-50mm 이면 TCP→마커 = y=+55, z=+50
    gm_x_mm: float = 0.0
    gm_y_mm: float = 55.0
    gm_z_mm: float = 50.0


@router.post('/compute')
def compute_calibration(req: ComputeRequest):
    with _lock:
        pts = list(_points)

    if len(pts) < _MIN_POINTS:
        raise HTTPException(status_code=422,
                            detail=f'{_MIN_POINTS}개 이상 필요 (현재 {len(pts)}개)')

    gm_mm     = np.array([req.gm_x_mm, req.gm_y_mm, req.gm_z_mm])
    R_tcp_fix = _euler_to_rotation(req.rx_deg, req.ry_deg, req.rz_deg)  # 저장된 자세 없을 때 fallback

    # 포인트마다 캡처 시점의 실제 TCP 자세로 gm_offset을 월드 좌표계로 변환
    P_base_list = []
    for p in pts:
        orient = p.get('tcp_orient_deg')
        R_i = _euler_to_rotation(orient[0], orient[1], orient[2]) if orient else R_tcp_fix
        gm_world_i = R_i @ gm_mm
        P_base_list.append((np.array(p['p_tcp_mm']) + gm_world_i) / 1000.0)

    P_base = np.array(P_base_list)
    P_cam  = np.array([p['p_cam_m'] for p in pts], dtype=float)

    R, t, residuals = _svd_rigid(P_cam, P_base)
    q = _rot_to_quat(R)

    res = {
        'translation_xyz':       t.tolist(),
        'rotation_quat_xyzw':    q,
        'mean_residual_mm':      float(np.mean(residuals) * 1000),
        'max_residual_mm':       float(np.max(residuals) * 1000),
        'per_point_residual_mm': [round(r * 1000, 2) for r in residuals],
        'samples':               len(pts),
        'gm_offset_tool_mm':     gm_mm.tolist(),
    }
    with _lock:
        _result.update(res)

    return res


@router.post('/save')
def save_calibration():
    with _lock:
        if not _result:
            raise HTTPException(status_code=422, detail='먼저 /compute 실행 필요')
        r = dict(_result)

    t = r['translation_xyz']
    q = r['rotation_quat_xyzw']

    # 기존 hand_eye_result 보존
    hey_block = ''
    if _CALIB_PATH.exists():
        try:
            raw = yaml.safe_load(_CALIB_PATH.read_text())
            hp  = raw.get('hand_eye_result', {}).get('ros__parameters', {})
            if hp:
                hey_block = (
                    '\nhand_eye_result:\n'
                    '  ros__parameters:\n'
                    f'    gripper_frame: "{hp.get("gripper_frame","link_6")}"\n'
                    f'    marker_frame: "{hp.get("marker_frame","calibration_aruco_marker")}"\n'
                    f'    gripper_to_marker_xyz: {hp.get("gripper_to_marker_xyz",[0,0,0])}\n'
                    f'    gripper_to_marker_quat_xyzw: {hp.get("gripper_to_marker_quat_xyzw",[0,0,0,1])}\n'
                    f'    marker_id: {hp.get("marker_id",0)}\n'
                    f'    marker_size_m: {hp.get("marker_size_m",0.05)}\n'
                )
        except Exception:
            pass

    content = (
        'camera_extrinsic:\n'
        '  ros__parameters:\n'
        '    base_frame: "base_link"\n'
        '    camera_frame: "camera_color_optical_frame"\n'
        f'    translation_xyz: [{t[0]:.8f}, {t[1]:.8f}, {t[2]:.8f}]\n'
        f'    rotation_quat_xyzw: [{q[0]:.8f}, {q[1]:.8f}, {q[2]:.8f}, {q[3]:.8f}]\n'
        f'\n# manual_calib: samples={r["samples"]}'
        f'  mean={r["mean_residual_mm"]:.2f}mm'
        f'  max={r["max_residual_mm"]:.2f}mm\n'
    ) + hey_block

    _CALIB_PATH.write_text(content, encoding='utf-8')
    return {'success': True, 'saved_to': str(_CALIB_PATH), **r}


# ── 자동 캘리브레이션 ─────────────────────────────────────────────────────────

class AutoRunRequest(BaseModel):
    x_min: float = 310.0
    x_max: float = 460.0
    y_min: float = -154.0
    y_max: float = 350.0
    z_min: float = 136.0
    z_max: float = 460.0
    count: int = 20
    velocity: float = 30.0
    acceleration: float = 60.0


async def _run_auto_calib(req: AutoRunRequest) -> None:
    node = ros.get_node()
    tcp0 = node.get_state().get('current_tcp', [0.0] * 6)
    rx_fix, ry_fix, rz_fix = float(tcp0[3]), float(tcp0[4]), float(tcp0[5])

    captured  = 0
    attempts  = 0
    max_tries = req.count * 3

    while captured < req.count and attempts < max_tries:
        with _auto_lock:
            if not _auto_status['running']:
                break

        attempts += 1
        xr = random.uniform(req.x_min, req.x_max)
        yr = random.uniform(req.y_min, req.y_max)
        zr = random.uniform(req.z_min, req.z_max)

        with _auto_lock:
            _auto_status['log'].append(f'[{attempts}] → ({xr:.0f}, {yr:.0f}, {zr:.0f}) mm')

        result = await node.call_movel(
            [xr, yr, zr, rx_fix, ry_fix, rz_fix],
            req.velocity, req.acceleration,
        )

        if not result.get('success'):
            with _auto_lock:
                _auto_status['log'].append(f'  ✗ {result.get("message", "이동 실패")}')
            await asyncio.sleep(0.3)
            continue

        await asyncio.sleep(0.4)  # 카메라 안정화

        markers = cam_module.camera.get_aruco_markers()
        mk = next((m for m in markers if m['id'] == 0), None)

        if mk is None or mk.get('x_cam_m') is None:
            with _auto_lock:
                _auto_status['log'].append('  ✗ 마커 미감지 — 스킵')
            continue

        cur = node.get_state().get('current_tcp', [0.0] * 6)
        entry = {
            'label':          f'auto{captured + 1}',
            'p_tcp_mm':       [float(cur[0]), float(cur[1]), float(cur[2])],
            'p_cam_m':        [mk['x_cam_m'], mk['y_cam_m'], mk['z_cam_m']],
            'tcp_orient_deg': [float(cur[3]), float(cur[4]), float(cur[5])],
        }
        with _lock:
            _points.append(entry)
            captured += 1

        with _auto_lock:
            _auto_status['progress'] = captured
            _auto_status['log'].append(f'  ✓ {captured}/{req.count} 캡처 완료')

    with _auto_lock:
        _auto_status['running'] = False
        _auto_status['done']    = True
        _auto_status['captured'] = captured
        tail = (f'완료! {captured}개 수집' if captured >= req.count
                else f'종료 — {captured}/{req.count}개 수집 (시도 {attempts}회)')
        _auto_status['log'].append(tail)


@router.post('/auto_run')
async def auto_run(req: AutoRunRequest):
    global _auto_status
    node = ros.get_node()
    if node is None:
        raise HTTPException(status_code=503, detail='ROS2 노드 미연결')

    with _auto_lock:
        if _auto_status.get('running'):
            raise HTTPException(status_code=409, detail='이미 자동 캘리브레이션 실행 중')
        _auto_status = {
            'running': True, 'done': False,
            'progress': 0, 'total': req.count, 'captured': 0,
            'log': ['자동 캘리브레이션 시작...'],
        }

    asyncio.create_task(_run_auto_calib(req))
    return {'started': True}


@router.post('/auto_stop')
def auto_stop():
    with _auto_lock:
        _auto_status['running'] = False
        _auto_status['log'].append('사용자 중지 요청 (현재 이동 완료 후 정지)')
    return {'stopped': True}


@router.get('/auto_status')
def get_auto_status():
    with _auto_lock:
        return dict(_auto_status)
