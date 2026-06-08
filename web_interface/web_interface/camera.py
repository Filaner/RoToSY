"""
RealSense camera manager.

- Color + Depth 스트림 캡처
- RealSense post-processing (spatial / temporal / hole-filling) 적용
- ArUco 마커 검출 → 픽셀 좌표 + 3D 좌표
- rs2_deproject_pixel_to_point (왜곡 보정) → 카메라 3D 좌표
- camera_extrinsic.yaml (T_base_camera) → 로봇 베이스 좌표 (mm)
"""

import threading
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import yaml

try:
    import pyrealsense2 as rs
    _RS_AVAILABLE = True
except ImportError:
    _RS_AVAILABLE = False

# ── calibration YAML 경로 ────────────────────────────────────────────────────
_CALIB_CANDIDATES = [
    Path('/home/cheol/RoToSY_ws/install/rotosy_calibration/share/rotosy_calibration/config/camera_extrinsic.yaml'),
    Path('/home/cheol/RoToSY_ws/src/rotosy_calibration/config/camera_extrinsic.yaml'),
]

def _find_calib_yaml() -> Optional[Path]:
    for p in _CALIB_CANDIDATES:
        if p.exists():
            return p
    return None

def _load_calib(yaml_path: Path) -> tuple:
    """camera_extrinsic.yaml → (T_base_camera 4×4, marker_size_m float).

    marker_size_m은 hand_eye_result 섹션에서 읽으며 없으면 0.05(5cm) 기본값.
    """
    T = None
    marker_size_m = 0.05
    try:
        with open(yaml_path) as f:
            data = yaml.safe_load(f)

        params = data['camera_extrinsic']['ros__parameters']
        t = np.array(params['translation_xyz'], dtype=float)
        q = params['rotation_quat_xyzw']
        x, y, z, w = q
        R = np.array([
            [1-2*(y*y+z*z),   2*(x*y-w*z),   2*(x*z+w*y)],
            [  2*(x*y+w*z), 1-2*(x*x+z*z),   2*(y*z-w*x)],
            [  2*(x*z-w*y),   2*(y*z+w*x), 1-2*(x*x+y*y)],
        ], dtype=float)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3]  = t

        try:
            her = data['hand_eye_result']['ros__parameters']
            marker_size_m = float(her.get('marker_size_m', marker_size_m))
        except (KeyError, TypeError):
            pass

    except Exception as exc:
        print(f'[camera] calib load failed: {exc}')

    return T, marker_size_m


class CameraManager:
    def __init__(self, width: int = 640, height: int = 480, fps: int = 30):
        self._width  = width
        self._height = height
        self._fps    = fps

        self._lock  = threading.Lock()
        self._jpeg: Optional[bytes] = None

        self._aruco_lock = threading.Lock()
        self._aruco_markers: list = []

        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._error: Optional[str] = None

        calib_path = _find_calib_yaml()
        self._T_base_camera: Optional[np.ndarray] = None
        self._marker_size_m: float = 0.05
        if calib_path:
            self._T_base_camera, self._marker_size_m = _load_calib(calib_path)
            print(f'[camera] calibration loaded: {calib_path}')
            print(f'[camera] marker_size_m = {self._marker_size_m}')
        else:
            print('[camera] WARNING: camera_extrinsic.yaml not found — robot coords unavailable')

    # ── Public API ────────────────────────────────────────────────────────────

    def start(self) -> None:
        if self._running:
            return
        if not _RS_AVAILABLE:
            self._error = 'pyrealsense2 not installed'
            return
        self._running = True
        self._thread  = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=4.0)

    def get_jpeg(self) -> Optional[bytes]:
        with self._lock:
            return self._jpeg

    def get_aruco_markers(self) -> list:
        with self._aruco_lock:
            return list(self._aruco_markers)

    @property
    def error(self) -> Optional[str]:
        return self._error

    @property
    def available(self) -> bool:
        return _RS_AVAILABLE

    # ── Internal ──────────────────────────────────────────────────────────────

    def _init_aruco(self):
        try:
            dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            params     = cv2.aruco.DetectorParameters()
            return cv2.aruco.ArucoDetector(dictionary, params)
        except Exception:
            return None

    @staticmethod
    def _sample_depth_patch(
        depth_data: np.ndarray,
        depth_scale: float,
        cx: int, cy: int,
        radius: int = 5,
    ) -> Optional[float]:
        h, w = depth_data.shape
        x1 = max(0, cx - radius)
        x2 = min(w, cx + radius + 1)
        y1 = max(0, cy - radius)
        y2 = min(h, cy + radius + 1)
        patch = depth_data[y1:y2, x1:x2].astype(np.float32) * depth_scale
        valid = patch[(patch > 0.05) & (patch < 5.0)]
        return float(np.median(valid)) if len(valid) > 0 else None

    def _capture_loop(self) -> None:
        import time as _t
        aruco_detector = self._init_aruco()
        _retry_delay = 5.0   # 지수 백오프 초기값

        while self._running:
            # ── 파이프라인 시작 (실패 시 지수 백오프 재시도) ──────────────────
            pipeline = rs.pipeline()
            config   = rs.config()
            config.enable_stream(rs.stream.depth, self._width, self._height, rs.format.z16,  self._fps)
            config.enable_stream(rs.stream.color, self._width, self._height, rs.format.bgr8, self._fps)

            try:
                profile = pipeline.start(config)
                _retry_delay = 5.0   # 연결 성공 시 초기화
            except Exception as exc:
                self._error = '카메라 연결 대기 중...'
                print(f'[camera] 시작 실패: {exc} — {_retry_delay:.0f}초 후 재시도')
                # 세분화된 슬립으로 GIL 해제 빈도 높임
                deadline = _t.monotonic() + _retry_delay
                while _t.monotonic() < deadline and self._running:
                    _t.sleep(0.5)
                _retry_delay = min(_retry_delay * 1.5, 60.0)
                continue

            try:
                color_profile = profile.get_stream(rs.stream.color)
                intrinsics    = color_profile.as_video_stream_profile().get_intrinsics()
                depth_scale   = profile.get_device().first_depth_sensor().get_depth_scale()
            except Exception as exc:
                print(f'[camera] 초기화 실패: {exc}')
                pipeline.stop()
                _t.sleep(3.0)
                continue

            # ArUco solvePnP용 카메라 행렬 및 마커 오브젝트 포인트 (파이프라인당 1회)
            _cam_matrix = np.array([
                [intrinsics.fx, 0,             intrinsics.ppx],
                [0,             intrinsics.fy, intrinsics.ppy],
                [0,             0,             1             ],
            ], dtype=np.float64)
            _dist_coeffs = np.array(intrinsics.coeffs, dtype=np.float64)
            _half = self._marker_size_m / 2.0
            _obj_pts = np.array([
                [-_half,  _half, 0],
                [ _half,  _half, 0],
                [ _half, -_half, 0],
                [-_half, -_half, 0],
            ], dtype=np.float32)

            spatial      = rs.spatial_filter()
            spatial.set_option(rs.option.filter_magnitude,    2)
            spatial.set_option(rs.option.filter_smooth_alpha, 0.5)
            spatial.set_option(rs.option.filter_smooth_delta, 20)
            temporal     = rs.temporal_filter()
            temporal.set_option(rs.option.filter_smooth_alpha, 0.4)
            temporal.set_option(rs.option.filter_smooth_delta, 20)
            hole_filling = rs.hole_filling_filter()
            align        = rs.align(rs.stream.color)
            self._error  = None
            print('[camera] 연결됨')

            # ── 캡처 루프 (연결 끊김 시 outer loop로 빠져나와 재연결) ─────────
            consecutive_errors = 0
            try:
                while self._running:
                    try:
                        frames = pipeline.wait_for_frames(timeout_ms=500)
                        consecutive_errors = 0
                    except RuntimeError:
                        consecutive_errors += 1
                        if consecutive_errors >= 3:
                            print('[camera] 연결 끊김 감지 — 재연결 시도')
                            self._error = '카메라 연결 끊김 — 재연결 중'
                            break
                        continue
                    except Exception as exc:
                        print(f'[camera] 프레임 오류: {exc} — 재연결 시도')
                        self._error = '카메라 연결 끊김 — 재연결 중'
                        break

                    aligned     = align.process(frames)
                    depth_raw   = aligned.get_depth_frame()
                    color_frame = aligned.get_color_frame()
                    if not color_frame or not depth_raw:
                        continue

                    depth_filtered = spatial.process(depth_raw)
                    depth_filtered = temporal.process(depth_filtered)
                    depth_filtered = hole_filling.process(depth_filtered)

                    depth_data = np.asanyarray(depth_filtered.get_data())
                    frame      = np.asanyarray(color_frame.get_data())

                    markers_out: list = []

                    # ── ArUco 검출 ───────────────────────────────────────────
                    if aruco_detector is not None:
                        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                        corners, ids, _ = aruco_detector.detectMarkers(gray)

                        if ids is not None:
                            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                            for i, marker_id in enumerate(ids.flatten()):
                                cx = int(corners[i][0][:, 0].mean())
                                cy = int(corners[i][0][:, 1].mean())

                                # 3D 좌표: depth sensor + rs2_deproject (기존 캘리브레이션과 동일 방식)
                                p_cam = None
                                robot = None
                                depth_m = self._sample_depth_patch(
                                    depth_data, depth_scale, cx, cy, radius=12
                                )
                                if depth_m is not None:
                                    pt = rs.rs2_deproject_pixel_to_point(
                                        intrinsics, [float(cx), float(cy)], depth_m
                                    )
                                    p_cam = (float(pt[0]), float(pt[1]), float(pt[2]))
                                    if self._T_base_camera is not None:
                                        p_h    = np.array([pt[0], pt[1], pt[2], 1.0])
                                        p_base = self._T_base_camera @ p_h
                                        robot  = (float(p_base[0]*1000), float(p_base[1]*1000), float(p_base[2]*1000))

                                # rvec: solvePnPGeneric으로 취득 (hand-eye 캘리브레이션 UI 전용)
                                rvec_flat = None
                                img_pts = corners[i][0].astype(np.float32)
                                n_sols, rvecs_s, _, _ = cv2.solvePnPGeneric(
                                    _obj_pts, img_pts, _cam_matrix, _dist_coeffs,
                                    flags=cv2.SOLVEPNP_IPPE_SQUARE,
                                )
                                if n_sols > 0:
                                    chosen_rvec = rvecs_s[0]
                                    for k in range(n_sols):
                                        R_k, _ = cv2.Rodrigues(rvecs_s[k])
                                        if R_k[2, 2] < 0:
                                            chosen_rvec = rvecs_s[k]
                                            break
                                    rv = chosen_rvec.flatten()
                                    rvec_flat = [float(rv[0]), float(rv[1]), float(rv[2])]

                                markers_out.append({
                                    'id':      int(marker_id),
                                    'px':      cx,
                                    'py':      cy,
                                    'x_cam_m': round(p_cam[0], 6) if p_cam else None,
                                    'y_cam_m': round(p_cam[1], 6) if p_cam else None,
                                    'z_cam_m': round(p_cam[2], 6) if p_cam else None,
                                    'rvec':    rvec_flat,
                                    'x_mm':    round(robot[0], 1) if robot else None,
                                    'y_mm':    round(robot[1], 1) if robot else None,
                                    'z_mm':    round(robot[2], 1) if robot else None,
                                })

                                label = f"ID {marker_id}"
                                if robot:
                                    label += f"  ({robot[0]:.0f},{robot[1]:.0f},{robot[2]:.0f})mm"
                                cv2.putText(
                                    frame, label,
                                    (cx - 30, max(20, cy - 14)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.48,
                                    (0, 255, 255), 2, cv2.LINE_AA,
                                )

                    with self._aruco_lock:
                        self._aruco_markers = markers_out

                    ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    if ok:
                        with self._lock:
                            self._jpeg = jpeg.tobytes()

            finally:
                try:
                    pipeline.stop()
                except Exception:
                    pass

            if self._running:
                _t.sleep(2.0)


# Module-level singleton
camera = CameraManager()
