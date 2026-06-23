"""
RealSense camera manager.

- Color + Depth 스트림 캡처
- RealSense post-processing (spatial / temporal / hole-filling) 적용
- ArUco 마커 검출 → 픽셀 좌표 + 3D 좌표
- rs2_deproject_pixel_to_point (왜곡 보정) → 카메라 3D 좌표
- camera_extrinsic.yaml (T_base_camera) → 로봇 베이스 좌표 (mm)
"""

import os
import queue
import threading
from pathlib import Path
from typing import Optional

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
import cv2
import numpy as np
import yaml

try:
    import pyrealsense2 as rs
    _RS_AVAILABLE = True
except ImportError:
    _RS_AVAILABLE = False

# ── calibration YAML 경로 ────────────────────────────────────────────────────
def _calib_candidates() -> list[Path]:
    candidates = []
    env_dir = os.environ.get('ROTOSY_CALIBRATION_CONFIG_DIR')
    if env_dir:
        candidates.append(Path(env_dir).expanduser() / 'camera_extrinsic.yaml')

    candidates.append(Path.cwd() / 'rotosy_calibration' / 'config' / 'camera_extrinsic.yaml')

    try:
        candidates.append(Path(get_package_share_directory('rotosy_calibration')) / 'config' / 'camera_extrinsic.yaml')
    except PackageNotFoundError:
        pass

    candidates.append(Path(__file__).resolve().parents[2] / 'rotosy_calibration' / 'config' / 'camera_extrinsic.yaml')
    return candidates

def _find_calib_yaml() -> Optional[Path]:
    for p in _calib_candidates():
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


from .vision_detector import VisionDetector

class CameraManager:
    def __init__(self, width: int = 640, height: int = 480, fps: int = 30):
        self._width  = width
        self._height = height
        self._fps    = fps

        self._lock  = threading.Lock()
        self._jpeg: Optional[bytes] = None
        self._jpeg_raw: Optional[bytes] = None   # ArUco/YOLO 오버레이 없는 원본 (OCR용)

        self._aruco_lock = threading.Lock()
        self._aruco_markers: list = []
        self._detections: list = []
        self._top_faces: list = []

        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._yolo_thread: Optional[threading.Thread] = None
        # maxsize=1: YOLO 스레드가 바쁘면 최신 프레임으로 교체
        self._yolo_queue: queue.Queue = queue.Queue(maxsize=1)
        self._error: Optional[str] = None

        self._detector = VisionDetector()
        self._detector_loaded = self._detector.load()

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
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        if self._detector_loaded:
            self._yolo_thread = threading.Thread(target=self._yolo_loop, daemon=True)
            self._yolo_thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=4.0)
        if self._yolo_thread:
            self._yolo_thread.join(timeout=6.0)

    def get_jpeg(self) -> Optional[bytes]:
        with self._lock:
            return self._jpeg

    def get_jpeg_raw(self) -> Optional[bytes]:
        """ArUco/YOLO 오버레이가 그려지지 않은 원본 프레임 (OCR 입력용).

        OCR(Google Vision)이 디버그 오버레이 텍스트(ID/좌표/신뢰도)를 약품
        라벨 텍스트와 함께 읽어 줄 순서를 뒤섞는 문제가 있어 별도로 둔다.
        """
        with self._lock:
            return self._jpeg_raw

    def get_aruco_markers(self) -> list:
        with self._aruco_lock:
            return list(self._aruco_markers)

    def get_detections(self) -> list:
        with self._aruco_lock:
            return list(self._detections)

    def get_top_faces(self) -> list:
        with self._aruco_lock:
            return list(self._top_faces)

    @property
    def detections(self) -> list:
        return self.get_detections()

    @property
    def detector_loaded(self) -> bool:
        return self._detector_loaded

    @property
    def detector_error(self) -> Optional[str]:
        return self._detector.error

    @property
    def error(self) -> Optional[str]:
        return self._error

    @property
    def available(self) -> bool:
        return _RS_AVAILABLE

    # ── Internal ──────────────────────────────────────────────────────────────

    def _yolo_loop(self) -> None:
        """YOLO 추론 전담 스레드 — 카메라 루프와 완전히 분리되어 블로킹하지 않음."""
        while self._running:
            try:
                frame, depth_data, depth_scale, intrinsics = self._yolo_queue.get(timeout=1.0)
            except queue.Empty:
                continue
            stable_dets, _ = self._detector.process(
                frame, depth_data, depth_scale, intrinsics, self._T_base_camera
            )
            with self._aruco_lock:
                self._detections = stable_dets

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

    @staticmethod
    def _depth_stats_in_polygon(
        depth_data: np.ndarray,
        depth_scale: float,
        polygon: np.ndarray,
    ) -> tuple[Optional[float], Optional[float], int]:
        mask = np.zeros(depth_data.shape, dtype=np.uint8)
        cv2.fillConvexPoly(mask, polygon.astype(np.int32), 255)
        values = depth_data[mask > 0].astype(np.float32) * depth_scale
        valid = values[(values > 0.05) & (values < 5.0)]
        if valid.size < 20:
            return None, None, int(valid.size)
        return float(np.median(valid)), float(np.std(valid)), int(valid.size)

    def _detect_top_faces(
        self,
        frame: np.ndarray,
        depth_data: np.ndarray,
        depth_scale: float,
        intrinsics,
        detections: list,
    ) -> list:
        """Detect rectangular top-face candidates inside YOLO bounding boxes."""
        height, width = frame.shape[:2]
        faces = []
        for detection in detections:
            bbox = detection.get('bbox')
            if bbox is None or len(bbox) != 4:
                continue

            x1, y1, x2, y2 = [int(value) for value in bbox]
            x1 = max(0, min(width - 1, x1))
            y1 = max(0, min(height - 1, y1))
            x2 = max(0, min(width, x2))
            y2 = max(0, min(height, y2))
            if x2 <= x1 or y2 <= y1:
                continue

            roi = frame[y1:y2, x1:x2]
            if roi.size == 0:
                continue

            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (5, 5), 0)
            kernel = np.ones((5, 5), np.uint8)

            contour_sets = []
            for low, high in ((35, 110), (45, 130), (60, 170)):
                edges = cv2.Canny(gray, low, high)
                contour_sets.append(edges)
                closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=1)
                contour_sets.append(closed)
                connected = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=2)
                connected = cv2.dilate(connected, kernel, iterations=1)
                contour_sets.append(connected)

            bbox_area = max(1, (x2 - x1) * (y2 - y1))
            candidates = []
            margin_px = 8
            for edge_image in contour_sets:
                contours, _ = cv2.findContours(
                    edge_image,
                    cv2.RETR_LIST,
                    cv2.CHAIN_APPROX_SIMPLE,
                )
                for contour in contours:
                    area = cv2.contourArea(contour)
                    if area < 800 or area > 120000:
                        continue

                    area_ratio = area / float(bbox_area)
                    if area_ratio < 0.03 or area_ratio > 0.85:
                        continue

                    rect = cv2.minAreaRect(contour)
                    (cx, cy), (rw, rh), angle = rect
                    if rw < 12 or rh < 12:
                        continue

                    ratio = max(rw, rh) / max(1.0, min(rw, rh))
                    if ratio < 0.35 or ratio > 3.5:
                        continue

                    rectangularity = area / max(1.0, rw * rh)
                    if rectangularity < 0.45:
                        continue

                    center_x = int(round(cx)) + x1
                    center_y = int(round(cy)) + y1
                    if center_x < 0 or center_x >= width or center_y < 0 or center_y >= height:
                        continue

                    box = cv2.boxPoints(((cx + x1, cy + y1), (rw, rh), angle)).astype(int)
                    if (
                        np.any(box[:, 0] < x1 - margin_px)
                        or np.any(box[:, 0] > x2 + margin_px)
                        or np.any(box[:, 1] < y1 - margin_px)
                        or np.any(box[:, 1] > y2 + margin_px)
                    ):
                        continue

                    depth_m = self._sample_depth_patch(
                        depth_data, depth_scale, center_x, center_y, radius=7
                    )
                    if depth_m is None or depth_m < 0.15 or depth_m > 1.50:
                        continue

                    point = rs.rs2_deproject_pixel_to_point(
                        intrinsics, [float(center_x), float(center_y)], depth_m
                    )
                    camera_position = [float(point[0]), float(point[1]), float(point[2])]
                    base_position = None
                    if self._T_base_camera is not None:
                        p_base = self._T_base_camera @ np.array([point[0], point[1], point[2], 1.0])
                        base_position = [float(p_base[0]), float(p_base[1]), float(p_base[2])]

                    size_score = min(area_ratio, 0.55)
                    score = (rectangularity * 2.0) + size_score
                    candidates.append({
                        'center_px': [center_x, center_y],
                        'area_px': float(area),
                        'corners': box.tolist(),
                        'camera_position_m': camera_position,
                        'base_position_m': base_position,
                        'score': float(score),
                        'yolo_bbox': [x1, y1, x2, y2],
                        'yolo_class_name': detection.get('class_name'),
                        'yolo_confidence': detection.get('confidence'),
                    })

            if candidates:
                faces.append(max(candidates, key=lambda item: item['score']))

        kept = []
        for face in sorted(faces, key=lambda item: item['area_px'], reverse=True):
            cx, cy = face['center_px']
            if any(np.hypot(cx - other['center_px'][0], cy - other['center_px'][1]) < 35 for other in kept):
                continue
            kept.append(face)
        return sorted(kept, key=lambda item: item['center_px'][0])

    @staticmethod
    def _annotate_top_faces(frame: np.ndarray, faces: list) -> np.ndarray:
        annotated = frame.copy()
        for index, face in enumerate(faces, start=1):
            corners = np.array(face['corners'], dtype=np.int32).reshape((-1, 1, 2))
            cx, cy = face['center_px']
            cv2.polylines(annotated, [corners], True, (255, 0, 255), 3)
            cv2.circle(annotated, (cx, cy), 6, (255, 0, 255), -1)
            label = f"TOP {index}"
            base = face.get('base_position_m')
            if base is not None:
                label += f" ({base[0]*1000:.0f},{base[1]*1000:.0f},{base[2]*1000:.0f})mm"
            text_origin = (cx + 8, max(22, cy - 10))
            cv2.putText(
                annotated, label, text_origin,
                cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 0, 0), 4, cv2.LINE_AA,
            )
            cv2.putText(
                annotated, label, text_origin,
                cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 0, 255), 2, cv2.LINE_AA,
            )
        return annotated

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

                    depth_data = np.asanyarray(depth_filtered.get_data()).copy()
                    raw_frame  = np.asanyarray(color_frame.get_data()).copy()
                    frame      = raw_frame.copy()

                    ok_raw, jpeg_raw = cv2.imencode('.jpg', raw_frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
                    if ok_raw:
                        with self._lock:
                            self._jpeg_raw = jpeg_raw.tobytes()

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
                                marker_rotation_base = None
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
                                    if self._T_base_camera is not None:
                                        R_marker_camera, _ = cv2.Rodrigues(chosen_rvec)
                                        R_marker_base = self._T_base_camera[:3, :3] @ R_marker_camera
                                        marker_rotation_base = [
                                            [float(v) for v in row]
                                            for row in R_marker_base
                                        ]

                                markers_out.append({
                                    'id':      int(marker_id),
                                    'px':      cx,
                                    'py':      cy,
                                    'x_cam_m': round(p_cam[0], 6) if p_cam else None,
                                    'y_cam_m': round(p_cam[1], 6) if p_cam else None,
                                    'z_cam_m': round(p_cam[2], 6) if p_cam else None,
                                    'rvec':    rvec_flat,
                                    'rotation_matrix_base': marker_rotation_base,
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

                    # ── YOLO: 큐에 프레임을 넘기고 즉시 반환 (블로킹 없음) ────────
                    current_dets = []
                    if self._detector_loaded:
                        try:
                            # 큐가 가득 차면 이전 대기 프레임을 버리고 최신으로 교체
                            self._yolo_queue.get_nowait()
                        except queue.Empty:
                            pass
                        self._yolo_queue.put_nowait(
                            (raw_frame.copy(), depth_data, depth_scale, intrinsics)
                        )
                        with self._aruco_lock:
                            current_dets = list(self._detections)

                    top_faces = self._detect_top_faces(
                        raw_frame, depth_data, depth_scale, intrinsics, current_dets
                    )
                    with self._aruco_lock:
                        self._aruco_markers = markers_out
                        self._top_faces = top_faces

                    if self._detector_loaded:
                        frame = self._detector._annotate(frame, current_dets)
                    frame = self._annotate_top_faces(frame, top_faces)

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
