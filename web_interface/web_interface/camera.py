"""
RealSense camera manager.

- Color + Depth 스트림 캡처
- RealSense post-processing (spatial / temporal / hole-filling) 적용
- ArUco 마커 검출 → 픽셀 좌표
- YOLO 객체 감지(수액팩) → 3D 픽 좌표
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

try:
    from ultralytics import YOLO as _YOLO
    _YOLO_AVAILABLE = True
except ImportError:
    _YOLO_AVAILABLE = False

_MODEL_PATH = Path('/home/cheol/Downloads/large_best.pt')

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

def _load_T_base_camera(yaml_path: Path) -> Optional[np.ndarray]:
    """camera_extrinsic.yaml → 4×4 변환 행렬 (T_base_camera)."""
    try:
        with open(yaml_path) as f:
            data = yaml.safe_load(f)
        params = data['camera_extrinsic']['ros__parameters']
        t = np.array(params['translation_xyz'], dtype=float)      # meters
        q = params['rotation_quat_xyzw']                           # [x,y,z,w]
        x, y, z, w = q
        R = np.array([
            [1-2*(y*y+z*z),   2*(x*y-w*z),   2*(x*z+w*y)],
            [  2*(x*y+w*z), 1-2*(x*x+z*z),   2*(y*z-w*x)],
            [  2*(x*z-w*y),   2*(y*z+w*x), 1-2*(x*x+y*y)],
        ], dtype=float)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3]  = t
        return T
    except Exception as exc:
        print(f'[camera] calib load failed: {exc}')
        return None


class CameraManager:
    def __init__(self, width: int = 640, height: int = 480, fps: int = 30):
        self._width  = width
        self._height = height
        self._fps    = fps

        self._lock  = threading.Lock()
        self._jpeg: Optional[bytes] = None

        self._aruco_lock = threading.Lock()
        self._aruco_markers: list = []   # [{id, px, py, x_mm, y_mm, z_mm}, ...]

        self._yolo_lock = threading.Lock()
        self._yolo_detections: list = []  # [{class, confidence, bbox, px, py, depth_m, x_mm, y_mm, z_mm}, ...]

        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._error: Optional[str] = None

        calib_path = _find_calib_yaml()
        self._T_base_camera: Optional[np.ndarray] = None
        if calib_path:
            self._T_base_camera = _load_T_base_camera(calib_path)
            print(f'[camera] calibration loaded: {calib_path}')
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
        """각 마커: {id, px, py, x_mm, y_mm, z_mm}"""
        with self._aruco_lock:
            return list(self._aruco_markers)

    def get_yolo_detections(self) -> list:
        """최신 YOLO 감지 결과: {class, confidence, bbox, px, py, depth_m, x_mm, y_mm, z_mm}"""
        with self._yolo_lock:
            return list(self._yolo_detections)

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
        """마커 중심 패치의 median depth (m)."""
        h, w = depth_data.shape
        x1 = max(0, cx - radius)
        x2 = min(w, cx + radius + 1)
        y1 = max(0, cy - radius)
        y2 = min(h, cy + radius + 1)
        patch = depth_data[y1:y2, x1:x2].astype(np.float32) * depth_scale
        valid = patch[(patch > 0.05) & (patch < 5.0)]
        return float(np.median(valid)) if len(valid) > 0 else None

    @staticmethod
    def _sample_depth_bbox(
        depth_data: np.ndarray,
        depth_scale: float,
        x1: int, y1: int, x2: int, y2: int,
    ) -> Optional[float]:
        """바운딩 박스 내부 50% 영역의 median depth (m).

        외곽 25%를 제외해 경계 노이즈와 배경을 걸러낸다.
        """
        h_img, w_img = depth_data.shape
        bw = max(x2 - x1, 1)
        bh = max(y2 - y1, 1)
        mx1 = max(0, x1 + bw // 4)
        mx2 = min(w_img, x2 - bw // 4)
        my1 = max(0, y1 + bh // 4)
        my2 = min(h_img, y2 - bh // 4)

        if mx2 <= mx1 or my2 <= my1:
            # bbox가 너무 작으면 중심 한 픽셀
            cx = max(0, min(w_img - 1, (x1 + x2) // 2))
            cy = max(0, min(h_img - 1, (y1 + y2) // 2))
            d = float(depth_data[cy, cx]) * depth_scale
            return d if 0.05 < d < 5.0 else None

        region = depth_data[my1:my2, mx1:mx2].astype(np.float32) * depth_scale
        valid = region[(region > 0.05) & (region < 5.0)]
        return float(np.median(valid)) if len(valid) > 0 else None

    def _deproject_to_robot_mm(
        self,
        cx: int, cy: int,
        depth_m: float,
        intrinsics,
    ) -> Optional[tuple]:
        """픽셀 + depth → 로봇 베이스 좌표 (mm).

        rs2_deproject_pixel_to_point을 사용해 렌즈 왜곡을 보정한다.
        """
        if self._T_base_camera is None:
            return None

        # RealSense SDK: 왜곡 모델 반영한 역투영
        pt = rs.rs2_deproject_pixel_to_point(intrinsics, [float(cx), float(cy)], depth_m)
        p_cam  = np.array([pt[0], pt[1], pt[2], 1.0])
        p_base = self._T_base_camera @ p_cam
        return (
            float(p_base[0] * 1000),
            float(p_base[1] * 1000),
            float(p_base[2] * 1000),
        )

    def _capture_loop(self) -> None:
        aruco_detector = self._init_aruco()

        # YOLO 모델 로드
        yolo_model = None
        if _YOLO_AVAILABLE and _MODEL_PATH.exists():
            try:
                yolo_model = _YOLO(str(_MODEL_PATH))
                print(f'[camera] YOLO model loaded: {_MODEL_PATH}  classes={yolo_model.names}')
            except Exception as exc:
                print(f'[camera] YOLO load failed: {exc}')
        else:
            if not _YOLO_AVAILABLE:
                print('[camera] ultralytics not installed — YOLO detection disabled')
            else:
                print(f'[camera] model not found: {_MODEL_PATH} — YOLO detection disabled')

        pipeline = rs.pipeline()
        config   = rs.config()
        config.enable_stream(rs.stream.depth, self._width, self._height, rs.format.z16,  self._fps)
        config.enable_stream(rs.stream.color, self._width, self._height, rs.format.bgr8, self._fps)

        try:
            profile = pipeline.start(config)
        except Exception as exc:
            # I/O 오류(errno=5)는 이전 프로세스가 카메라를 해제하지 않은 경우 발생.
            # 하드웨어 리셋 후 한 번 재시도한다.
            print(f'[camera] 첫 시작 실패: {exc} — 하드웨어 리셋 후 재시도')
            try:
                import time as _time
                ctx = rs.context()
                for dev in ctx.query_devices():
                    dev.hardware_reset()
                _time.sleep(3.0)
                profile = pipeline.start(config)
                print('[camera] 리셋 후 재시작 성공')
            except Exception as exc2:
                self._error   = f'RealSense 시작 실패: {exc2}'
                self._running = False
                return

        # Intrinsics & depth scale
        color_profile = profile.get_stream(rs.stream.color)
        intrinsics    = color_profile.as_video_stream_profile().get_intrinsics()
        depth_scale   = profile.get_device().first_depth_sensor().get_depth_scale()

        # Depth post-processing 필터 (한 번 생성해서 매 프레임 재사용)
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

        try:
            while self._running:
                try:
                    frames = pipeline.wait_for_frames(timeout_ms=2000)
                except RuntimeError:
                    continue

                aligned     = align.process(frames)
                depth_raw   = aligned.get_depth_frame()
                color_frame = aligned.get_color_frame()
                if not color_frame or not depth_raw:
                    continue

                # Depth 후처리: spatial → temporal → hole-filling
                depth_filtered = spatial.process(depth_raw)
                depth_filtered = temporal.process(depth_filtered)
                depth_filtered = hole_filling.process(depth_filtered)

                depth_data = np.asanyarray(depth_filtered.get_data())   # uint16 (H×W)
                frame      = np.asanyarray(color_frame.get_data())

                markers_out: list    = []
                detections_out: list = []

                # ── ArUco 검출 ───────────────────────────────────────────────
                if aruco_detector is not None:
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    corners, ids, _ = aruco_detector.detectMarkers(gray)

                    if ids is not None:
                        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                        for i, marker_id in enumerate(ids.flatten()):
                            cx = int(corners[i][0][:, 0].mean())
                            cy = int(corners[i][0][:, 1].mean())

                            depth_m  = self._sample_depth_patch(depth_data, depth_scale, cx, cy, radius=12)
                            p_cam    = None
                            robot    = None
                            if depth_m is not None:
                                pt    = rs.rs2_deproject_pixel_to_point(intrinsics, [float(cx), float(cy)], depth_m)
                                p_cam = (float(pt[0]), float(pt[1]), float(pt[2]))
                                if self._T_base_camera is not None:
                                    p_h    = np.array([pt[0], pt[1], pt[2], 1.0])
                                    p_base = self._T_base_camera @ p_h
                                    robot  = (float(p_base[0]*1000), float(p_base[1]*1000), float(p_base[2]*1000))

                            markers_out.append({
                                'id':      int(marker_id),
                                'px':      cx,
                                'py':      cy,
                                # 카메라 프레임 좌표 (m) — manual calibration 용
                                'x_cam_m': round(p_cam[0], 6) if p_cam else None,
                                'y_cam_m': round(p_cam[1], 6) if p_cam else None,
                                'z_cam_m': round(p_cam[2], 6) if p_cam else None,
                                # 로봇 베이스 프레임 좌표 (mm)
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

                # ── YOLO 감지 ────────────────────────────────────────────────
                if yolo_model is not None:
                    results = yolo_model(frame, verbose=False)[0]
                    for box in results.boxes:
                        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                        conf     = float(box.conf[0])
                        cls_name = yolo_model.names[int(box.cls[0])]
                        cx_box   = (x1 + x2) // 2
                        cy_box   = (y1 + y2) // 2

                        depth_m = self._sample_depth_bbox(depth_data, depth_scale, x1, y1, x2, y2)
                        robot   = None
                        if depth_m is not None:
                            robot = self._deproject_to_robot_mm(cx_box, cy_box, depth_m, intrinsics)

                        detections_out.append({
                            'class':      cls_name,
                            'confidence': round(conf, 3),
                            'bbox':       [x1, y1, x2, y2],
                            'px':         cx_box,
                            'py':         cy_box,
                            'depth_m':    round(depth_m, 4) if depth_m is not None else None,
                            'x_mm':       round(robot[0], 1) if robot else None,
                            'y_mm':       round(robot[1], 1) if robot else None,
                            'z_mm':       round(robot[2], 1) if robot else None,
                        })

                        # 화면 오버레이
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        label = f"{cls_name} {conf:.2f}"
                        if robot:
                            label += f" ({robot[0]:.0f},{robot[1]:.0f},{robot[2]:.0f})mm"
                        cv2.putText(
                            frame, label,
                            (x1, max(20, y1 - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.50,
                            (0, 255, 0), 2, cv2.LINE_AA,
                        )

                with self._aruco_lock:
                    self._aruco_markers = markers_out
                with self._yolo_lock:
                    self._yolo_detections = detections_out

                ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ok:
                    with self._lock:
                        self._jpeg = jpeg.tobytes()

        finally:
            pipeline.stop()


# Module-level singleton
camera = CameraManager()
