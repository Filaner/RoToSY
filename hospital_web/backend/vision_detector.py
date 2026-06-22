"""YOLO medicine detection using frames owned by ``CameraManager``."""

from collections import deque
from pathlib import Path
import os
import time
from typing import Optional

import cv2
import numpy as np


MEDICINE_CLASS_NAME = 'medicine'
WATER_PACK_CLASS_NAME = 'water_pack'


def _model_candidates() -> list[Path]:
    _models_dir = Path(__file__).resolve().parent / 'models'
    candidates = []
    configured = os.environ.get('ROTOSY_VISION_MODEL')
    if configured:
        candidates.append(Path(configured).expanduser())
    candidates.append(_models_dir / 'best.pt')
    candidates.append(_models_dir / 'medium_best_version2.pt')
    return candidates


def find_model_path() -> Optional[Path]:
    return next((path for path in _model_candidates() if path.exists()), None)


def _bbox_iou(a: tuple[int, int, int, int], b: tuple[int, int, int, int]) -> float:
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
    intersection = iw * ih
    if intersection == 0:
        return 0.0
    area_a = max(0, ax2 - ax1) * max(0, ay2 - ay1)
    area_b = max(0, bx2 - bx1) * max(0, by2 - by1)
    union = area_a + area_b - intersection
    return float(intersection / union) if union > 0 else 0.0


class VisionDetector:
    """Runs the supplied YOLO model and stabilizes labels across recent frames."""

    def __init__(
        self,
        medicine_conf: float = 0.5,
        water_pack_conf: float = 0.5,
        history_frames: int = 6,
        track_iou: float = 0.5,
        min_votes: int = 2,
        switch_margin: float = 1.2,
        device: Optional[str] = None,
        inference_hz: float = 30.0,
    ) -> None:
        self.medicine_conf = medicine_conf
        self.water_pack_conf = water_pack_conf
        self.history_frames = history_frames
        self.track_iou = track_iou
        self.min_votes = min_votes
        self.switch_margin = switch_margin
        self.device = device
        self.inference_interval_sec = 1.0 / max(0.1, inference_hz)
        self.model = None
        self.names: dict[int, str] = {}
        self.tracks: list[dict] = []
        self.error: Optional[str] = None
        self._last_inference_at = 0.0
        self._last_detections: list[dict] = []

    def load(self) -> bool:
        path = find_model_path()
        if path is None:
            self.error = 'YOLO 모델 파일 없음 — best.pt 를 hospital_web/backend/models/ 에 넣으세요'
            return False
        try:
            from ultralytics import YOLO
            self.model = YOLO(str(path))
            self.names = dict(self.model.names)
            print(f'[vision] model loaded: {path} names={self.names}')
            self.error = None
            return True
        except Exception as exc:
            self.error = f'YOLO load failed: {exc}'
            print(f'[vision] {self.error}')
            return False

    @staticmethod
    def _sample_depth(
        depth_data: np.ndarray,
        depth_scale: float,
        cx: int,
        cy: int,
        radius: int = 5,
    ) -> Optional[float]:
        height, width = depth_data.shape
        patch = depth_data[
            max(0, cy - radius):min(height, cy + radius + 1),
            max(0, cx - radius):min(width, cx + radius + 1),
        ].astype(np.float32) * depth_scale
        valid = patch[(patch > 0.05) & (patch < 5.0)]
        return float(np.median(valid)) if valid.size else None

    def _choose_label(self, history: deque, current: Optional[str]) -> Optional[str]:
        scores: dict[str, float] = {}
        counts: dict[str, int] = {}
        for item in history:
            label = item['class_name']
            scores[label] = scores.get(label, 0.0) + item['confidence']
            counts[label] = counts.get(label, 0) + 1
        candidates = {
            label: score for label, score in scores.items()
            if counts[label] >= self.min_votes
        }
        if not candidates:
            return current
        best = max(candidates, key=candidates.get)
        if current is None:
            return best
        if best != current and scores[best] > scores.get(current, 0.0) * self.switch_margin:
            return best
        return current

    def _update_tracks(self, detections: list[dict]) -> list[dict]:
        for track in self.tracks:
            track['age'] += 1

        matched: set[int] = set()
        for detection in detections:
            best_idx = None
            best_iou = 0.0
            for idx, track in enumerate(self.tracks):
                if idx in matched:
                    continue
                iou = _bbox_iou(detection['bbox'], track['bbox'])
                if iou > best_iou:
                    best_iou, best_idx = iou, idx

            if best_idx is None or best_iou < self.track_iou:
                track = {
                    'bbox': detection['bbox'],
                    'history': deque(maxlen=self.history_frames),
                    'stable_label': None,
                    'age': 0,
                }
                self.tracks.append(track)
                best_idx = len(self.tracks) - 1
            else:
                track = self.tracks[best_idx]
                track['bbox'] = detection['bbox']
                track['age'] = 0

            matched.add(best_idx)
            track['history'].append(detection)
            track['stable_label'] = self._choose_label(
                track['history'], track['stable_label']
            )

        self.tracks[:] = [
            track for track in self.tracks if track['age'] <= self.history_frames
        ]

        stable = []
        for track in self.tracks:
            if track['age'] != 0 or track['stable_label'] is None:
                continue
            matching = [
                item for item in track['history']
                if item['class_name'] == track['stable_label']
            ]
            if not matching: continue
            latest = matching[-1]
            output = dict(latest)
            output['confidence'] = max(item['confidence'] for item in matching)
            stable.append(output)
        return stable

    def _detections_from_result(
        self,
        result,
        depth_data: np.ndarray,
        depth_scale: float,
        intrinsics,
        base_to_camera: Optional[np.ndarray],
        x_offset: int = 0,
        y_offset: int = 0,
    ) -> list[dict]:
        detections = []
        if result.boxes is None:
            return detections

        for box in result.boxes:
            cls_id = int(box.cls[0].item())
            class_name = self.names.get(cls_id, str(cls_id))
            
            # 클래스별 개별 임계값 적용 (통일 작업)
            confidence = float(box.conf[0].item())
            # Use medicine_conf if class is 'medicine' or 'medicine_box'
            is_medicine = class_name.lower() in [MEDICINE_CLASS_NAME, 'medicine_box']
            target_conf = self.medicine_conf if is_medicine else self.water_pack_conf
            if confidence < target_conf:
                continue

            x1, y1, x2, y2 = [int(value) for value in box.xyxy[0].tolist()]
            x1 += x_offset
            x2 += x_offset
            y1 += y_offset
            y2 += y_offset
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            depth_m = self._sample_depth(depth_data, depth_scale, cx, cy)
            detection = {
                'class_name': class_name,
                'confidence': confidence,
                'bbox': (x1, y1, x2, y2),
                'depth_m': depth_m,
                'camera_position_m': None,
                'base_position_m': None,
                'stamp_ns': time.time_ns(),
            }
            if depth_m is not None:
                point = __import__('pyrealsense2').rs2_deproject_pixel_to_point(
                    intrinsics, [float(cx), float(cy)], depth_m
                )
                camera_point = np.array(point, dtype=float)
                detection['camera_position_m'] = camera_point.tolist()
                if base_to_camera is not None:
                    base_point = base_to_camera @ np.array([
                        camera_point[0], camera_point[1], camera_point[2], 1.0
                    ])
                    detection['base_position_m'] = base_point[:3].tolist()
            detections.append(detection)
        return detections

    @staticmethod
    def _suppress_overlaps(detections: list[dict]) -> list[dict]:
        kept = []
        for detection in sorted(
            detections, key=lambda item: item['confidence'], reverse=True
        ):
            if any(_bbox_iou(detection['bbox'], item['bbox']) > 0.7 for item in kept):
                continue
            kept.append(detection)
        return kept

    def process(
        self,
        color: np.ndarray,
        depth_data: np.ndarray,
        depth_scale: float,
        intrinsics,
        base_to_camera: Optional[np.ndarray],
    ) -> tuple[list[dict], np.ndarray]:
        if self.model is None:
            return [], color

        now = time.monotonic()
        if now - self._last_inference_at < self.inference_interval_sec:
            return self._last_detections, self._annotate(color, self._last_detections)

        self._last_inference_at = now
        
        try:
            result = self.model.predict(
                color,
                conf=min(self.medicine_conf, self.water_pack_conf),
                imgsz=640,
                device=self.device,
                verbose=False,
            )[0]
        except Exception as exc:
            self.error = f'YOLO inference failed: {exc}'
            return self._last_detections, self._annotate(color, self._last_detections)

        raw = self._detections_from_result(
            result, depth_data, depth_scale, intrinsics, base_to_camera
        )

        # 성공률이 높았던 Left-Lower ROI(화면 왼쪽 하단) 재시도 로직 유지
        if not raw:
            height, width = color.shape[:2]
            x1_roi, y1_roi = 0, height // 4
            x2_roi, y2_roi = width // 2, height
            roi = color[y1_roi:y2_roi, x1_roi:x2_roi]
            
            try:
                roi_result = self.model.predict(
                    roi,
                    conf=min(self.medicine_conf, self.water_pack_conf),
                    imgsz=640,
                    device=self.device,
                    verbose=False,
                )[0]
                raw = self._detections_from_result(
                    roi_result,
                    depth_data,
                    depth_scale,
                    intrinsics,
                    base_to_camera,
                    x_offset=x1_roi,
                    y_offset=y1_roi,
                )
            except:
                pass

        raw = self._suppress_overlaps(raw)

        stable = self._update_tracks(raw)
        self._last_detections = stable
        self.error = None
        return stable, self._annotate(color, stable)

    @staticmethod
    def _annotate(color: np.ndarray, detections: list[dict]) -> np.ndarray:
        annotated = color.copy()
        for item in detections:
            x1, y1, x2, y2 = item['bbox']
            # Yellow for medicine, Green for water_pack
            is_medicine = item['class_name'].lower() in [MEDICINE_CLASS_NAME, 'medicine_box']
            color_bgr = (0, 255, 255) if is_medicine else (0, 255, 0)
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color_bgr, 2)
            label = f"{item['class_name']} {item['confidence']:.2f}"
            base = item.get('base_position_m')
            if base is not None:
                label += f" ({base[0]*1000:.0f},{base[1]*1000:.0f},{base[2]*1000:.0f})mm"
            cv2.putText(
                annotated, label, (x1, max(20, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, color_bgr, 2, cv2.LINE_AA,
            )
        return annotated
