"""Top-face detection from YOLO boxes using color and depth edges.

This module keeps the camera ownership and YOLO inference flow in
``camera.py`` unchanged. It only refines a YOLO box into a candidate top-face
polygon and base-frame center.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

try:
    import pyrealsense2 as rs
except ImportError:  # pragma: no cover - camera.py already gates RealSense use
    rs = None


MEDICINE_CLASSES = {'medicine', 'medicine_box', 'water_pack'}


@dataclass(frozen=True)
class TopFaceConfig:
    color_low: int = 30
    color_high: int = 90
    depth_low: int = 15
    depth_high: int = 50
    band_mm: float = 14.0
    min_depth_m: float = 0.05
    max_depth_m: float = 2.0


# Drawer ID -> (medicine label, top-face width/height candidates in mm).
# These are used only as optional geometric priors. If no prior matches, the
# detector falls back to a convex hull from measured top-edge pixels.
DRAWER_SPECS: dict[int, tuple[str, list[tuple[float, float]]]] = {
    2: ('수액팩', []),
    3: ('심미안정', [(78.0, 107.0)]),
    4: ('일본 유산균', [(57.0, 102.0), (55.0, 102.0)]),
    5: ('벤포벨S', [(57.0, 110.0), (56.0, 110.0)]),
}


class DualCannyTopFaceDetector:
    def __init__(self, config: TopFaceConfig | None = None) -> None:
        self.config = config or TopFaceConfig()

    def detect(
        self,
        frame: np.ndarray,
        depth_data: np.ndarray,
        depth_scale: float,
        intrinsics,
        base_from_camera: Optional[np.ndarray],
        detections: list[dict],
        active_drawer: Optional[int] = None,
    ) -> list[dict]:
        if rs is None or base_from_camera is None:
            return []

        image_h, image_w = frame.shape[:2]
        rotation = base_from_camera[:3, :3]
        translation = base_from_camera[:3, 3]
        faces = []

        for detection in detections:
            if str(detection.get('class_name', '')).lower() not in MEDICINE_CLASSES:
                continue

            bbox = detection.get('bbox')
            if bbox is None or len(bbox) != 4:
                continue

            x1, y1, x2, y2 = [int(value) for value in bbox]
            x1 = max(0, min(image_w - 1, x1))
            y1 = max(0, min(image_h - 1, y1))
            x2 = max(0, min(image_w, x2))
            y2 = max(0, min(image_h, y2))
            if (x2 - x1) <= 10 or (y2 - y1) <= 10:
                continue

            result = self._detect_one(
                frame,
                depth_data,
                depth_scale,
                intrinsics,
                rotation,
                translation,
                detection,
                (x1, y1, x2, y2),
                active_drawer,
            )
            if result is not None:
                faces.append(result)

        return self._deduplicate(faces)

    def _detect_one(
        self,
        frame: np.ndarray,
        depth_data: np.ndarray,
        depth_scale: float,
        intrinsics,
        rotation: np.ndarray,
        translation: np.ndarray,
        detection: dict,
        bbox: tuple[int, int, int, int],
        active_drawer: Optional[int],
    ) -> Optional[dict]:
        x1, y1, x2, y2 = bbox
        roi_color = frame[y1:y2, x1:x2]
        roi_depth_m = depth_data[y1:y2, x1:x2].astype(np.float32) * depth_scale

        gray = cv2.cvtColor(roi_color, cv2.COLOR_BGR2GRAY)
        blurred_color = cv2.GaussianBlur(gray, (5, 5), 0)
        edges_color = cv2.Canny(
            blurred_color,
            self.config.color_low,
            self.config.color_high,
        )

        edges_depth = self._depth_edges(roi_depth_m)
        union_mask = (edges_color > 0) | (edges_depth > 0)
        ys_local, xs_local = np.where(union_mask)
        if len(xs_local) < 10:
            return None

        edge_points_base = []
        edge_pixels_global = []
        for local_y, local_x in zip(ys_local, xs_local):
            global_x = x1 + int(local_x)
            global_y = y1 + int(local_y)
            depth_m = float(roi_depth_m[local_y, local_x])
            if depth_m <= self.config.min_depth_m or depth_m > self.config.max_depth_m:
                continue

            point_camera = rs.rs2_deproject_pixel_to_point(
                intrinsics,
                [float(global_x), float(global_y)],
                depth_m,
            )
            point_base = rotation @ np.array(point_camera, dtype=np.float64) + translation
            edge_points_base.append(point_base)
            edge_pixels_global.append((global_x, global_y))

        if len(edge_points_base) < 10:
            return None

        edge_points_base = np.asarray(edge_points_base, dtype=np.float64)
        edge_pixels_global = np.asarray(edge_pixels_global, dtype=np.int32)

        highest_z_m = float(np.max(edge_points_base[:, 2]))
        top_indices = edge_points_base[:, 2] >= (
            highest_z_m - self.config.band_mm / 1000.0
        )
        top_pixels = edge_pixels_global[top_indices]
        top_points_3d = edge_points_base[top_indices]
        if len(top_pixels) < 6:
            return None

        points_2d = top_points_3d[:, :2].astype(np.float32)
        rect = cv2.minAreaRect(points_2d)
        (_rect_cx, _rect_cy), (width_m, height_m), angle = rect
        measured_w_mm = float(width_m * 1000.0)
        measured_h_mm = float(height_m * 1000.0)

        prior = self._best_size_prior(
            str(detection.get('class_name', '')).lower(),
            measured_w_mm,
            measured_h_mm,
            active_drawer,
        )
        size_error_mm = None
        if prior is not None:
            top_pixels, top_points_3d, center_xy = self._crop_by_size_prior(
                points_2d,
                top_pixels,
                top_points_3d,
                angle,
                prior,
            )
            if len(top_pixels) < 3:
                return None
            approx_polygon = self._project_prior_polygon(
                center_xy,
                float(np.mean(top_points_3d[:, 2])),
                angle,
                prior,
                rotation,
                translation,
                intrinsics,
            )
            size_error_mm = (
                abs(measured_w_mm - prior[0]) + abs(measured_h_mm - prior[1])
            )
        else:
            hull = cv2.convexHull(top_pixels.astype(np.int32))
            perimeter = cv2.arcLength(hull, True)
            approx_polygon = cv2.approxPolyDP(
                hull,
                0.02 * perimeter,
                True,
            ).reshape(-1, 2)

        if not 3 <= len(approx_polygon) <= 8:
            return None
        if not self._inside_yolo_neighborhood(approx_polygon, bbox):
            return None

        center_base_m = np.mean(top_points_3d, axis=0)
        moments = cv2.moments(approx_polygon.astype(np.float32))
        if abs(moments['m00']) > 1e-6:
            center_px = [
                int(moments['m10'] / moments['m00']),
                int(moments['m01'] / moments['m00']),
            ]
        else:
            center_px = [
                int(np.mean(approx_polygon[:, 0])),
                int(np.mean(approx_polygon[:, 1])),
            ]

        return {
            'center_px': center_px,
            'corners': approx_polygon.astype(int).tolist(),
            'base_position_m': [float(v) for v in center_base_m[:3]],
            'score': float(len(top_pixels)),
            'edge_point_count': int(len(top_pixels)),
            'measured_size_mm': [measured_w_mm, measured_h_mm],
            'size_prior_mm': list(prior) if prior is not None else None,
            'active_drawer': int(active_drawer) if active_drawer is not None else None,
            'size_error_mm': float(size_error_mm) if size_error_mm is not None else None,
            'yolo_bbox': [int(v) for v in bbox],
            'yolo_class_name': detection.get('class_name'),
            'yolo_confidence': detection.get('confidence'),
        }

    def _depth_edges(self, roi_depth_m: np.ndarray) -> np.ndarray:
        valid_mask = (
            (roi_depth_m > self.config.min_depth_m)
            & (roi_depth_m < self.config.max_depth_m)
        )
        edges_depth = np.zeros(roi_depth_m.shape, dtype=np.uint8)
        if np.count_nonzero(valid_mask) < 20:
            return edges_depth

        min_d = float(roi_depth_m[valid_mask].min())
        max_d = float(roi_depth_m[valid_mask].max())
        depth_8bit = np.zeros(roi_depth_m.shape, dtype=np.uint8)
        if max_d > min_d:
            normalized = (roi_depth_m - min_d) / (max_d - min_d) * 255.0
            depth_8bit[valid_mask] = normalized[valid_mask].astype(np.uint8)

        blurred_depth = cv2.GaussianBlur(depth_8bit, (5, 5), 0)
        return cv2.Canny(
            blurred_depth,
            self.config.depth_low,
            self.config.depth_high,
        )

    @staticmethod
    def _size_candidates(active_drawer: Optional[int]) -> list[tuple[float, float]]:
        if active_drawer is not None:
            return list(DRAWER_SPECS.get(active_drawer, ('', []))[1])
        out = []
        for _label, candidates in DRAWER_SPECS.values():
            out.extend(candidates)
        return out

    def _best_size_prior(
        self,
        class_name: str,
        measured_w_mm: float,
        measured_h_mm: float,
        active_drawer: Optional[int],
    ) -> Optional[tuple[float, float]]:
        if class_name == 'water_pack':
            return None

        best = None
        best_error = float('inf')
        for w_target, h_target in self._size_candidates(active_drawer):
            for candidate in ((w_target, h_target), (h_target, w_target)):
                error = (
                    abs(measured_w_mm - candidate[0])
                    + abs(measured_h_mm - candidate[1])
                )
                if error < best_error:
                    best_error = error
                    best = candidate
        return best

    @staticmethod
    def _crop_by_size_prior(
        points_2d: np.ndarray,
        top_pixels: np.ndarray,
        top_points_3d: np.ndarray,
        angle: float,
        prior_mm: tuple[float, float],
    ) -> tuple[np.ndarray, np.ndarray, tuple[float, float]]:
        cx_base = float(np.median(points_2d[:, 0]))
        cy_base = float(np.median(points_2d[:, 1]))
        rad = np.radians(angle)
        rotation_2d = np.array([
            [np.cos(rad), -np.sin(rad)],
            [np.sin(rad), np.cos(rad)],
        ])

        best_inliers = -1
        opt_cx, opt_cy = cx_base, cy_base
        for dx in np.linspace(-0.04, 0.04, 21):
            for dy in np.linspace(-0.04, 0.04, 21):
                tx = cx_base + dx
                ty = cy_base + dy
                local_pts = (points_2d - np.array([tx, ty])) @ rotation_2d
                inliers = np.count_nonzero(
                    (np.abs(local_pts[:, 0]) <= prior_mm[0] / 2000.0)
                    & (np.abs(local_pts[:, 1]) <= prior_mm[1] / 2000.0)
                )
                if inliers > best_inliers:
                    best_inliers = inliers
                    opt_cx, opt_cy = tx, ty

        local_pts = (points_2d - np.array([opt_cx, opt_cy])) @ rotation_2d
        inside_mask = (
            (np.abs(local_pts[:, 0]) <= prior_mm[0] / 2000.0)
            & (np.abs(local_pts[:, 1]) <= prior_mm[1] / 2000.0)
        )
        return top_pixels[inside_mask], top_points_3d[inside_mask], (opt_cx, opt_cy)

    @staticmethod
    def _project_prior_polygon(
        center_xy: tuple[float, float],
        center_z: float,
        angle: float,
        prior_mm: tuple[float, float],
        rotation: np.ndarray,
        translation: np.ndarray,
        intrinsics,
    ) -> np.ndarray:
        half_w = prior_mm[0] / 2000.0
        half_h = prior_mm[1] / 2000.0
        local_corners = np.array([
            [-half_w, -half_h],
            [half_w, -half_h],
            [half_w, half_h],
            [-half_w, half_h],
        ])
        rad = np.radians(angle)
        rotation_2d = np.array([
            [np.cos(rad), -np.sin(rad)],
            [np.sin(rad), np.cos(rad)],
        ])

        inv_rotation = rotation.T
        projected = []
        for point in local_corners:
            rotated = rotation_2d @ point
            point_base = np.array([
                center_xy[0] + rotated[0],
                center_xy[1] + rotated[1],
                center_z,
            ])
            point_camera = inv_rotation @ (point_base - translation)
            pixel = rs.rs2_project_point_to_pixel(
                intrinsics,
                [float(point_camera[0]), float(point_camera[1]), float(point_camera[2])],
            )
            projected.append([int(pixel[0]), int(pixel[1])])
        return np.array(projected, dtype=np.int32)

    @staticmethod
    def _inside_yolo_neighborhood(
        polygon: np.ndarray,
        bbox: tuple[int, int, int, int],
    ) -> bool:
        x1, y1, x2, y2 = bbox
        yolo_cx = (x1 + x2) / 2.0
        yolo_cy = (y1 + y2) / 2.0
        yolo_w = x2 - x1
        yolo_h = y2 - y1
        proj_cx = float(np.mean(polygon[:, 0]))
        proj_cy = float(np.mean(polygon[:, 1]))
        return (
            abs(proj_cx - yolo_cx) <= yolo_w * 0.55
            and abs(proj_cy - yolo_cy) <= yolo_h * 0.55
        )

    @staticmethod
    def _deduplicate(faces: list[dict]) -> list[dict]:
        kept = []
        for face in sorted(faces, key=lambda item: item['score'], reverse=True):
            cx, cy = face['center_px']
            if any(
                np.hypot(cx - other['center_px'][0], cy - other['center_px'][1]) < 35
                for other in kept
            ):
                continue
            kept.append(face)
        return sorted(kept, key=lambda item: item['center_px'][0])
