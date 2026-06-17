"""Detect medicine box face centers and publish their 3D camera coordinates."""

from __future__ import annotations

import os
import time
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass
from pathlib import Path
from typing import Any, List, Optional, Tuple

import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
from geometry_msgs.msg import PointStamped, PoseArray, Pose
from rclpy.node import Node
from std_msgs.msg import Int32

try:
    from ultralytics import YOLO
except Exception:  # noqa: B902
    YOLO = None

PROJECT_ID = "ocr1-496908"
GCLOUD_ADC_PATH = (
    "/home/user/.config/gcloud/legacy_credentials/"
    "jaegang1000@gmail.com/adc.json"
)
ARUCO_DICTIONARY_NAME = "DICT_4X4_50"
YOLO_DEFAULT_MODEL_PATH = os.environ.get(
    "ROTOSY_VISION_MODEL",
    str(Path(__file__).resolve().parent / "models" / "best.pt"),
)


def create_aruco_detector(dictionary_name: str) -> Any:
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("cv2.aruco is not available. Install opencv-contrib-python.")

    if not hasattr(cv2.aruco, dictionary_name):
        available = sorted(name for name in dir(cv2.aruco) if name.startswith("DICT_"))
        raise ValueError(
            f"Unknown ArUco dictionary: {dictionary_name}. "
            f"Available dictionaries include: {', '.join(available)}"
        )

    dictionary_id = getattr(cv2.aruco, dictionary_name)
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    parameters = cv2.aruco.DetectorParameters()
    return cv2.aruco.ArucoDetector(dictionary, parameters)


def setup_ocr_credentials():
    os.environ.setdefault("GOOGLE_CLOUD_PROJECT", PROJECT_ID)
    key_path = os.environ.get("GOOGLE_APPLICATION_CREDENTIALS")

    if not key_path:
        if os.path.isfile(GCLOUD_ADC_PATH):
            os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = GCLOUD_ADC_PATH
            print(f"OCR 인증: gcloud 인증 파일 사용, project={PROJECT_ID}")
            return True

        print("OCR 인증: Application Default Credentials 사용, "
              f"project={PROJECT_ID}")
        return True

    if not os.path.isfile(key_path):
        if os.path.isfile(GCLOUD_ADC_PATH):
            os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = GCLOUD_ADC_PATH
            print("OCR 인증: 잘못된 키 경로 대신 gcloud 인증 파일 사용, "
                  f"project={PROJECT_ID}")
            return True

        print("OCR 인증: 키 파일을 찾을 수 없어 기본 인증 사용, "
              f"project={PROJECT_ID}")
        return True

    print(f"OCR 인증: service account key 사용, project={PROJECT_ID}")
    return True


def can_use_ocr():
    return setup_ocr_credentials()


def run_ocr(image_path):
    import google.auth
    from google.cloud import vision

    setup_ocr_credentials()
    credentials, _ = google.auth.default(
        scopes=["https://www.googleapis.com/auth/cloud-platform"],
        quota_project_id=PROJECT_ID,
    )
    client = vision.ImageAnnotatorClient(credentials=credentials)

    with open(image_path, "rb") as image_file:
        content = image_file.read()

    image = vision.Image(content=content)
    response = client.document_text_detection(
        image=image,
        image_context={"language_hints": ["ko", "en", "ja"]},
    )

    if response.error.message:
        raise RuntimeError(response.error.message)

    return response.full_text_annotation.text.strip()


@dataclass
class BoxFace:
    contour: np.ndarray
    rect: Tuple[Tuple[float, float], Tuple[float, float], float]
    center_px: Tuple[int, int]
    point_m: Tuple[float, float, float]
    area_px: float
    normal_m: Optional[Tuple[float, float, float]]
    top_score_m: float


@dataclass
class MedicineBox:
    faces: List[BoxFace]
    grasp_face: BoxFace
    center_px: Tuple[int, int]
    point_m: Tuple[float, float, float]


@dataclass
class ArucoMarker:
    marker_id: int
    corners: np.ndarray
    center_px: Tuple[int, int]
    point_m: Optional[Tuple[float, float, float]]


@dataclass
class YoloDetection:
    class_id: int
    class_name: str
    confidence: float
    xyxy: Tuple[int, int, int, int]
    center_px: Tuple[int, int]
    point_m: Optional[Tuple[float, float, float]]


class MedicineBoxCenterDetector(Node):
    """
    Find rectangular medicine box faces and publish center 3D points.

    This node intentionally uses OpenCV rules, not AI. It is suitable when the
    camera, drawer ROI, object size, and lighting are reasonably controlled.
    """

    def __init__(self) -> None:
        super().__init__("medicine_box_center_detector")

        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30)
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("roi_x_min", 0.05)
        self.declare_parameter("roi_x_max", 0.95)
        self.declare_parameter("roi_y_min", 0.05)
        self.declare_parameter("roi_y_max", 0.95)
        self.declare_parameter("min_area_px", 800)
        self.declare_parameter("max_area_px", 120000)
        self.declare_parameter("min_aspect_ratio", 0.35)
        self.declare_parameter("max_aspect_ratio", 3.5)
        self.declare_parameter("min_depth_m", 0.15)
        self.declare_parameter("max_depth_m", 1.50)
        self.declare_parameter("depth_window_px", 7)
        self.declare_parameter("box_cluster_distance_px", 130.0)
        self.declare_parameter("box_cluster_distance_m", 0.12)
        self.declare_parameter("drawer_floor_up_x", 0.0)
        self.declare_parameter("drawer_floor_up_y", 0.0)
        self.declare_parameter("drawer_floor_up_z", -1.0)
        self.declare_parameter("top_face_parallel_cos_min", 0.90)
        self.declare_parameter("face_plane_sample_step_px", 4)
        self.declare_parameter("face_plane_min_points", 25)
        self.declare_parameter("publish_nearest_only", False)
        self.declare_parameter("realtime_ocr", False)
        self.declare_parameter("ocr_interval_sec", 2.0)
        self.declare_parameter("ocr_image_path", "medicine_box_ocr.jpg")
        self.declare_parameter("ocr_crop_face", True)
        self.declare_parameter("ocr_min_text_length", 1)
        self.declare_parameter("use_aruco", True)
        self.declare_parameter("aruco_dictionary", ARUCO_DICTIONARY_NAME)
        self.declare_parameter("aruco_log_ids", True)
        self.declare_parameter("use_yolo", True)
        self.declare_parameter("yolo_model_path", YOLO_DEFAULT_MODEL_PATH)
        self.declare_parameter("yolo_conf", 0.25)
        self.declare_parameter("yolo_device", "0")
        self.declare_parameter("yolo_log_detections", True)
        self.declare_parameter("debug_view", True)

        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.fps = int(self.get_parameter("fps").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.debug_view = bool(self.get_parameter("debug_view").value)
        self.ocr_enabled = can_use_ocr()
        self.ocr_executor = ThreadPoolExecutor(max_workers=1)
        self.ocr_future: Optional[Future] = None
        self.last_ocr_time = 0.0
        self.last_ocr_text = ""
        self.aruco_detector = self._create_configured_aruco_detector()
        self.yolo_model = self._create_configured_yolo_model()

        self.center_pub = self.create_publisher(PointStamped, "medicine_box/center_point", 10)
        self.centers_pub = self.create_publisher(PoseArray, "medicine_box/center_points", 10)
        self.count_pub = self.create_publisher(Int32, "medicine_box/count", 10)
        self.aruco_center_pub = self.create_publisher(PointStamped, "aruco/center_point", 10)
        self.aruco_centers_pub = self.create_publisher(PoseArray, "aruco/center_points", 10)
        self.aruco_count_pub = self.create_publisher(Int32, "aruco/count", 10)
        self.yolo_center_pub = self.create_publisher(PointStamped, "yolo/center_point", 10)
        self.yolo_centers_pub = self.create_publisher(PoseArray, "yolo/center_points", 10)
        self.yolo_count_pub = self.create_publisher(Int32, "yolo/count", 10)

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        profile = self.pipeline.start(config)

        depth_sensor = profile.get_device().first_depth_sensor()
        if depth_sensor.supports(rs.option.visual_preset):
            depth_sensor.set_option(rs.option.visual_preset, 3)
        self.depth_scale = depth_sensor.get_depth_scale()
        self.align = rs.align(rs.stream.color)
        color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
        self.intrinsics = color_profile.get_intrinsics()

        self.timer = self.create_timer(1.0 / max(1, self.fps), self.process_frame)
        self.get_logger().info("medicine_box_center_detector started")

    def _create_configured_aruco_detector(self) -> Optional[Any]:
        if not bool(self.get_parameter("use_aruco").value):
            return None

        dictionary_name = str(self.get_parameter("aruco_dictionary").value)
        try:
            detector = create_aruco_detector(dictionary_name)
        except Exception as exc:  # noqa: B902
            self.get_logger().error(f"ArUco detector disabled: {exc}")
            return None

        self.get_logger().info(f"ArUco marker detection enabled ({dictionary_name})")
        return detector

    def _create_configured_yolo_model(self) -> Optional[Any]:
        if not bool(self.get_parameter("use_yolo").value):
            return None
        if YOLO is None:
            self.get_logger().error("YOLO disabled: ultralytics is not installed.")
            return None

        model_path = Path(str(self.get_parameter("yolo_model_path").value)).expanduser()
        if not model_path.exists():
            self.get_logger().error(f"YOLO disabled: model not found: {model_path}")
            return None

        try:
            model = YOLO(str(model_path))
        except Exception as exc:  # noqa: B902
            self.get_logger().error(f"YOLO disabled: failed to load {model_path}: {exc}")
            return None

        self.get_logger().info(f"YOLO detection enabled: {model_path}")
        self.get_logger().info(f"YOLO names: {model.names}")
        return model

    def destroy_node(self) -> bool:
        self.ocr_executor.shutdown(wait=False, cancel_futures=True)
        self.pipeline.stop()
        if self.debug_view:
            cv2.destroyAllWindows()
        return super().destroy_node()

    def process_frame(self) -> None:
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
        except RuntimeError as exc:
            self.get_logger().warn(f"RealSense frame timeout: {exc}")
            return

        aligned = self.align.process(frames)
        depth_frame = aligned.get_depth_frame()
        color_frame = aligned.get_color_frame()
        if not depth_frame or not color_frame:
            return

        color = np.asanyarray(color_frame.get_data())
        depth_m = np.asanyarray(depth_frame.get_data()).astype(np.float32) * self.depth_scale
        aruco_markers = self._detect_aruco_markers(color, depth_m)
        yolo_detections = self._detect_yolo_boxes(color, depth_m)
        faces = self._detect_faces(color, depth_m)
        boxes = self._group_faces_into_boxes(faces)

        if bool(self.get_parameter("publish_nearest_only").value) and boxes:
            boxes = [min(boxes, key=lambda box: box.point_m[2])]
            faces = boxes[0].faces

        self._publish_boxes(boxes)
        self._publish_aruco_markers(aruco_markers)
        self._publish_yolo_detections(yolo_detections)
        self._handle_realtime_ocr(color, [box.grasp_face for box in boxes])

        if self.debug_view:
            self._show_debug(color, faces, boxes, aruco_markers, yolo_detections)

    def _detect_aruco_markers(
        self,
        color: np.ndarray,
        depth_m: np.ndarray,
    ) -> List[ArucoMarker]:
        if self.aruco_detector is None:
            return []

        gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.aruco_detector.detectMarkers(gray)
        if ids is None:
            return []

        markers: List[ArucoMarker] = []
        for index, marker_id in enumerate(ids.flatten()):
            marker_corners = corners[index][0]
            center_px = (
                int(round(float(marker_corners[:, 0].mean()))),
                int(round(float(marker_corners[:, 1].mean()))),
            )
            point_m = self._point_from_depth(depth_m, center_px)
            markers.append(
                ArucoMarker(
                    marker_id=int(marker_id),
                    corners=marker_corners.astype(np.float32),
                    center_px=center_px,
                    point_m=point_m,
                )
            )

        if bool(self.get_parameter("aruco_log_ids").value) and markers:
            ids_text = ", ".join(str(marker.marker_id) for marker in markers)
            self.get_logger().info(f"ArUco IDs: {ids_text}", throttle_duration_sec=0.5)
        return markers

    def _detect_yolo_boxes(
        self,
        color: np.ndarray,
        depth_m: np.ndarray,
    ) -> List[YoloDetection]:
        if self.yolo_model is None:
            return []

        conf = float(self.get_parameter("yolo_conf").value)
        device = str(self.get_parameter("yolo_device").value)
        try:
            result = self.yolo_model.predict(
                color,
                conf=conf,
                device=device,
                verbose=False,
            )[0]
        except Exception as exc:  # noqa: B902
            self.get_logger().error(f"YOLO prediction failed: {exc}", throttle_duration_sec=2.0)
            return []

        detections: List[YoloDetection] = []
        if result.boxes is None:
            return detections

        names = getattr(result, "names", None) or getattr(self.yolo_model, "names", {})
        for box in result.boxes:
            x1, y1, x2, y2 = [int(round(v)) for v in box.xyxy[0].tolist()]
            x1 = max(0, min(self.width - 1, x1))
            y1 = max(0, min(self.height - 1, y1))
            x2 = max(0, min(self.width - 1, x2))
            y2 = max(0, min(self.height - 1, y2))
            if x2 <= x1 or y2 <= y1:
                continue

            class_id = int(box.cls[0].item()) if box.cls is not None else -1
            confidence = float(box.conf[0].item()) if box.conf is not None else 0.0
            class_name = str(names.get(class_id, class_id)) if isinstance(names, dict) else str(class_id)
            center_px = ((x1 + x2) // 2, (y1 + y2) // 2)
            point_m = self._point_from_depth(depth_m, center_px)
            detections.append(
                YoloDetection(
                    class_id=class_id,
                    class_name=class_name,
                    confidence=confidence,
                    xyxy=(x1, y1, x2, y2),
                    center_px=center_px,
                    point_m=point_m,
                )
            )

        if bool(self.get_parameter("yolo_log_detections").value) and detections:
            summary = ", ".join(
                f"{det.class_name} {det.confidence:.2f} px={det.center_px}"
                for det in detections
            )
            self.get_logger().info(f"YOLO detections: {summary}", throttle_duration_sec=0.5)
        return detections

    def _detect_faces(self, color: np.ndarray, depth_m: np.ndarray) -> List[BoxFace]:
        x0, x1, y0, y1 = self._roi_bounds()
        roi_color = color[y0:y1, x0:x1]

        gray = cv2.cvtColor(roi_color, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(gray, 45, 130)
        kernel = np.ones((5, 5), np.uint8)
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=2)
        edges = cv2.dilate(edges, kernel, iterations=1)

        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        faces: List[BoxFace] = []
        min_area = float(self.get_parameter("min_area_px").value)
        max_area = float(self.get_parameter("max_area_px").value)
        min_ratio = float(self.get_parameter("min_aspect_ratio").value)
        max_ratio = float(self.get_parameter("max_aspect_ratio").value)

        for contour in contours:
            contour = contour + np.array([[[x0, y0]]], dtype=contour.dtype)
            area = cv2.contourArea(contour)
            if area < min_area or area > max_area:
                continue

            rect = cv2.minAreaRect(contour)
            (cx, cy), (rw, rh), _ = rect
            if rw < 12 or rh < 12:
                continue

            ratio = max(rw, rh) / max(1.0, min(rw, rh))
            if ratio < min_ratio or ratio > max_ratio:
                continue

            box = cv2.boxPoints(rect).astype(np.float32)
            rectangularity = area / max(1.0, rw * rh)
            if rectangularity < 0.45:
                continue

            center_px = (int(round(cx)), int(round(cy)))
            depth = self._median_depth(depth_m, center_px)
            if depth is None:
                continue

            point = rs.rs2_deproject_pixel_to_point(
                self.intrinsics,
                [float(center_px[0]), float(center_px[1])],
                depth,
            )
            normal = self._fit_face_normal(depth_m, box)
            top_score = float(np.dot(np.array(point, dtype=np.float32), self._floor_up_vector()))
            faces.append(
                BoxFace(
                    contour=box.reshape((-1, 1, 2)).astype(np.int32),
                    rect=rect,
                    center_px=center_px,
                    point_m=(float(point[0]), float(point[1]), float(point[2])),
                    area_px=float(area),
                    normal_m=normal,
                    top_score_m=top_score,
                )
            )

        return self._remove_duplicate_faces(faces)

    def _floor_up_vector(self) -> np.ndarray:
        vector = np.array(
            [
                float(self.get_parameter("drawer_floor_up_x").value),
                float(self.get_parameter("drawer_floor_up_y").value),
                float(self.get_parameter("drawer_floor_up_z").value),
            ],
            dtype=np.float32,
        )
        norm = float(np.linalg.norm(vector))
        if norm < 1e-6:
            return np.array([0.0, 0.0, -1.0], dtype=np.float32)
        return vector / norm

    def _fit_face_normal(
        self,
        depth_m: np.ndarray,
        box: np.ndarray,
    ) -> Optional[Tuple[float, float, float]]:
        mask = np.zeros((self.height, self.width), dtype=np.uint8)
        cv2.fillConvexPoly(mask, box.astype(np.int32), 255)
        x, y, w, h = cv2.boundingRect(box.astype(np.int32))
        step = max(1, int(self.get_parameter("face_plane_sample_step_px").value))
        min_points = int(self.get_parameter("face_plane_min_points").value)
        min_depth = float(self.get_parameter("min_depth_m").value)
        max_depth = float(self.get_parameter("max_depth_m").value)
        points = []

        x_end = min(self.width, x + w)
        y_end = min(self.height, y + h)
        for v in range(max(0, y), y_end, step):
            for u in range(max(0, x), x_end, step):
                if mask[v, u] == 0:
                    continue
                depth = float(depth_m[v, u])
                if depth < min_depth or depth > max_depth:
                    continue
                point = rs.rs2_deproject_pixel_to_point(
                    self.intrinsics,
                    [float(u), float(v)],
                    depth,
                )
                points.append(point)

        if len(points) < min_points:
            return None

        pts = np.array(points, dtype=np.float32)
        pts = pts - np.mean(pts, axis=0)
        _, _, vh = np.linalg.svd(pts, full_matrices=False)
        normal = vh[-1].astype(np.float32)
        norm = float(np.linalg.norm(normal))
        if norm < 1e-6:
            return None
        normal = normal / norm
        floor_up = self._floor_up_vector()
        if float(np.dot(normal, floor_up)) < 0.0:
            normal = -normal
        return (float(normal[0]), float(normal[1]), float(normal[2]))

    def _median_depth(self, depth_m: np.ndarray, center_px: Tuple[int, int]) -> Optional[float]:
        u, v = center_px
        window = int(self.get_parameter("depth_window_px").value)
        if window % 2 == 0:
            window += 1
        half = max(1, window // 2)
        x0 = max(0, u - half)
        x1 = min(self.width, u + half + 1)
        y0 = max(0, v - half)
        y1 = min(self.height, v + half + 1)
        patch = depth_m[y0:y1, x0:x1]
        min_depth = float(self.get_parameter("min_depth_m").value)
        max_depth = float(self.get_parameter("max_depth_m").value)
        valid = patch[(patch >= min_depth) & (patch <= max_depth)]
        if valid.size < 4:
            return None
        return float(np.median(valid))

    def _point_from_depth(
        self,
        depth_m: np.ndarray,
        center_px: Tuple[int, int],
    ) -> Optional[Tuple[float, float, float]]:
        depth = self._median_depth(depth_m, center_px)
        if depth is None:
            return None

        point = rs.rs2_deproject_pixel_to_point(
            self.intrinsics,
            [float(center_px[0]), float(center_px[1])],
            depth,
        )
        return (float(point[0]), float(point[1]), float(point[2]))

    def _remove_duplicate_faces(self, faces: List[BoxFace]) -> List[BoxFace]:
        faces = sorted(faces, key=lambda face: face.area_px, reverse=True)
        kept: List[BoxFace] = []
        for face in faces:
            cx, cy = face.center_px
            duplicate = False
            for other in kept:
                ox, oy = other.center_px
                if np.hypot(cx - ox, cy - oy) < 35:
                    duplicate = True
                    break
            if not duplicate:
                kept.append(face)
        return sorted(kept, key=lambda face: face.center_px[0])

    def _group_faces_into_boxes(self, faces: List[BoxFace]) -> List[MedicineBox]:
        clusters: List[List[BoxFace]] = []

        for face in sorted(faces, key=lambda item: item.center_px[0]):
            for cluster in clusters:
                if any(self._same_box(face, other) for other in cluster):
                    cluster.append(face)
                    break
            else:
                clusters.append([face])

        boxes = [self._make_box(cluster) for cluster in clusters]
        return sorted(boxes, key=lambda box: box.center_px[0])

    def _same_box(self, first: BoxFace, second: BoxFace) -> bool:
        max_px = float(self.get_parameter("box_cluster_distance_px").value)
        max_m = float(self.get_parameter("box_cluster_distance_m").value)
        px_dist = np.hypot(
            first.center_px[0] - second.center_px[0],
            first.center_px[1] - second.center_px[1],
        )
        xyz_dist = np.linalg.norm(np.array(first.point_m) - np.array(second.point_m))
        return px_dist <= max_px and xyz_dist <= max_m

    def _make_box(self, faces: List[BoxFace]) -> MedicineBox:
        grasp_face = self._select_grasp_face(faces)
        return MedicineBox(
            faces=faces,
            grasp_face=grasp_face,
            center_px=grasp_face.center_px,
            point_m=grasp_face.point_m,
        )

    @staticmethod
    def _select_grasp_face_static(faces: List[BoxFace]) -> BoxFace:
        return max(faces, key=lambda face: (face.top_score_m, face.area_px))

    def _select_grasp_face(self, faces: List[BoxFace]) -> BoxFace:
        floor_up = self._floor_up_vector()
        min_cos = float(self.get_parameter("top_face_parallel_cos_min").value)
        parallel_faces = []
        for face in faces:
            if face.normal_m is None:
                continue
            normal = np.array(face.normal_m, dtype=np.float32)
            if abs(float(np.dot(normal, floor_up))) >= min_cos:
                parallel_faces.append(face)
        if parallel_faces:
            return max(parallel_faces, key=lambda face: (face.top_score_m, face.area_px))
        return self._select_grasp_face_static(faces)

    def _publish_boxes(self, boxes: List[MedicineBox]) -> None:
        stamp = self.get_clock().now().to_msg()
        pose_array = PoseArray()
        pose_array.header.stamp = stamp
        pose_array.header.frame_id = self.camera_frame

        for index, box in enumerate(boxes):
            point_msg = PointStamped()
            point_msg.header.stamp = stamp
            point_msg.header.frame_id = self.camera_frame
            point_msg.point.x = box.point_m[0]
            point_msg.point.y = box.point_m[1]
            point_msg.point.z = box.point_m[2]
            if index == 0:
                self.center_pub.publish(point_msg)

            pose = Pose()
            pose.position = point_msg.point
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)

        self.centers_pub.publish(pose_array)
        self.count_pub.publish(Int32(data=len(boxes)))
        if boxes:
            coords = ", ".join(
                f"#{i}: px={box.center_px}, xyz=({box.point_m[0]:.3f}, "
                f"{box.point_m[1]:.3f}, {box.point_m[2]:.3f})m, "
                f"faces={len(box.faces)}"
                for i, box in enumerate(boxes, start=1)
            )
            self.get_logger().info(f"boxes={len(boxes)} {coords}", throttle_duration_sec=0.5)

    def _publish_aruco_markers(self, markers: List[ArucoMarker]) -> None:
        stamp = self.get_clock().now().to_msg()
        pose_array = PoseArray()
        pose_array.header.stamp = stamp
        pose_array.header.frame_id = self.camera_frame

        published_first = False
        for marker in markers:
            if marker.point_m is None:
                continue

            point_msg = PointStamped()
            point_msg.header.stamp = stamp
            point_msg.header.frame_id = self.camera_frame
            point_msg.point.x = marker.point_m[0]
            point_msg.point.y = marker.point_m[1]
            point_msg.point.z = marker.point_m[2]
            if not published_first:
                self.aruco_center_pub.publish(point_msg)
                published_first = True

            pose = Pose()
            pose.position = point_msg.point
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)

        self.aruco_centers_pub.publish(pose_array)
        self.aruco_count_pub.publish(Int32(data=len(markers)))

    def _publish_yolo_detections(self, detections: List[YoloDetection]) -> None:
        stamp = self.get_clock().now().to_msg()
        pose_array = PoseArray()
        pose_array.header.stamp = stamp
        pose_array.header.frame_id = self.camera_frame

        published_first = False
        for detection in detections:
            if detection.point_m is None:
                continue

            point_msg = PointStamped()
            point_msg.header.stamp = stamp
            point_msg.header.frame_id = self.camera_frame
            point_msg.point.x = detection.point_m[0]
            point_msg.point.y = detection.point_m[1]
            point_msg.point.z = detection.point_m[2]
            if not published_first:
                self.yolo_center_pub.publish(point_msg)
                published_first = True

            pose = Pose()
            pose.position = point_msg.point
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)

        self.yolo_centers_pub.publish(pose_array)
        self.yolo_count_pub.publish(Int32(data=len(detections)))


    def _roi_bounds(self) -> Tuple[int, int, int, int]:
        x0 = int(self.width * float(self.get_parameter("roi_x_min").value))
        x1 = int(self.width * float(self.get_parameter("roi_x_max").value))
        y0 = int(self.height * float(self.get_parameter("roi_y_min").value))
        y1 = int(self.height * float(self.get_parameter("roi_y_max").value))
        return max(0, x0), min(self.width, x1), max(0, y0), min(self.height, y1)

    def _handle_realtime_ocr(self, color: np.ndarray, faces: List[BoxFace]) -> None:
        if not bool(self.get_parameter("realtime_ocr").value):
            return
        if not self.ocr_enabled:
            return
        if self.ocr_future is not None and self.ocr_future.done():
            self._consume_ocr_result(self.ocr_future)
            self.ocr_future = None
        if self.ocr_future is not None:
            return

        now = time.monotonic()
        interval = float(self.get_parameter("ocr_interval_sec").value)
        if now - self.last_ocr_time < interval:
            return
        if not faces:
            return

        image = color
        if bool(self.get_parameter("ocr_crop_face").value):
            image = self._extract_face_crop(color, faces[0])

        image_path = str(self.get_parameter("ocr_image_path").value)
        if not cv2.imwrite(image_path, image):
            self.get_logger().error(f"failed to save image: {image_path}")
            return

        self.last_ocr_time = now
        self.ocr_future = self.ocr_executor.submit(run_ocr, image_path)

    def _consume_ocr_result(self, future: Future) -> None:
        try:
            text = str(future.result()).strip()
        except Exception as exc:  # noqa: B902
            self.get_logger().error(f"OCR failed: {exc}")
            return

        min_len = int(self.get_parameter("ocr_min_text_length").value)
        if len(text) >= min_len and text != self.last_ocr_text:
            self.last_ocr_text = text
            self.get_logger().info(f"OCR result:\n{text}")

    def _extract_face_crop(self, color: np.ndarray, face: BoxFace) -> np.ndarray:
        points = face.contour.reshape(4, 2).astype(np.float32)
        ordered = self._order_points(points)
        top_width = np.linalg.norm(ordered[1] - ordered[0])
        bottom_width = np.linalg.norm(ordered[2] - ordered[3])
        left_height = np.linalg.norm(ordered[3] - ordered[0])
        right_height = np.linalg.norm(ordered[2] - ordered[1])
        width = max(1, int(max(top_width, bottom_width)))
        height = max(1, int(max(left_height, right_height)))
        target = np.array(
            [[0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1]],
            dtype=np.float32,
        )
        matrix = cv2.getPerspectiveTransform(ordered, target)
        return cv2.warpPerspective(color, matrix, (width, height))

    @staticmethod
    def _order_points(points: np.ndarray) -> np.ndarray:
        ordered = np.zeros((4, 2), dtype=np.float32)
        summed = points.sum(axis=1)
        diffed = np.diff(points, axis=1).reshape(-1)
        ordered[0] = points[np.argmin(summed)]
        ordered[2] = points[np.argmax(summed)]
        ordered[1] = points[np.argmin(diffed)]
        ordered[3] = points[np.argmax(diffed)]
        return ordered

    def _show_debug(
        self,
        color: np.ndarray,
        faces: List[BoxFace],
        boxes: List[MedicineBox],
        aruco_markers: List[ArucoMarker],
        yolo_detections: List[YoloDetection],
    ) -> None:
        view = color.copy()
        x0, x1, y0, y1 = self._roi_bounds()
        cv2.rectangle(view, (x0, y0), (x1, y1), (80, 180, 255), 1)

        cv2.putText(
            view,
            f"Box Count: {len(boxes)}",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

        for face in faces:
            cv2.drawContours(view, [face.contour], -1, (0, 255, 0), 2)
            cv2.circle(view, face.center_px, 4, (0, 0, 255), -1)

        for index, box in enumerate(boxes, start=1):
            cv2.circle(view, box.center_px, 7, (255, 0, 255), -1)
            label = (
                f"{index}: {box.point_m[0]:.2f}, "
                f"{box.point_m[1]:.2f}, {box.point_m[2]:.2f}m"
            )
            cv2.putText(
                view,
                label,
                (box.center_px[0] + 8, box.center_px[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (255, 0, 255),
                1,
                cv2.LINE_AA,
            )

        for marker in aruco_markers:
            corners = marker.corners.astype(np.int32).reshape((-1, 1, 2))
            cv2.polylines(view, [corners], True, (0, 255, 255), 2)
            cv2.circle(view, marker.center_px, 4, (0, 255, 255), -1)
            if marker.point_m is None:
                label = f"ArUco {marker.marker_id}"
            else:
                label = (
                    f"ArUco {marker.marker_id}: {marker.point_m[0]:.2f}, "
                    f"{marker.point_m[1]:.2f}, {marker.point_m[2]:.2f}m"
                )
            cv2.putText(
                view,
                label,
                (marker.center_px[0] + 8, marker.center_px[1] + 18),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 255, 255),
                1,
                cv2.LINE_AA,
            )

        for detection in yolo_detections:
            x1, y1, x2, y2 = detection.xyxy
            cv2.rectangle(view, (x1, y1), (x2, y2), (255, 80, 0), 2)
            if detection.point_m is None:
                label = f"YOLO {detection.class_name} {detection.confidence:.2f}"
            else:
                label = (
                    f"YOLO {detection.class_name} {detection.confidence:.2f}: "
                    f"{detection.point_m[2]:.2f}m"
                )
            cv2.putText(
                view,
                label,
                (x1, max(20, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 80, 0),
                2,
                cv2.LINE_AA,
            )

        if self.last_ocr_text:
            preview = self.last_ocr_text.splitlines()[0][:60]
            cv2.putText(
                view,
                f"OCR: {preview}",
                (12, self.height - 16),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 0),
                2,
                cv2.LINE_AA,
            )

        cv2.imshow("medicine_box_center_detector", view)
        cv2.waitKey(1)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = MedicineBoxCenterDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
