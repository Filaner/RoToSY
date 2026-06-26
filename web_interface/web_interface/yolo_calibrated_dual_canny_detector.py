#!/usr/bin/env python3
"""YOLO Bounding Box + Calibrated Dual Canny 상자 윗면 검출 및 규격 크롭 보정 프로그램.

YOLO BBox 내 컬러/뎁스 에지를 3D 공간으로 역투영하여 로봇 베이스 프레임으로 변환합니다.
실시간으로 마우스 클릭 또는 키보드(2,3,4,5) 입력을 받아 서랍장 치수를 변경하고, 검출 오차가 클 경우
약상자의 가로세로 치수 스펙을 참고해 에지 영역을 크롭(보정)하여 정밀 윗면 중심을 검출합니다.
"""

import argparse
from pathlib import Path
import cv2
import numpy as np
import pyrealsense2 as rs
import yaml
from ultralytics import YOLO
import PIL.Image as PILImage
import PIL.ImageDraw as PILImageDraw
import PIL.ImageFont as PILImageFont

MEDICINE_CLASSES = {"medicine_box", "water_pack"}
DEFAULT_MODEL_PATH = Path("/home/user/RoToSY/src/web_interface/web_interface/models/best.pt")
DEFAULT_CALIBRATION = Path("/home/user/RoToSY/src/rotosy_calibration/config/camera_extrinsic.yaml")

FONT_PATH = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
_cached_font = {}

def get_font(size):
    """지정한 크기의 Pillow 폰트 객체를 캐싱하여 반환합니다."""
    if size not in _cached_font:
        try:
            _cached_font[size] = PILImageFont.truetype(FONT_PATH, size)
        except Exception:
            _cached_font[size] = PILImageFont.load_default()
    return _cached_font[size]

def draw_korean_text(img, text, position, font_size=15, color=(255, 255, 255)):
    """OpenCV BGR 이미지에 Pillow를 사용하여 한글 텍스트를 그리는 함수."""
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    pil_img = PILImage.fromarray(img_rgb)
    draw = PILImageDraw.Draw(pil_img)
    font = get_font(font_size)
    
    # PIL fill color는 RGB 순서이므로 (R, G, B)로 매핑.
    # 입력 color가 BGR 형태(OpenCV 표준)이므로 역순으로 전달.
    draw.text(position, text, font=font, fill=(color[2], color[1], color[0]))
    
    return cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)
DEFAULT_CALIBRATION = Path("/home/user/RoToSY/src/rotosy_calibration/config/camera_extrinsic.yaml")

# 서랍 번호별 스펙 데이터 (단위: mm)
# Drawer ID -> (약 이름, [(가로, 세로) 윗면 조합 치수 후보])
DRAWER_SPECS = {
    2: ("수액팩", []),
    3: ("심미안정", [(78.0, 107.0)]),
    4: ("일본 유산균", [(57.0, 102.0), (55.0, 102.0)]),
    5: ("벤포벨S", [(57.0, 110.0), (56.0, 110.0)])
}

# 전역 상태 변수
current_drawer = 3

def quaternion_to_matrix(quaternion_xyzw: list[float]) -> np.ndarray:
    """단위 쿼터니언 [x, y, z, w]를 3x3 직교 회전 행렬(Rotation Matrix)로 변환하는 함수."""
    x, y, z, w = (float(v) for v in quaternion_xyzw)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)

def load_base_from_camera_transform(path: Path) -> np.ndarray:
    """카메라 외부 변수 캘리브레이션 yaml 파일을 로드하여 4x4 동차 변환 행렬을 빌드하는 함수."""
    with path.open(encoding="utf-8") as file:
        params = yaml.safe_load(file)["camera_extrinsic"]["ros__parameters"]
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = quaternion_to_matrix(params["rotation_quat_xyzw"])
    transform[:3, 3] = np.asarray(params["translation_xyz"], dtype=np.float64)
    return transform


def rectangle_perimeter_support(
    local_points: np.ndarray,
    half_width_m: float,
    half_height_m: float,
    band_m: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Return points near the four finite sides of a rotated rectangle.

    The point coordinates are already expressed in the rectangle's local XY
    axes.  Counting only these points prevents interior texture edges from
    improving a size-fit score.
    """
    within_width = np.abs(local_points[:, 0]) <= half_width_m + band_m
    within_height = np.abs(local_points[:, 1]) <= half_height_m + band_m
    left = (np.abs(local_points[:, 0] + half_width_m) <= band_m) & within_height
    right = (np.abs(local_points[:, 0] - half_width_m) <= band_m) & within_height
    top = (np.abs(local_points[:, 1] + half_height_m) <= band_m) & within_width
    bottom = (np.abs(local_points[:, 1] - half_height_m) <= band_m) & within_width
    support_mask = left | right | top | bottom
    side_counts = np.array([
        np.count_nonzero(left),
        np.count_nonzero(right),
        np.count_nonzero(top),
        np.count_nonzero(bottom),
    ])
    return support_mask, side_counts


def bbox_iou(a, b) -> float:
    """Intersection-over-union for matching a face hold to a YOLO box."""
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    intersection = max(0, ix2 - ix1) * max(0, iy2 - iy1)
    if not intersection:
        return 0.0
    area_a = max(0, ax2 - ax1) * max(0, ay2 - ay1)
    area_b = max(0, bx2 - bx1) * max(0, by2 - by1)
    union = area_a + area_b - intersection
    return float(intersection / union) if union else 0.0


def mouse_callback(event, x, y, flags, param) -> None:
    """화면 하단 버튼 클릭 시 서랍 번호를 실시간 변경하는 마우스 콜백 함수."""
    global current_drawer
    if event == cv2.EVENT_LBUTTONDOWN:
        # 버튼 영역 좌표 범위 검사 (1280x480 프레임 이미지 기준)
        # 각 버튼 가로 크기: 180, 높이: 40, Y축: 430 ~ 470
        for i, btn in enumerate([2, 3, 4, 5]):
            btn_x1 = 20 + i * 200
            btn_x2 = btn_x1 + 180
            if btn_x1 <= x <= btn_x2 and 430 <= y <= 470:
                current_drawer = btn
                print(f"Mouse Click: Selected Drawer {current_drawer} ({DRAWER_SPECS[current_drawer][0]})")

def main() -> None:
    global current_drawer
    parser = argparse.ArgumentParser(description="YOLO BBox + Calibrated Dual Canny Top Face Detector with Size Fitting")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL_PATH, help="YOLO .pt model path")
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION, help="camera_extrinsic.yaml path")
    parser.add_argument("--conf", type=float, default=0.50, help="YOLO confidence threshold")
    parser.add_argument("--color-low", type=int, default=30, help="Color Canny low threshold")
    parser.add_argument("--color-high", type=int, default=90, help="Color Canny high threshold")
    parser.add_argument("--depth-low", type=int, default=15, help="Depth Canny low threshold")
    parser.add_argument("--depth-high", type=int, default=50, help="Depth Canny high threshold")
    parser.add_argument("--band-mm", type=float, default=14.0, help="Height band (in mm) from the absolute highest edge point")
    parser.add_argument(
        "--top-z-reference",
        choices=("max_edge", "bbox_center"),
        default="max_edge",
        help=(
            "Z-band reference: max_edge uses the highest edge point; "
            "bbox_center uses the YOLO bounding-box center depth."
        ),
    )
    parser.add_argument(
        "--top-z-above-mm",
        type=float,
        default=None,
        help="Optional upper Z allowance above the reference height in mm.",
    )
    parser.add_argument(
        "--size-fit-score",
        choices=("inside", "perimeter"),
        default="inside",
        help=(
            "inside counts all points inside the size template (v1); "
            "perimeter scores only points close to its four sides (v2)."
        ),
    )
    parser.add_argument(
        "--edge-band-mm",
        type=float,
        default=4.0,
        help="Perimeter score: maximum edge-to-template-side distance in mm.",
    )
    parser.add_argument(
        "--min-edge-points-per-side",
        type=int,
        default=0,
        help="Perimeter score: minimum supporting edge points required on each side.",
    )
    parser.add_argument(
        "--hold-frames",
        type=int,
        default=0,
        help="Keep the last valid top-face result for this many missed frames.",
    )
    args = parser.parse_args()

    if not args.model.exists():
        parser.error(f"YOLO model not found at: {args.model}")
    if not args.calibration.exists():
        parser.error(f"Calibration file not found at: {args.calibration}")

    # Load calibration transform
    print(f"Loading calibration: {args.calibration}...")
    base_from_camera = load_base_from_camera_transform(args.calibration)
    rotation, translation = base_from_camera[:3, :3], base_from_camera[:3, 3]

    # Load YOLO Model
    print(f"Loading YOLO model: {args.model}...")
    model = YOLO(str(args.model))
    names = dict(model.names)

    # Configure RealSense aligned pipeline
    print("Starting RealSense color & depth streams...")
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    profile = pipeline.start(config)
    
    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
    align = rs.align(rs.stream.color)
    color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
    intrinsics = color_profile.get_intrinsics()

    # 윈도우 이름 설정 및 마우스 콜백 등록
    window_name = "YOLO BBox Calibrated Dual Canny"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 2560, 960)
    cv2.setMouseCallback(window_name, mouse_callback)

    print("Pipeline started. Press 'q' or 'Esc' to exit. Click buttons on screen or press 2,3,4,5 to select drawer.")

    # Per-YOLO-box top-face holds.  v2 uses this to hide one-frame Canny/
    # depth dropouts without changing the original v1 behavior.
    face_tracks = []

    try:
        while True:
            frames = align.process(pipeline.wait_for_frames())
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            color = np.asanyarray(color_frame.get_data())
            depth = np.asanyarray(depth_frame.get_data())
            annotated = color.copy()
            height, width = color.shape[:2]
            detected_faces = []

            # 실시간 서랍장 정보 텍스트 오버레이 (두 화면 모두 상단에 표시)
            drawer_name, candidates = DRAWER_SPECS[current_drawer]
            candidate_strs = [f"{w}x{h}mm" for w, h in candidates]
            spec_desc = ", ".join(candidate_strs) if candidate_strs else "스펙 없음"
            info_text = f"Drawer: {current_drawer} ({drawer_name}) | Target Spec: {spec_desc}"
            color = draw_korean_text(color, info_text, (20, 15), font_size=18, color=(255, 255, 0))
            annotated = draw_korean_text(annotated, info_text, (20, 15), font_size=18, color=(255, 255, 0))

            # YOLO inference
            results = model(color, conf=args.conf, verbose=False)[0]

            if results.boxes is not None:
                for box in results.boxes:
                    class_name = str(names.get(int(box.cls[0]), int(box.cls[0])))
                    confidence = float(box.conf[0])
                    if class_name.lower() not in MEDICINE_CLASSES:
                        continue

                    # BBox coordinates
                    x1, y1, x2, y2 = (int(value) for value in box.xyxy[0].tolist())
                    x1, x2 = max(0, x1), min(width, x2)
                    y1, y2 = max(0, y1), min(height, y2)

                    # Draw original YOLO BBox
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 255), 1)

                    if (x2 - x1) <= 10 or (y2 - y1) <= 10:
                        continue

                    # Extract ROI
                    roi_color = color[y1:y2, x1:x2]
                    roi_depth = depth[y1:y2, x1:x2].astype(np.float32) * depth_scale

                    # 1. Color Canny
                    gray = cv2.cvtColor(roi_color, cv2.COLOR_BGR2GRAY)
                    blurred_color = cv2.GaussianBlur(gray, (5, 5), 0)
                    edges_color = cv2.Canny(blurred_color, args.color_low, args.color_high)

                    # 2. Depth Canny
                    valid_mask = (roi_depth > 0.05) & (roi_depth < 2.0)
                    edges_depth = np.zeros_like(edges_color)
                    
                    if np.count_nonzero(valid_mask) >= 20:
                        min_d = roi_depth[valid_mask].min()
                        max_d = roi_depth[valid_mask].max()
                        roi_depth_8bit = np.zeros(roi_depth.shape, dtype=np.uint8)
                        if max_d > min_d:
                            normalized = ((roi_depth - min_d) / (max_d - min_d) * 255.0)
                            roi_depth_8bit[valid_mask] = normalized[valid_mask].astype(np.uint8)
                        
                        blurred_depth = cv2.GaussianBlur(roi_depth_8bit, (5, 5), 0)
                        edges_depth = cv2.Canny(blurred_depth, args.depth_low, args.depth_high)

                    # 3. Create 2D Overlay representation
                    color_mask = (edges_color > 0)
                    depth_mask = (edges_depth > 0)
                    intersection_mask = color_mask & depth_mask
                    
                    roi_annotated = annotated[y1:y2, x1:x2]
                    roi_annotated[color_mask] = (0, 0, 255)       # Red
                    roi_annotated[depth_mask] = (0, 255, 0)       # Green
                    roi_annotated[intersection_mask] = (255, 0, 0) # Blue

                    # 4. Deproject Union (Color or Depth) edges to 3D base frame
                    union_mask = color_mask | depth_mask
                    ys_local, xs_local = np.where(union_mask)
                    if len(xs_local) < 10:
                        continue

                    edge_points_base = []
                    edge_pixels_global = []
                    for local_y, local_x in zip(ys_local, xs_local):
                        global_x = x1 + local_x
                        global_y = y1 + local_y
                        d_m = roi_depth[local_y, local_x]
                        
                        if d_m <= 0.05 or d_m > 2.0:
                            continue

                        pt_cam = rs.rs2_deproject_pixel_to_point(intrinsics, [float(global_x), float(global_y)], float(d_m))
                        pt_base = (rotation @ np.array(pt_cam)) + translation
                        edge_points_base.append(pt_base)
                        edge_pixels_global.append((global_x, global_y))

                    if len(edge_points_base) < 10:
                        continue

                    edge_points_base = np.asarray(edge_points_base)
                    edge_pixels_global = np.asarray(edge_pixels_global)

                    # Filter edges by Z-axis height.  The default uses the
                    # highest edge point; v2 can instead anchor the band on
                    # the depth at the YOLO bounding-box center.
                    if args.top_z_reference == "bbox_center":
                        bbox_cx = (x1 + x2) // 2
                        bbox_cy = (y1 + y2) // 2
                        center_d_m = float(depth[bbox_cy, bbox_cx]) * depth_scale
                        if center_d_m <= 0.05 or center_d_m > 2.0:
                            continue
                        center_cam = rs.rs2_deproject_pixel_to_point(
                            intrinsics,
                            [float(bbox_cx), float(bbox_cy)],
                            center_d_m,
                        )
                        z_reference_m = float(
                            (rotation @ np.array(center_cam) + translation)[2]
                        )
                    else:
                        z_reference_m = float(np.max(edge_points_base[:, 2]))

                    top_z_values = edge_points_base[:, 2]
                    top_indices = top_z_values >= (
                        z_reference_m - args.band_mm / 1000.0
                    )
                    if args.top_z_above_mm is not None and args.top_z_above_mm >= 0:
                        top_indices &= top_z_values <= (
                            z_reference_m + args.top_z_above_mm / 1000.0
                        )
                    top_pixels = edge_pixels_global[top_indices]
                    top_points_3d = edge_points_base[top_indices]

                    if len(top_pixels) < 6:
                        continue

                    # 5. 가로/세로 물리 크기 측정 및 오차 분석 보정 (Crop)
                    points_2d = top_points_3d[:, :2].astype(np.float32)
                    rect = cv2.minAreaRect(points_2d)
                    (cx_base_rect, cy_base_rect), (width_m, height_m), angle = rect
                    width_mm = width_m * 1000.0
                    height_mm = height_m * 1000.0

                    # 뎁스 튀는 노이즈 차단을 위해 중앙값(Median)을 그리드 서치의 기준 중심으로 사용
                    cx_base = float(np.median(points_2d[:, 0]))
                    cy_base = float(np.median(points_2d[:, 1]))

                    current_candidates = [] if class_name.lower() == "water_pack" else candidates

                    best_w_tgt, best_h_tgt = None, None
                    min_error = float('inf')

                    if current_candidates:
                        for w_tgt, h_tgt in current_candidates:
                            err1 = abs(width_mm - w_tgt) + abs(height_mm - h_tgt)
                            err2 = abs(width_mm - h_tgt) + abs(height_mm - w_tgt)
                            if err1 < min_error:
                                min_error = err1
                                best_w_tgt, best_h_tgt = w_tgt, h_tgt
                            if err2 < min_error:
                                min_error = err2
                                best_w_tgt, best_h_tgt = h_tgt, w_tgt

                    # 스펙 규격 후보가 존재할 경우 항시 치수 기반 로컬 크롭(Crop)을 적용하여 에지 보정 수행
                    if best_w_tgt is not None:
                        rad = np.radians(angle)
                        cos_r, sin_r = np.cos(rad), np.sin(rad)
                        R_yaw = np.array([
                            [cos_r, -sin_r],
                            [sin_r, cos_r]
                        ])
                        
                        half_w_m = best_w_tgt / 2000.0
                        half_h_m = best_h_tgt / 2000.0
                        edge_band_m = args.edge_band_mm / 1000.0
                        
                        best_score = -1
                        opt_cx, opt_cy = cx_base, cy_base
                        search_range = np.linspace(-0.04, 0.04, 21)
                        for dx in search_range:
                            for dy in search_range:
                                tx = cx_base + dx
                                ty = cy_base + dy
                                local_pts = (points_2d - np.array([tx, ty])) @ R_yaw
                                if args.size_fit_score == "perimeter":
                                    _, side_counts = rectangle_perimeter_support(
                                        local_pts, half_w_m, half_h_m, edge_band_m
                                    )
                                    score = int(side_counts.sum() + 4 * side_counts.min())
                                else:
                                    score = int(np.count_nonzero(
                                        (np.abs(local_pts[:, 0]) <= half_w_m) &
                                        (np.abs(local_pts[:, 1]) <= half_h_m)
                                    ))
                                if score > best_score:
                                    best_score = score
                                    opt_cx, opt_cy = tx, ty
                        
                        # 탐색된 최적의 중심으로 최종 크롭 수행
                        local_pts = (points_2d - np.array([opt_cx, opt_cy])) @ R_yaw
                        if args.size_fit_score == "perimeter":
                            inside_mask, side_counts = rectangle_perimeter_support(
                                local_pts, half_w_m, half_h_m, edge_band_m
                            )
                            if side_counts.min() < args.min_edge_points_per_side:
                                continue
                        else:
                            inside_mask = (np.abs(local_pts[:, 0]) <= half_w_m) & \
                                          (np.abs(local_pts[:, 1]) <= half_h_m)
                        
                        top_pixels = top_pixels[inside_mask]
                        top_points_3d = top_points_3d[inside_mask]

                        # 깜빡임 최소화를 위해 임계값을 3으로 완화
                        if len(top_pixels) < 3:
                            continue

                    # 6. 최종 다각형 복원
                    if best_w_tgt is not None:
                        # 3D 베이스 좌표계상의 꼭짓점 계산
                        half_w = (best_w_tgt / 1000.0) / 2.0
                        half_h = (best_h_tgt / 1000.0) / 2.0
                        
                        # 로컬 꼭짓점 4개
                        local_corners = np.array([
                            [-half_w, -half_h],
                            [half_w, -half_h],
                            [half_w, half_h],
                            [-half_w, half_h]
                        ])
                        
                        # Z축 회전 적용 및 베이스 중심 변환
                        rad = np.radians(angle)
                        cos_r, sin_r = np.cos(rad), np.sin(rad)
                        R_yaw_rect = np.array([
                            [cos_r, -sin_r],
                            [sin_r, cos_r]
                        ])
                        
                        opt_cz = np.mean(top_points_3d[:, 2])
                        corners_base_3d = []
                        for pt in local_corners:
                            rotated_pt = R_yaw_rect @ pt
                            corners_base_3d.append([
                                opt_cx + rotated_pt[0],
                                opt_cy + rotated_pt[1],
                                opt_cz
                            ])
                        
                        # 3D Base -> 3D Camera -> 2D Pixel 사영
                        inv_rot = rotation.T
                        proj_corners = []
                        for pt_base in corners_base_3d:
                            pt_cam = inv_rot @ (np.array(pt_base) - translation)
                            pixel = rs.rs2_project_point_to_pixel(intrinsics, [float(pt_cam[0]), float(pt_cam[1]), float(pt_cam[2])])
                            proj_corners.append([int(pixel[0]), int(pixel[1])])
                        approx_polygon = np.array(proj_corners, dtype=np.int32)
                    else:
                        hull = cv2.convexHull(top_pixels.astype(np.int32))
                        perimeter = cv2.arcLength(hull, True)
                        approx_polygon = cv2.approxPolyDP(hull, 0.02 * perimeter, True).reshape(-1, 2)

                    if not 3 <= len(approx_polygon) <= 8:
                        continue

                    # 3D 깊이 노이즈로 인해 복원된 사각형이 YOLO 2D BBox 밖으로 튀는 것을 차단하는 안전장치
                    yolo_cx = (x1 + x2) / 2.0
                    yolo_cy = (y1 + y2) / 2.0
                    yolo_w = x2 - x1
                    yolo_h = y2 - y1
                    proj_cx = np.mean(approx_polygon[:, 0])
                    proj_cy = np.mean(approx_polygon[:, 1])
                    if abs(proj_cx - yolo_cx) > (yolo_w * 0.55) or abs(proj_cy - yolo_cy) > (yolo_h * 0.55):
                        continue

                    center_base_m = np.mean(top_points_3d, axis=0)

                    # Center
                    moments = cv2.moments(approx_polygon.astype(np.float32))
                    if abs(moments["m00"]) > 1e-6:
                        cx = int(moments["m10"] / moments["m00"])
                        cy = int(moments["m01"] / moments["m00"])
                    else:
                        cx, cy = int(np.mean(approx_polygon[:, 0])), int(np.mean(approx_polygon[:, 1]))
                    detected_faces.append({
                        'bbox': (x1, y1, x2, y2),
                        'polygon': approx_polygon.astype(np.int32),
                        'center': (cx, cy),
                        'z_m': float(center_base_m[2]),
                        'error_mm': float(min_error),
                        'missed_frames': 0,
                    })

            # Match current valid faces to their previous YOLO boxes.  A
            # previous face is drawn for a short time only when its matching
            # current box produced no valid top-face polygon this frame.
            matched_tracks = set()
            for face in detected_faces:
                best_index = None
                best_iou = 0.0
                for index, track in enumerate(face_tracks):
                    if index in matched_tracks:
                        continue
                    iou = bbox_iou(face['bbox'], track['bbox'])
                    if iou > best_iou:
                        best_iou, best_index = iou, index

                if best_index is not None and best_iou >= 0.3:
                    face_tracks[best_index].update(face)
                    matched_tracks.add(best_index)
                else:
                    face_tracks.append(face)
                    matched_tracks.add(len(face_tracks) - 1)

            for index, track in enumerate(face_tracks):
                if index not in matched_tracks:
                    track['missed_frames'] += 1

            hold_frames = max(0, args.hold_frames)
            face_tracks[:] = [
                track for track in face_tracks
                if track['missed_frames'] <= hold_frames
            ]

            for track in face_tracks:
                polygon = track['polygon']
                cx, cy = track['center']
                cv2.polylines(annotated, [polygon], True, (255, 255, 0), 2)
                cv2.circle(annotated, (cx, cy), 3, (255, 255, 0), -1)
                label = (
                    f"FACE DUAL z={track['z_m']*1000:.0f}mm "
                    f"Error={track['error_mm']:.1f}mm"
                )
                cv2.putText(
                    annotated, label, (cx + 6, cy + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 0), 1, cv2.LINE_AA,
                )

            # 원본 영상(color)과 검출 화면(annotated)을 가로로 합쳐서 하나의 창에 출력
            combined = np.hstack((color, annotated))

            # 7. 마우스로 클릭해서 서랍장을 선택하는 GUI 버튼 영역 그리기 (1280x480 프레임 이미지 하단)
            btn_w = 180
            btn_h = 40
            btn_y1 = 430
            btn_y2 = 470
            for i, btn in enumerate([2, 3, 4, 5]):
                btn_x1 = 20 + i * 200
                btn_x2 = btn_x1 + btn_w
                # 선택 상태에 따른 색상 및 채우기 두께 지정 (선택됨: 노랑 채움, 비선택: 회색 테두리)
                btn_color = (0, 255, 255) if current_drawer == btn else (120, 120, 120)
                thickness = -1 if current_drawer == btn else 2
                cv2.rectangle(combined, (btn_x1, btn_y1), (btn_x2, btn_y2), btn_color, thickness)
                
                # 버튼 내 텍스트 (서랍 번호와 실제 약 이름 표기)
                text_color = (0, 0, 0) if current_drawer == btn else (255, 255, 255)
                btn_text = f"{btn}: {DRAWER_SPECS[btn][0]}"
                combined = draw_korean_text(combined, btn_text, (btn_x1 + 12, btn_y1 + 10), font_size=15, color=text_color)

            cv2.imshow(window_name, combined)
            
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
            elif key == ord('2'):
                current_drawer = 2
                print("Keyboard Key 2: Selected Drawer 2 (수액팩)")
            elif key == ord('3'):
                current_drawer = 3
                print("Keyboard Key 3: Selected Drawer 3 (심미안정)")
            elif key == ord('4'):
                current_drawer = 4
                print("Keyboard Key 4: Selected Drawer 4 (일본 유산균)")
            elif key == ord('5'):
                current_drawer = 5
                print("Keyboard Key 5: Selected Drawer 5 (벤포벨S)")

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print("Pipeline stopped.")

if __name__ == "__main__":
    main()
