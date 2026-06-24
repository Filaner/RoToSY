"""
Motion Sequence Node with Interactive (Step-by-Step) Control.

Usage:
  1. Persistent Node:
     ros2 run doosan_controller motion_sequence
  2. One-shot CLI:
     ros2 run doosan_controller motion_sequence <drawer_number 1~6> [--step]
"""

import fcntl
import json
import math
import os
import re
import sys
import threading
import time
import urllib.request
from dataclasses import dataclass
from pathlib import Path

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Empty, String, Int32

from dsr_msgs2.srv import Ikin, MoveCircle
from robot_arm_interfaces.action import MoveJ, MoveL
from robot_arm_interfaces.msg import PlcCommand, RobotStatus
from robot_arm_interfaces.srv import Home
from rotosy_gripper_control.keyboard_electromagnet_gripper import KeyboardElectromagnetGripper

# ── 공통 파라미터 ────────────────────────────────────────────────────────────

HOSPITAL_WEB_BASE_URL = os.environ.get('HOSPITAL_WEB_BASE_URL', 'http://localhost:8080')
CAMERA_API          = f'{HOSPITAL_WEB_BASE_URL}/camera/markers'
MEDICINE_DETECTION_API = f'{HOSPITAL_WEB_BASE_URL}/camera/detections'
CAMERA_SNAPSHOT_API = f'{HOSPITAL_WEB_BASE_URL}/camera/snapshot_raw'
CAMERA_ACTIVE_DRAWER_API = f'{HOSPITAL_WEB_BASE_URL}/camera/active_drawer'
OCR_VERIFY_API      = f'{HOSPITAL_WEB_BASE_URL}/api/ocr/verify'
PALLET_PLAN_API     = f'{HOSPITAL_WEB_BASE_URL}/api/pallet/plan'
PALLET_PLACED_API   = f'{HOSPITAL_WEB_BASE_URL}/api/pallet/placed'
ROBOT_NS            = 'dsr01'
GOOGLE_CREDENTIALS_PATH = str(
    Path(__file__).resolve().parents[2] / 'application_default_credentials.json'
)
GOOGLE_PROJECT_ID   = 'ocr1-496908'

DEFAULT_MOTION_TOPIC_PREFIX = 'motion'
DEFAULT_INSTANCE_LOCK_PATH = '/tmp/rotosy_motion_sequence.lock'

VEL_MM           = 30.0
ACC_MM           = 60.0
VEL_DEG          = 30.0
ACC_DEG          = 60.0

DEFAULT_RADIUS   = 200.0   # MoveC 기본 반지름 (mm)
DEFAULT_RZ       = 0.0

# E0509 실제 관절 한계 (deg) — dsr_moveit_config_e0509/config/joint_limits.yaml 기준,
# 실기 알람(JNT(2) MAX 95 / MIN -95)과 일치 확인됨. URDF의 ±360°는 실제 한계가 아님.
JOINT_LIMITS_DEG = (
    (-180.0, 180.0),
    (-95.0, 95.0),
    (-135.0, 135.0),
    (-180.0, 180.0),
    (-135.0, 135.0),
    (-180.0, 180.0),
)


def _within_joint_limits(joints: list) -> bool:
    return all(lo <= j <= hi for j, (lo, hi) in zip(joints, JOINT_LIMITS_DEG))


# 카메라 디버그 오버레이 텍스트 줄 (ArUco/YOLO/TOP face 라벨). 정상적으로는 OCR이
# /camera/snapshot_raw(오버레이 없는 원본)를 사용하므로 나타나지 않아야 하지만,
# 폴백 시에도 약품명으로 잘못 채택되지 않도록 안전망으로 남겨둔다. OCR이 숫자/공백
# 순서를 뒤섞는 경우(예: "ID (80,-218,467)mm")까지 포함해 매칭한다.
_ARUCO_LINE_RE = re.compile(
    r'^(?:id\s*[=:]?\s*\d*\s*\(?[-\d,\s]*\)?\s*(?:mm)?'  # ID 1 (x,y,z)mm / id=1 / ID (x,y,z)mm
    r'|top\s*\d+\s*\([-\d,\s]+\)mm'                       # TOP 1 (x,y,z)mm
    r'|\d+(?:\.\d+)?\s*\([-\d,\s]+\)mm)$',                 # YOLO "0.86 (x,y,z)mm"
    re.IGNORECASE,
)


def _parse_medicine_label(raw_text: str) -> dict:
    """Google Cloud Vision OCR 텍스트 → 약품 라벨 구조화 파싱."""
    # ArUco 마커 좌표 오버레이 줄 제거 후 유효 줄만 추출
    lines = [
        l.strip() for l in raw_text.splitlines()
        if l.strip() and not _ARUCO_LINE_RE.match(l.strip())
    ]
    medicine_name = lines[0] if lines else ''

    # 모든 용량 후보를 찾아 mg > ml > 나머지 순으로 우선순위 선택
    dosage_candidates = re.findall(
        r'\d+(?:\.\d+)?\s*(?:mg|ml|mcg|µg|g|IU|정|캡슐|Tab|Cap)',
        raw_text, re.IGNORECASE,
    )
    def _dosage_priority(d: str) -> int:
        d_lower = d.lower()
        if 'mg' in d_lower: return 0
        if 'ml' in d_lower: return 1
        return 2
    dosage = min(dosage_candidates, key=_dosage_priority).strip() if dosage_candidates else ''

    instr_match = re.search(
        r'(?:1일\s*\d+\s*회|하루\s*\d+\s*번|식후|식전|취침\s*전|\d+\s*정씩)',
        raw_text,
    )
    instructions = instr_match.group().strip() if instr_match else ''

    return {
        'medicine_name':     medicine_name,
        'dosage':            dosage,
        'instructions':      instructions,
        'patient_name':      None,
        'prescription_date': None,
        'ward':              None,
        'raw_text':          raw_text,
    }


@dataclass(frozen=True)
class MotionProfile:
    """MoveJ/MoveL speed and acceleration profile.

    MoveJ uses joint velocity/acceleration only. MoveL uses linear and angular
    velocity/acceleration. Keep values conservative until each segment is tested
    on the real robot.
    """

    name: str
    joint_vel_deg_s: float = VEL_DEG
    joint_acc_deg_s2: float = ACC_DEG
    linear_vel_mm_s: float = VEL_MM
    linear_acc_mm_s2: float = ACC_MM
    angular_vel_deg_s: float = VEL_DEG
    angular_acc_deg_s2: float = ACC_DEG


MOTION_PROFILES: dict[str, MotionProfile] = {
    # Current behavior. Existing calls should remain equivalent unless a
    # specific profile is selected at the call site.
    'DEFAULT': MotionProfile('DEFAULT'),

    # Long moves through open space. Tune upward only after verifying clearance.
    'TRANSIT': MotionProfile(
        'TRANSIT',
        joint_vel_deg_s=50.0,
        joint_acc_deg_s2=100.0,
        linear_vel_mm_s=72.0,
        linear_acc_mm_s2=144.0,
        angular_vel_deg_s=50.0,
        angular_acc_deg_s2=100.0,
    ),

    # Approach moves near drawer handles, medicine, or delivery box.
    'APPROACH': MotionProfile(
        'APPROACH',
        joint_vel_deg_s=35.0,
        joint_acc_deg_s2=70.0,
        linear_vel_mm_s=42.0,
        linear_acc_mm_s2=84.0,
        angular_vel_deg_s=35.0,
        angular_acc_deg_s2=70.0,
    ),

    # Physical contact, drawer pull/push, pickup descent, and placement descent.
    'CONTACT': MotionProfile(
        'CONTACT',
        joint_vel_deg_s=22.0,
        joint_acc_deg_s2=44.0,
        linear_vel_mm_s=24.0,
        linear_acc_mm_s2=48.0,
        angular_vel_deg_s=22.0,
        angular_acc_deg_s2=44.0,
    ),

    # Small camera/vision alignment corrections.
    'VISION_ALIGN': MotionProfile(
        'VISION_ALIGN',
        joint_vel_deg_s=31.0,
        joint_acc_deg_s2=62.0,
        linear_vel_mm_s=35.0,
        linear_acc_mm_s2=70.0,
        angular_vel_deg_s=31.0,
        angular_acc_deg_s2=62.0,
    ),

    # Vertical moves while carrying medicine.
    'LIFT': MotionProfile(
        'LIFT',
        joint_vel_deg_s=39.0,
        joint_acc_deg_s2=78.0,
        linear_vel_mm_s=50.0,
        linear_acc_mm_s2=100.0,
        angular_vel_deg_s=39.0,
        angular_acc_deg_s2=78.0,
    ),

    # OCR 촬영을 위해 카메라 앞으로 이동하는 전용 프로파일 (step 13).
    'OCR_APPROACH': MotionProfile(
        'OCR_APPROACH',
        joint_vel_deg_s=50.0,
        joint_acc_deg_s2=100.0,
        linear_vel_mm_s=72.0,
        linear_acc_mm_s2=144.0,
        angular_vel_deg_s=50.0,
        angular_acc_deg_s2=100.0,
    ),

    # OCR 불일치 시 컨베이어로 내려놓는 rollback 블록의 MoveJ 8개 전용.
    # 원래 TRANSIT보다 살짝 더 느리게 (다른 동작들은 전부 조금씩 빨라졌으므로
    # 상대적으로도 가장 느린 동작이 됨).
    'OCR_ROLLBACK': MotionProfile(
        'OCR_ROLLBACK',
        joint_vel_deg_s=40.0,
        joint_acc_deg_s2=79.0,
        linear_vel_mm_s=57.0,
        linear_acc_mm_s2=114.0,
        angular_vel_deg_s=40.0,
        angular_acc_deg_s2=79.0,
    ),
}


BLEND_PROFILES: dict[str, float] = {
    'NONE': 0.0,
    'TINY' : 5.0,
    'SMALL': 10.0,
    'ARC': 20.0,
}

BLEND_ALLOWED_MOTION_PROFILES = {
    'TRANSIT',
    'APPROACH',
    'LIFT',
    'VISION_ALIGN',
    'OCR_APPROACH',
    'OCR_ROLLBACK',
}

DEFAULT_CABINET_GEOMETRY = {
    'marker_id': 1,
    'reference_drawer': 6,
    'coordinate_frame': 'base_link',
    'marker_to_reference_handle_mm': [0.0, -75.0, 205.0],
    'pull_direction_base': [0.0, 1.0, 0.0],
    'column_pitch_mm': 227.0,
    'row_pitch_mm': 112.0,
    'approach_mm': 30.0,
    'release_retreat_mm': 40.0,
    'pull_mm': 200.0,
    'gripper_length_mm': 97.0,
}
# 박스 theta_box 고정값. 측면 마커를 카메라가 79° 각도에서 바라봐 회전행렬 노이즈가 심함.
# None → 회전행렬 기반 계산(+_MARKER_YAW_OFFSET_DEG 보정) 사용.
# 숫자 → 카메라 회전행렬 무시하고 이 값을 theta_box로 고정 사용.
_MARKER_THETA_FIXED: dict[int, float | None] = {
    4: None,    # BOX-A: 별도 캘리브레이션 필요
    3: 82.0,    # BOX-B: 두 슬롯 독립 역산으로 확인 (2026-06-23)
}
_MARKER_YAW_OFFSET_DEG: dict[int, float] = {
    4: 0.0,    # BOX-A: 회전행렬 기반 보정값 (별도 테스트로 결정)
    3: 0.0,    # BOX-B: _MARKER_THETA_FIXED 사용 시 미적용
}
# 카메라 base frame 변환 오차 보정값 (mm).
# 카메라 검출 center에 더해 로봇 base frame 실측값에 맞춘다.
# 실측 방법: 로봇 TCP를 마커 정중앙에 위치시켜 읽은 좌표 − 카메라 검출 좌표
# 카메라 base frame 변환 오차 보정값 (mm).
# Y: 카메라가 항상 +110mm 과대 측정 (extrinsic 오차, 위치 무관하게 일정함 확인)
# X: +20mm 보정 (2026-06-24 실측)
_MARKER_XYZ_CORRECTION_MM: dict[int, tuple[float, float, float]] = {
    4: (0.0, -110.0, 0.0),   # BOX-A
    3: (0.0, -110.0, 0.0),   # BOX-B
}

_INSTANCE_LOCK_FD = None


def _cabinet_geometry_candidates() -> list[Path]:
    candidates = []
    config_dir = os.environ.get('ROTOSY_DOOSAN_CONFIG_DIR')
    if config_dir:
        candidates.append(Path(config_dir).expanduser() / 'cabinet_geometry.yaml')
    candidates.append(Path.cwd() / 'doosan_controller' / 'config' / 'cabinet_geometry.yaml')
    candidates.append(
        Path(__file__).resolve().parent.parent / 'config' / 'cabinet_geometry.yaml'
    )
    try:
        candidates.append(
            Path(get_package_share_directory('doosan_controller'))
            / 'config' / 'cabinet_geometry.yaml'
        )
    except PackageNotFoundError:
        pass
    return candidates


def _load_cabinet_geometry() -> tuple[dict, Path | None]:
    geometry = dict(DEFAULT_CABINET_GEOMETRY)
    for path in _cabinet_geometry_candidates():
        if not path.exists():
            continue
        with path.open(encoding='utf-8') as stream:
            loaded = yaml.safe_load(stream) or {}
        geometry.update(loaded.get('cabinet_geometry', {}))
        return geometry, path
    return geometry, None


def _drawer_calibration_candidates() -> list[Path]:
    candidates = []
    config_dir = os.environ.get('ROTOSY_DOOSAN_CONFIG_DIR')
    if config_dir:
        candidates.append(Path(config_dir).expanduser() / 'drawer_tcp_calibration.yaml')
    candidates.append(Path.cwd() / 'doosan_controller' / 'config' / 'drawer_tcp_calibration.yaml')
    candidates.append(
        Path(__file__).resolve().parent.parent / 'config' / 'drawer_tcp_calibration.yaml'
    )
    try:
        candidates.append(
            Path(get_package_share_directory('doosan_controller'))
            / 'config' / 'drawer_tcp_calibration.yaml'
        )
    except PackageNotFoundError:
        pass
    return candidates


def _load_drawer_corrections() -> tuple[dict[int, tuple[float, float, float]], Path | None]:
    for path in _drawer_calibration_candidates():
        if not path.exists():
            continue
        with path.open(encoding='utf-8') as stream:
            loaded = yaml.safe_load(stream) or {}
        observations = (
            loaded.get('drawer_tcp_calibration', {}).get('observations', {}) or {}
        )
        corrections = {}
        for observation in observations.values():
            if not isinstance(observation, dict):
                continue
            slot_index = observation.get('slot_index')
            correction = observation.get('correction_mm')
            if not isinstance(slot_index, int) or not 0 <= slot_index <= 5:
                continue
            if not isinstance(correction, list) or len(correction) != 3:
                continue
            values = tuple(float(value) for value in correction)
            if not all(math.isfinite(value) for value in values):
                continue
            corrections[slot_index] = values
        return corrections, path
    return {}, None


def _acquire_instance_lock(lock_path: str = DEFAULT_INSTANCE_LOCK_PATH) -> bool:
    """Prevent duplicate sequence nodes from subscribing to command topics."""
    global _INSTANCE_LOCK_FD
    fd = os.open(lock_path, os.O_CREAT | os.O_RDWR, 0o644)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        os.close(fd)
        return False
    os.ftruncate(fd, 0)
    os.write(fd, str(os.getpid()).encode('ascii'))
    _INSTANCE_LOCK_FD = fd
    return True


class MotionSequenceNode(Node):
    """실시간 시퀀스 제어가 가능한 동작 시퀀스 노드."""

    # /<prefix>/start 로 시퀀스를 시작할 때 기본 step 모드.
    # 서브클래스(예: TempSequenceNode)에서 오버라이드해 항상 step 모드로 강제 가능.
    _DEFAULT_STEP_MODE = False

    def __init__(
        self,
        topic_prefix: str = DEFAULT_MOTION_TOPIC_PREFIX,
        node_name: str = 'motion_sequence_node',
    ):
        super().__init__(node_name)
        self._cb_group = ReentrantCallbackGroup()
        self._topic_prefix = topic_prefix.strip('/') or DEFAULT_MOTION_TOPIC_PREFIX

        # 팔레타이징 (PALLETIZING.md) 파라미터 — OCR 일치 시 14~18단계는 항상 DB 연계
        # plan에 따라 병동 박스의 계산된 슬롯에 배치한다(고정좌표 경로 없음).
        self.declare_parameter('box_staging_joints',
                                [31.04, 48.6, 38.63, 0.0, 92.77, -58.96])
        self.declare_parameter('approach_clearance_mm', 150.0)
        # '높이 0인 품목'이 박스 바닥에 닿는 TCP Z(base frame, mm) — 마커 Z 측정은 노이즈가
        # 커서 안 쓰고 실측 calibration 상수로 고정. 실측: 新ビオフェルミンS錠
        # (적재높이 stack_h_mm=55mm) 배치 성공 TCP z=144.21 → 199.21 = 144.21 + 55.0.
        self.declare_parameter('place_floor_z_mm', 199.21)
        self.declare_parameter('carry_rx', 90.0)
        self.declare_parameter('carry_ry', -180.0)
        self.declare_parameter('enable_orientation_correction', True)
        # Clients
        self._home_cli   = self.create_client(Home, '/arm/home', callback_group=self._cb_group)
        self._movel      = ActionClient(self, MoveL, '/arm/move_l', callback_group=self._cb_group)
        self._movej      = ActionClient(self, MoveJ, '/arm/move_j', callback_group=self._cb_group)
        self._cli_circle = self.create_client(
            MoveCircle, f'/{ROBOT_NS}/motion/move_circle', callback_group=self._cb_group
        )
        self._ikin_cli = self.create_client(
            Ikin, f'/{ROBOT_NS}/motion/ikin', callback_group=self._cb_group
        )
        self._gripper = KeyboardElectromagnetGripper(self)

        # Subscribers
        self._status_sub = self.create_subscription(
            RobotStatus, '/arm/status', self._status_cb, 10
        )
        self._next_sub = self.create_subscription(
            Empty, self._topic('next_step'), self._next_step_cb, 10, callback_group=self._cb_group
        )
        self._start_sub = self.create_subscription(
            Int32, self._topic('start'), self._start_cb, 10, callback_group=self._cb_group
        )
        self._stop_sub = self.create_subscription(
            Empty, self._topic('stop'), self._stop_cb, 10, callback_group=self._cb_group
        )
        self._reset_sub = self.create_subscription(
            Empty, self._topic('reset'), self._reset_cb, 10, callback_group=self._cb_group
        )

        # Publishers
        self._step_info_pub = self.create_publisher(String, self._topic('step_info'), 10)
        self._plc_pub = self.create_publisher(PlcCommand, '/plc_command', 10)

        # State
        self._tcp    = None   # [x, y, z, rx, ry, rz] mm / deg
        self._joints = None   # [j1..j6] deg
        self._prev_servo_on = False
        self._last_ocr_medicine_name: str | None = None
        self._box_pose_cache: dict[int, tuple] = {}   # marker_id -> (theta_box_deg, (x,y,z))

        self._step_mode = self._DEFAULT_STEP_MODE
        self._next_step_event = threading.Event()
        self._stop_requested = False
        self._current_sequence_thread = None
        self._is_running = False
        self._active_goal_handle = None
        self._state_lock = threading.Lock()
        self._reset_worker_running = False
        self._cabinet_geometry, geometry_path = _load_cabinet_geometry()
        self._drawer_corrections, calibration_path = _load_drawer_corrections()
        self.get_logger().info(
            f'Cabinet geometry loaded from {geometry_path or "built-in defaults"}'
        )
        self.get_logger().info(
            f'Drawer TCP corrections loaded from '
            f'{calibration_path or "no calibration file"}: '
            f'{len(self._drawer_corrections)} drawer(s)'
        )

    def _topic(self, suffix: str) -> str:
        return f'/{self._topic_prefix}/{suffix}'

    # ── 콜백 ─────────────────────────────────────────────────────────────────

    def _status_cb(self, msg: RobotStatus):
        prev = self._prev_servo_on
        self._prev_servo_on = msg.servo_on
        self._tcp    = list(msg.current_tcp)
        self._joints = list(msg.current_joints_deg)
        if prev and not msg.servo_on:
            self.get_logger().warn('[안전] 서보 OFF 감지 — PLC M20/M21/M23 차단')
            self._plc_safety_off()

    def _next_step_cb(self, msg: Empty):
        self.get_logger().info('Next step signal received')
        self._next_step_event.set()

    def _stop_cb(self, msg: Empty):
        self.get_logger().info('Stop signal received')
        self._request_reset()

    def _reset_cb(self, msg: Empty):
        self.get_logger().info('Reset signal received')
        self._request_reset()

    def _request_reset(self):
        self._stop_requested = True
        self._next_step_event.set()
        self._step_info_pub.publish(String(data='RESETTING'))

        with self._state_lock:
            handle = self._active_goal_handle
            if self._reset_worker_running:
                return
            self._reset_worker_running = True
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warning(f'Active goal cancel failed: {exc}')

        thread = self._current_sequence_thread
        threading.Thread(
            target=self._complete_reset, args=(thread,), daemon=True
        ).start()

    def _complete_reset(self, thread):
        try:
            if not self._set_magnet(False):
                self.get_logger().error('Failed to turn gripper OFF while stopping sequence')

            if thread is not None and thread.is_alive():
                thread.join(timeout=10.0)
                if thread.is_alive():
                    self.get_logger().warning('Sequence is still stopping')
                    self._step_info_pub.publish(String(data='STOPPING'))
                    thread.join()

            self._finish_reset()
        finally:
            with self._state_lock:
                self._reset_worker_running = False

    def _finish_reset(self):
        self._is_running = False
        self._current_sequence_thread = None
        self._next_step_event.clear()
        self._step_info_pub.publish(String(data='IDLE'))

    def _start_cb(self, msg: Int32):
        if self._is_running:
            self.get_logger().warn('Sequence is already running. Stopping previous one...')
            self._stop_requested = True
            self._next_step_event.set()
            if self._current_sequence_thread:
                self._current_sequence_thread.join(timeout=5.0)
                if self._current_sequence_thread.is_alive():
                    self.get_logger().error('Previous sequence is still stopping')
                    self._step_info_pub.publish(String(data='STOPPING'))
                    return

        drawer_number = msg.data   # 사용자 기준 1~6
        if drawer_number < 1 or drawer_number > 6:
            self.get_logger().error(
                f'Invalid drawer number {drawer_number}; expected 1..6'
            )
            self._step_info_pub.publish(String(data='IDLE'))
            return
        drawer_index = drawer_number - 1   # 내부 로직 기준 0~5
        self._step_mode = self._DEFAULT_STEP_MODE
        self._stop_requested = False

        reference_marker_id = int(self._cabinet_geometry['marker_id'])
        self.get_logger().info(
            f'Starting drawer {drawer_index + 1} (index={drawer_index}) using '
            f'fixed ArUco marker ID {reference_marker_id} '
            f'(Step Mode: {self._step_mode})'
        )
        self._current_sequence_thread = threading.Thread(
            target=self.run_sequence, args=(drawer_index, DEFAULT_RADIUS)
        )
        self._current_sequence_thread.start()

    # ── 유틸 ─────────────────────────────────────────────────────────────────

    def _wait_for_step(self, step_name: str):
        """Step 모드일 경우 신호를 기다림."""
        if self._stop_requested:
            return False

        self._next_step_event.clear()

        # WAIT: 접두사 → 웹에서 다음 단계 버튼 활성화
        info_msg = String()
        info_msg.data = f'WAIT:{step_name}'
        self._step_info_pub.publish(info_msg)

        if not self._step_mode:
            self.get_logger().info(f'Step: {step_name}')
            info_msg.data = f'RUN:{step_name}'
            self._step_info_pub.publish(info_msg)
            return True

        self.get_logger().info(f'Waiting for Next Step signal to: {step_name}...')
        self._next_step_event.wait()

        if self._stop_requested:
            self.get_logger().info('Step wait cancelled by stop request')
            return False

        self.get_logger().info(f'Proceeding to: {step_name}')
        # RUN: 접두사 → 웹에서 다음 단계 버튼 비활성화 (실행 중)
        info_msg.data = f'RUN:{step_name}'
        self._step_info_pub.publish(info_msg)
        return True

    def _wait_tcp(self, timeout: float = 5.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._tcp and any(v != 0.0 for v in self._tcp[:3]):
                return True
            time.sleep(0.1)
        return False

    def _wait_joints(self, timeout: float = 5.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._joints and len(self._joints) >= 6:
                return True
            time.sleep(0.1)
        return False

    # ── 동작 헬퍼 ────────────────────────────────────────────────────────────

    def _motion_profile(self, profile: str | MotionProfile | None) -> MotionProfile:
        if isinstance(profile, MotionProfile):
            return profile

        key = (profile or 'DEFAULT').upper()
        selected = MOTION_PROFILES.get(key)
        if selected is None:
            self.get_logger().warning(
                f'Unknown motion profile "{profile}", using DEFAULT'
            )
            return MOTION_PROFILES['DEFAULT']
        return selected

    def _blend_radius(
        self,
        blend: str | float | int | None,
        motion_profile: MotionProfile,
        *,
        label: str,
    ) -> float:
        if blend is None:
            radius = 0.0
            blend_name = 'NONE'
        elif isinstance(blend, str):
            blend_name = blend.upper()
            if blend_name not in BLEND_PROFILES:
                self.get_logger().warning(
                    f'Unknown blend profile "{blend}" for {label}, using NONE'
                )
                blend_name = 'NONE'
            radius = BLEND_PROFILES[blend_name]
        else:
            radius = float(blend)
            blend_name = f'{radius:.1f}'

        if radius <= 0.0:
            return 0.0

        if motion_profile.name not in BLEND_ALLOWED_MOTION_PROFILES:
            self.get_logger().warning(
                f'Blend {blend_name} requested for {label} with '
                f'profile={motion_profile.name}; disabled by safety policy'
            )
            return 0.0

        return radius

    def _home(self) -> bool:
        self.get_logger().info('[홈] 이동 중...')
        if not self._home_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Home 서비스 없음')
            return False
        req = Home.Request()
        req.target = 0
        fut = self._home_cli.call_async(req)
        while rclpy.ok() and not fut.done():
            time.sleep(0.1)

        resp = fut.result()
        if not resp or not resp.success:
            self.get_logger().error(f'홈 실패: {getattr(resp, "message", "?")}')
            return False
        self.get_logger().info('홈 완료')
        time.sleep(0.5)
        return True

    def _move_l(self, x, y, z, rx=0.0, ry=0.0, rz=0.0, relative: bool = False,
                blend_radius: str | float | int | None = None,
                vel_mm: float | None = None, vel_deg: float | None = None,
                acc_mm: float | None = None, acc_deg: float | None = None,
                profile: str | MotionProfile | None = None) -> bool:
        motion_profile = self._motion_profile(profile)
        blend = self._blend_radius(blend_radius, motion_profile, label='MoveL')
        linear_vel = motion_profile.linear_vel_mm_s if vel_mm is None else float(vel_mm)
        angular_vel = motion_profile.angular_vel_deg_s if vel_deg is None else float(vel_deg)
        linear_acc = motion_profile.linear_acc_mm_s2 if acc_mm is None else float(acc_mm)
        angular_acc = motion_profile.angular_acc_deg_s2 if acc_deg is None else float(acc_deg)
        mode = '상대' if relative else '절대'
        self.get_logger().info(
            f'MoveL({mode}, profile={motion_profile.name}) → '
            f'({x:.1f}, {y:.1f}, {z:.1f})  rx={rx} ry={ry} rz={rz} '
            f'vel=({linear_vel:.1f}mm/s,{angular_vel:.1f}deg/s) '
            f'acc=({linear_acc:.1f}mm/s²,{angular_acc:.1f}deg/s²)'
            + (f' blend={blend:.1f}mm' if blend > 0.0 else '')
        )
        self._movel.wait_for_server()
        goal = MoveL.Goal()
        goal.x, goal.y, goal.z     = float(x), float(y), float(z)
        goal.rx, goal.ry, goal.rz  = float(rx), float(ry), float(rz)
        goal.linear_velocity_mm_s   = linear_vel
        goal.angular_velocity_deg_s = angular_vel
        goal.linear_accel_mm_s2     = linear_acc
        goal.angular_accel_deg_s2   = angular_acc
        goal.blend_radius_mm        = blend
        goal.reference_frame        = 0
        goal.relative               = relative

        fut = self._movel.send_goal_async(goal)
        while rclpy.ok() and not fut.done():
            time.sleep(0.05)
        handle = fut.result()
        if not handle.accepted:
            self.get_logger().error('MoveL 거부 — Servo ON 확인')
            return False

        with self._state_lock:
            self._active_goal_handle = handle

        res_fut = handle.get_result_async()
        while rclpy.ok() and not res_fut.done():
            if self._stop_requested:
                handle.cancel_goal_async()
            time.sleep(0.1)

        with self._state_lock:
            self._active_goal_handle = None
        wrapped_result = res_fut.result()
        if wrapped_result is None:
            return False
        result = wrapped_result.result
        if result.success:
            self.get_logger().info(f'MoveL 완료 ({result.execution_time_sec:.2f}s)')
            return True
        self.get_logger().error(f'MoveL 실패: {result.message}')
        return False

    def _move_j(
        self,
        joint_angles: list,
        blend_radius: str | float | int | None = None,
        profile: str | MotionProfile | None = None,
    ) -> bool:
        motion_profile = self._motion_profile(profile)
        blend = self._blend_radius(blend_radius, motion_profile, label='MoveJ')
        self.get_logger().info(
            f'MoveJ(profile={motion_profile.name}) → {[round(a, 1) for a in joint_angles]} '
            f'vel={motion_profile.joint_vel_deg_s:.1f}deg/s '
            f'acc={motion_profile.joint_acc_deg_s2:.1f}deg/s²'
            + (f' blend={blend:.1f}mm' if blend > 0.0 else '')
        )
        self._movej.wait_for_server()
        goal = MoveJ.Goal()
        goal.joint_angles_deg    = [float(a) for a in joint_angles]
        goal.velocity_deg_s      = motion_profile.joint_vel_deg_s
        goal.acceleration_deg_s2 = motion_profile.joint_acc_deg_s2
        goal.blend_radius_mm     = blend

        fut = self._movej.send_goal_async(goal)
        while rclpy.ok() and not fut.done():
            time.sleep(0.05)
        handle = fut.result()
        if not handle.accepted:
            self.get_logger().error('MoveJ 거부')
            return False

        with self._state_lock:
            self._active_goal_handle = handle

        res_fut = handle.get_result_async()
        while rclpy.ok() and not res_fut.done():
            if self._stop_requested:
                handle.cancel_goal_async()
            time.sleep(0.1)

        with self._state_lock:
            self._active_goal_handle = None
        wrapped_result = res_fut.result()
        if wrapped_result is None:
            return False
        result = wrapped_result.result
        if result.success:
            self.get_logger().info(f'MoveJ 완료 ({result.execution_time_sec:.2f}s)')
            return True
        self.get_logger().error(f'MoveJ 실패: {result.message}')
        return False

    def _dsr_ik(self, pose6: list) -> tuple:
        """DSR ikin을 8개 solution space에 대해 호출, 현재 관절과 가장 가까운 해 반환.

        Returns:
            (joint_angles_deg, '') 성공 시
            (None, reason_str)    실패 시
        """
        if not self._ikin_cli.wait_for_service(timeout_sec=2.0):
            return None, 'ikin 서비스 없음'

        best_joints = None
        best_dist   = float('inf')

        for sol in range(8):
            req           = Ikin.Request()
            req.pos       = [float(v) for v in pose6]
            req.sol_space = sol
            req.ref       = 0   # DR_BASE

            fut = self._ikin_cli.call_async(req)
            while rclpy.ok() and not fut.done():
                time.sleep(0.02)
            resp = fut.result()
            if resp is None or not resp.success:
                continue

            joints = list(resp.conv_posj)
            if not _within_joint_limits(joints):
                continue
            dist = sum((a - b) ** 2 for a, b in zip(joints, self._joints))
            if dist < best_dist:
                best_dist   = dist
                best_joints = joints

        if best_joints is None:
            return None, '8개 solution space 모두 실패 또는 관절 한계 초과'
        return best_joints, ''

    def _move_l_hybrid(self, x, y, z, rx=0.0, ry=0.0, rz=0.0,
                        blend_radius: str | float | int | None = None,
                        vel_mm: float | None = None, vel_deg: float | None = None,
                        acc_mm: float | None = None, acc_deg: float | None = None,
                        profile: str | MotionProfile | None = None) -> bool:
        """절대 좌표 MoveL을 IK 기반 MoveJ + 잔차 보정 MoveL로 대체.

        원거리/큰 자세 변화 이동에서 MoveL 직선보간 대신 DSR ikin으로 목표
        joint를 구해 MoveJ로 먼저 이동시키고, 남는 오차만 MoveL로 보정한다
        (test_movel.py와 동일한 방식). IK가 실패하면 기존처럼 MoveL 단독
        이동으로 대체한다. relative 이동에는 적용하지 않는다(IK는 절대좌표만).
        """
        fallback = lambda: self._move_l(
            x, y, z, rx, ry, rz,
            blend_radius=blend_radius, vel_mm=vel_mm, vel_deg=vel_deg,
            acc_mm=acc_mm, acc_deg=acc_deg, profile=profile,
        )

        if not self._joints:
            self.get_logger().warning('Hybrid MoveL: 관절 위치 미수신 — MoveL 단독 이동으로 대체')
            return fallback()

        ik_joints, err_msg = self._dsr_ik([x, y, z, rx, ry, rz])
        if ik_joints is None:
            self.get_logger().warning(
                f'Hybrid MoveL: DSR IK 실패({err_msg}) — MoveL 단독 이동으로 대체'
            )
            return fallback()

        self.get_logger().info(f'Hybrid MoveL: IK → {[round(j, 1) for j in ik_joints]}')
        if not self._move_j(ik_joints, profile=profile):
            return False
        time.sleep(0.2)

        return fallback()

    def _set_magnet(self, enabled: bool) -> bool:
        """전자석 ON(enabled=True) / OFF(enabled=False)."""
        return self._gripper.set_gripper(enabled)

    def _plc_safety_off(self):
        """서보 OFF 또는 노드 종료 시 PLC 안전 코일 일괄 차단."""
        self.get_logger().warning('[PLC] 안전 차단 — M20/M21/M23 OFF')
        for addr in (0x20, 0x21, 0x23):
            self._set_plc_coil(addr, False)

    def _publish_plc(self, msg: PlcCommand, label: str) -> None:
        """PlcCommand를 발행하고, 구독자 수를 포함한 진단 로그를 출력한다."""
        sub_count = self._plc_pub.get_subscription_count()
        self._plc_pub.publish(msg)
        self.get_logger().info(f'[PLC→발행] {label} [구독자: {sub_count}]')
        if sub_count == 0:
            self.get_logger().warning(
                '[PLC] ★ /plc_command 구독자 없음 — '
                'plc_controller_node가 실행 중인지 확인하세요 ★'
            )

    def _set_plc_coil(self, address: int, value: bool, slave_id: int = 1, target: str = 'PLC'):
        """PLC 코일(M 릴레이) ON/OFF. 주소는 hex 기준 (M20 → 0x20)."""
        msg = PlcCommand()
        msg.target   = target
        msg.command  = 'COIL'
        msg.address  = address
        msg.value    = int(value)
        msg.slave_id = slave_id
        self._publish_plc(
            msg,
            f'{target} coil 0x{address:02X} → {"ON" if value else "OFF"} (slave={slave_id})',
        )

    def _set_inverter_freq(self, freq: int):
        """인버터 목표 주파수 설정. freq 단위: 0.01 Hz (3000 = 30.00 Hz)."""
        msg = PlcCommand()
        msg.target   = 'INVERTER'
        msg.command  = 'REGISTER'
        msg.address  = 4
        msg.value    = freq
        msg.slave_id = 2
        self._publish_plc(msg, f'인버터 주파수 → {freq / 100:.2f} Hz (reg=4 slave=2)')

    def _set_inverter_run(self, run: bool):
        """인버터 RUN(run=True) / STOP(run=False) + PLC M21 연동."""
        msg = PlcCommand()
        msg.target   = 'INVERTER'
        msg.command  = 'REGISTER'
        msg.address  = 5
        msg.value    = 2 if run else 1
        msg.slave_id = 2
        self._publish_plc(
            msg,
            f'인버터 {"RUN" if run else "STOP"} (reg=5 val={2 if run else 1} slave=2)',
        )

        # M21 연동 (인버터 운전 상태 표시 릴레이)
        time.sleep(0.1)
        self._set_plc_coil(0x21, run, target='PLC')   # M21 ON/OFF

    def _move_circle(self, via: list, end: list) -> bool:
        if not self._cli_circle.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('move_circle 서비스 없음')
            return False
        via_msg      = Float64MultiArray(data=[float(v) for v in via])
        end_msg      = Float64MultiArray(data=[float(v) for v in end])
        req          = MoveCircle.Request()
        req.pos      = [via_msg, end_msg]
        req.vel      = [VEL_MM, VEL_DEG]
        req.acc      = [ACC_MM, ACC_DEG]
        req.time     = 0.0
        req.radius   = 0.0
        req.ref      = 0
        req.mode     = 0   # MOVE_MODE_ABSOLUTE: via/end가 절대 좌표
        req.angle1   = 0.0
        req.angle2   = 0.0
        req.blend_type = 0
        req.sync_type  = 0

        fut = self._cli_circle.call_async(req)
        while rclpy.ok() and not fut.done():
            time.sleep(0.1)
        resp = fut.result()
        if resp is None:
            self.get_logger().error('MoveC 타임아웃')
            return False
        return bool(resp.success)

    def _move_c(self, radius: float, direction: int) -> tuple | None:
        """현재 TCP 기준 90° 호 이동 (MoveL N분할).

        direction=-1:
          - 구 중심 C = (x0, y0-r, z0)  ← Y축 기준 -r 위치
          - 시작(x0,y0,z0) → 끝(x0, y0-r, z0+r)  [X 고정, Y 감소, Z 증가]
          - 자세: rx=90 고정, rz=0 고정, ry: -90 → -180 (구 중심을 바라보며 회전)
        """
        if not self._tcp:
            self.get_logger().error('MoveC: TCP 위치 미수신')
            return None

        x0, y0, z0 = self._tcp[0], self._tcp[1], self._tcp[2]
        r = float(radius)
        N = 6       # 분할 수 (15° 간격)

        if direction < 0:
            Cy, Cz = y0 - r, z0
        else:
            self.get_logger().error('direction >= 0 호 이동 미구현')
            return None

        self.get_logger().info(
            f'호 이동(MoveL×{N})  시작({x0:.1f},{y0:.1f},{z0:.1f})'
            f'  center({x0:.1f},{Cy:.1f},{Cz:.1f})  r={r:.0f}mm'
        )

        for i in range(1, N + 1):
            theta_deg = i * 90.0 / N
            theta_rad = math.radians(theta_deg)

            px = x0
            py = Cy + r * math.cos(theta_rad)
            pz = Cz + r * math.sin(theta_rad)

            r_x = 90.0
            r_y = -90.0 - theta_deg
            r_z = 0.0

            blend = 'SMALL' if i == N else 'ARC'

            self.get_logger().info(
                f'  [{i}/{N}] θ={theta_deg:.0f}°'
                f'  pos=({px:.1f},{py:.1f},{pz:.1f})'
                f'  ori=({r_x:.0f},{r_y:.0f},{r_z:.0f})'
            )

            if not self._move_l(
                px, py, pz, r_x, r_y, r_z,
                blend_radius=blend,
                profile='VISION_ALIGN',
            ):
                self.get_logger().error(f'호 이동 실패 θ={theta_deg:.0f}°')
                return None

        end_pos = (x0, Cy, Cz + r)
        self.get_logger().info(f'호 이동 완료  end=({end_pos[0]:.1f},{end_pos[1]:.1f},{end_pos[2]:.1f})')
        return end_pos

    def _move_c_reverse_from_end(self, start_pose: tuple, radius: float) -> bool:
        """Step 8 MoveC(-1)의 분할 경로를 끝점에서 시작점으로 역재생."""
        if len(start_pose) < 3:
            self.get_logger().error('역방향 MoveC: 시작 pose 정보 부족')
            return False

        x0, y0, z0 = float(start_pose[0]), float(start_pose[1]), float(start_pose[2])
        r = float(radius)
        N = 6
        Cy, Cz = y0 - r, z0

        self.get_logger().info(
            f'역방향 호 이동(MoveL×{N})  start=({x0:.1f},{Cy:.1f},{Cz + r:.1f})'
            f'  end=({x0:.1f},{y0:.1f},{z0:.1f})  r={r:.0f}mm'
        )

        for idx, i in enumerate(range(N - 1, -1, -1), start=1):
            theta_deg = i * 90.0 / N
            theta_rad = math.radians(theta_deg)

            px = x0
            py = Cy + r * math.cos(theta_rad)
            pz = Cz + r * math.sin(theta_rad)

            r_x = 90.0
            r_y = -90.0 - theta_deg
            r_z = 0.0

            blend = 'NONE' if i == 0 else 'ARC'
            self.get_logger().info(
                f'  [rev {idx}/{N}] θ={theta_deg:.0f}°'
                f'  pos=({px:.1f},{py:.1f},{pz:.1f})'
                f'  ori=({r_x:.0f},{r_y:.0f},{r_z:.0f})'
            )

            if not self._move_l(
                px, py, pz, r_x, r_y, r_z,
                blend_radius=blend,
                profile='VISION_ALIGN',
            ):
                self.get_logger().error(f'역방향 호 이동 실패 θ={theta_deg:.0f}°')
                return False

        return True

    def _move_c_retreat(self, radius_mm: float) -> bool:
        """현재 TCP를 시작점으로 진짜 원호를 그리며 후퇴 (test_moveC.py direction=2와 동일 동작).

        원 중심 C = (x0, y0+r, z0). P(θ) = (x0, y0 + r·(1-cosθ), z0 + r·sinθ),
        θ: 0°→90° → Y, Z 모두 단조 증가. 자세는 Rx=90, Rz=0 고정, Ry만 현재값에서
        +90°(예: -180 → -90, 코드베이스 전체에서 쓰는 '손잡이' 표준 포즈로 복귀).
        """
        if not self._tcp:
            self.get_logger().error('MoveC 후퇴: TCP 위치 미수신')
            return False

        x0, y0, z0, _rx0, ry0, _rz0 = self._tcp[:6]
        r = float(radius_mm)
        N = 6

        self.get_logger().info(
            f'호 이동(후퇴, MoveL×{N})  시작({x0:.1f},{y0:.1f},{z0:.1f})'
            f'  center({x0:.1f},{y0 + r:.1f},{z0:.1f})  r={r:.0f}mm'
        )

        for i in range(1, N + 1):
            theta_deg = i * 90.0 / N
            theta_rad = math.radians(theta_deg)

            px = x0
            py = y0 + r * (1.0 - math.cos(theta_rad))
            pz = z0 + r * math.sin(theta_rad)

            r_y_raw = ry0 + theta_deg
            r_y = r_y_raw - 360.0 if r_y_raw > 180.0 else r_y_raw

            blend = 'SMALL' if i == N else 'ARC'

            self.get_logger().info(
                f'  [{i}/{N}] θ={theta_deg:.0f}°'
                f'  pos=({px:.1f},{py:.1f},{pz:.1f})'
                f'  ori=(90,{r_y:.0f},0)'
            )

            if not self._move_l(
                px, py, pz, 90.0, r_y, 0.0,
                blend_radius=blend,
                profile='LIFT',
            ):
                self.get_logger().error(f'후퇴 호 이동 실패 θ={theta_deg:.0f}°')
                return False

        return True

    # ── OCR 파이프라인 ────────────────────────────────────────────────────────

    def _capture_image(self) -> bytes | None:
        """hospital_web에서 현재 카메라 프레임을 JPEG bytes로 취득."""
        try:
            with urllib.request.urlopen(CAMERA_SNAPSHOT_API, timeout=5) as resp:
                return resp.read()
        except Exception as e:
            self.get_logger().error(f'[OCR] 스냅샷 취득 실패: {e}')
            return None

    def _ocr_and_parse(self) -> dict | None:
        """카메라 스냅샷 → Google Cloud Vision OCR → 구조화 파싱 → ROS 토픽 발행.

        인증: GOOGLE_APPLICATION_CREDENTIALS 미설정 시 GOOGLE_CREDENTIALS_PATH를 자동 사용.
        """
        self.get_logger().info('[OCR] 카메라 스냅샷 취득...')
        img = self._capture_image()
        if img is None:
            return None

        self.get_logger().info('[OCR] Google Cloud Vision OCR 분석 중...')
        try:
            if not os.environ.get('GOOGLE_APPLICATION_CREDENTIALS'):
                os.environ['GOOGLE_APPLICATION_CREDENTIALS'] = GOOGLE_CREDENTIALS_PATH
            import google.auth
            from google.cloud import vision as _vision
            credentials, _ = google.auth.default(
                scopes=['https://www.googleapis.com/auth/cloud-platform'],
                quota_project_id=GOOGLE_PROJECT_ID,
            )
            client = _vision.ImageAnnotatorClient(credentials=credentials)
            image = _vision.Image(content=img)
            response = client.document_text_detection(
                image=image,
                image_context={'language_hints': ['ko', 'ja', 'en']},
            )
            if response.error.message:
                raise RuntimeError(response.error.message)
            raw_text = response.full_text_annotation.text.strip()
        except Exception as e:
            self.get_logger().error(f'[OCR] Google Cloud Vision 호출 실패: {e}')
            return None

        result = _parse_medicine_label(raw_text)

        self.get_logger().info(f'[OCR] 파싱 결과: {result}')
        self._last_ocr_medicine_name = result.get('medicine_name') or None
        self._step_info_pub.publish(
            String(data=f'OCR:{json.dumps(result, ensure_ascii=False)}')
        )

        # hospital_web에 인증 요청
        try:
            payload = json.dumps(result, ensure_ascii=False).encode()
            req = urllib.request.Request(
                OCR_VERIFY_API,
                data=payload,
                headers={'Content-Type': 'application/json'},
                method='POST',
            )
            with urllib.request.urlopen(req, timeout=5) as resp:
                verify = json.loads(resp.read())
            status = verify.get('match_status', 'UNKNOWN')
            pending = verify.get('pending_count', '?')
            self.get_logger().info(
                f'[OCR] 인증 결과: {status}  남은 품목: {pending}개'
            )
            if status == 'MISMATCH':
                self.get_logger().warn(
                    f'[OCR] 불일치: {verify.get("mismatch_reason", "")}'
                )
            return status # status 반환 추가
        except Exception as e:
            self.get_logger().warn(f'[OCR] hospital_web 전송 실패 (계속 진행): {e}')
            return 'UNKNOWN'

    def _get_marker(self, marker_id: int, retries: int = 5):
        for attempt in range(retries):
            try:
                with urllib.request.urlopen(CAMERA_API, timeout=3) as resp:
                    data = json.loads(resp.read())
                for m in data.get('markers', []):
                    if (
                        m['id'] == marker_id
                        and m.get('x_mm') is not None
                        and m.get('rotation_matrix_base') is not None
                    ):
                        return m
            except Exception as e:
                self.get_logger().warning(f'마커 취득 시도 {attempt+1}: {e}')
            time.sleep(0.5)
        return None

    def _get_medicine_target(self, retries: int = 5) -> tuple | None:
        """hospital_web에서 약품 베이스 좌표를 취득.

        보정된 top-face 중심이 있으면 우선 사용하고, 없으면 기존 YOLO bbox 중심
        좌표로 폴백한다. 여러 약품이 검출되면 표준 ROS camera optical frame의
        +X(화면 우측) 방향에
        가장 있는 약품을 선택한다.
        """
        for attempt in range(retries):
            try:
                with urllib.request.urlopen(MEDICINE_DETECTION_API, timeout=3) as resp:
                    data = json.loads(resp.read())

                # 'medicine', 'medicine_box' 또는 'water_pack' 클래스 중 유효한 좌표가 있는 대상을 모두 선택
                targets = [
                    d for d in data.get('detections', [])
                    if d['class_name'].lower() in ['medicine', 'medicine_box', 'water_pack']
                    and (
                        d.get('base_position_m') is not None
                        or (d.get('top_face') or {}).get('base_position_m') is not None
                    )
                ]

                if targets:
                    # base_position_m의 Y 좌표가 가장 큰 약품을 먼저 집는다.
                    # Y 좌표가 없는 검출은 신뢰도로만 보조 선택한다.
                    def target_priority(detection):
                        top_face = detection.get('top_face') or {}
                        base_pos = top_face.get('base_position_m') or detection.get('base_position_m')
                        try:
                            base_y = float(base_pos[1])
                        except (TypeError, ValueError, IndexError):
                            return (0, float('-inf'), detection.get('confidence', 0.0))
                        if not math.isfinite(base_y):
                            return (0, float('-inf'), detection.get('confidence', 0.0))
                        return (1, base_y, detection.get('confidence', 0.0))

                    targets.sort(key=target_priority, reverse=True)
                    target = targets[0]
                    top_face = target.get('top_face') or {}
                    best = top_face.get('base_position_m') or target['base_position_m']
                    source = 'top_face' if top_face.get('base_position_m') else 'bbox_center'
                    priority = target_priority(target)
                    base_y = priority[1] if priority[0] else None
                    self.get_logger().info(
                        f"Target found: {target['class_name']} "
                        f"(base_y: {base_y}, conf: {target['confidence']:.2f}, "
                        f"source={source})"
                    )
                    # m -> mm 변환
                    return best[0] * 1000.0, best[1] * 1000.0, best[2] * 1000.0
            except Exception as e:
                self.get_logger().warning(f'목표 좌표 취득 시도 {attempt+1}: {e}')
            time.sleep(0.5)
        return None

    def _set_camera_active_drawer(self, drawer_number: int) -> None:
        """Tell hospital_web which drawer's medicine size prior should be used."""
        try:
            req = urllib.request.Request(
                f'{CAMERA_ACTIVE_DRAWER_API}/{drawer_number}',
                data=b'',
                method='POST',
            )
            with urllib.request.urlopen(req, timeout=2) as resp:
                json.loads(resp.read())
            self.get_logger().info(f'[Vision] active drawer context set: {drawer_number}')
        except Exception as exc:
            self.get_logger().warning(
                f'[Vision] active drawer context update failed: {exc}'
            )

    def _clear_camera_active_drawer(self) -> None:
        try:
            req = urllib.request.Request(
                CAMERA_ACTIVE_DRAWER_API,
                data=b'',
                method='DELETE',
            )
            with urllib.request.urlopen(req, timeout=2) as resp:
                json.loads(resp.read())
            self.get_logger().info('[Vision] active drawer context cleared')
        except Exception as exc:
            self.get_logger().warning(
                f'[Vision] active drawer context clear failed: {exc}'
            )

    def _on_missing_medicine(
        self,
        contact: tuple[float, float, float],
        pull_dir: tuple[float, float, float],
    ) -> tuple[float, float, float] | None:
        """Hook for subclasses that want to continue without a detected box."""
        return None

    # ── 팔레타이징 (PALLETIZING.md) ──────────────────────────────────────────

    def _get_box_pose(self, marker_id: int) -> tuple | None:
        """배송 박스 ArUco 마커에서 (theta_box_deg, center_xyz) 취득.

        매번 신선 탐색을 시도한다.
        - 탐색 성공: 측정값으로 캐시를 갱신하고 반환.
        - 탐색 실패: 이전 적재 시 캐시된 값이 있으면 그 값을 재사용.
        - 캐시도 없음: None 반환 → 호출 측이 DB origin fallback 처리.
        """
        marker = self._get_marker(marker_id)
        if marker is not None:
            R = marker['rotation_matrix_base']
            theta_detected = math.degrees(math.atan2(R[1][0], R[0][0]))
            fixed_theta = _MARKER_THETA_FIXED.get(marker_id)
            if fixed_theta is not None:
                theta_box = fixed_theta
            else:
                yaw_offset = _MARKER_YAW_OFFSET_DEG.get(marker_id, 0.0)
                theta_box = theta_detected + yaw_offset
            raw = (float(marker['x_mm']), float(marker['y_mm']), float(marker['z_mm']))
            dx, dy, dz = _MARKER_XYZ_CORRECTION_MM.get(marker_id, (0.0, 0.0, 0.0))
            center = (raw[0] + dx, raw[1] + dy, raw[2] + dz)
            self._box_pose_cache[marker_id] = (theta_box, center)
            self.get_logger().info(
                f'[팔레타이징] 박스 마커 {marker_id} 측정·캐시 갱신: '
                f'theta_detected={theta_detected:.1f}° '
                f'theta_box={theta_box:.1f}°{"(고정)" if fixed_theta is not None else ""} '
                f'raw=({raw[0]:.1f},{raw[1]:.1f},{raw[2]:.1f}) '
                f'correction=({dx:+.1f},{dy:+.1f},{dz:+.1f}) '
                f'center=({center[0]:.1f},{center[1]:.1f},{center[2]:.1f})'
            )
            return self._box_pose_cache[marker_id]

        if marker_id in self._box_pose_cache:
            self.get_logger().warning(
                f'[팔레타이징] 박스 마커 {marker_id} 미검출 — 이전 측정값 재사용'
            )
            return self._box_pose_cache[marker_id]

        return None

    def _get_grasped_yaw(self) -> float:
        """그리퍼로 잡은 약품의 yaw 오차(theta_item).

        TODO: vision OBB(obb_angle_deg)가 아직 web_interface에 구현되어 있지 않아
        현재는 0.0(보정 없음)을 반환한다. theta_box(박스 회전) 보정은 정상 적용된다.
        """
        return 0.0

    def _fetch_pallet_plan(self) -> dict | None:
        """hospital_web에 저장된 현재 미션의 적재 레이아웃 + 박스 메타 조회."""
        try:
            with urllib.request.urlopen(PALLET_PLAN_API, timeout=5) as resp:
                plan = json.loads(resp.read())
        except Exception as e:
            self.get_logger().error(f'[팔레타이징] plan 조회 실패: {e}')
            return None
        if not plan.get('ok'):
            self.get_logger().error(f"[팔레타이징] plan 없음: {plan.get('error')}")
            return None
        return plan

    def _mark_pallet_placed(self, slot_idx: int) -> bool:
        """슬롯 배치 완료를 hospital_web(pallet_plan)에 보고."""
        try:
            payload = json.dumps({'slot_idx': slot_idx}).encode()
            req = urllib.request.Request(
                PALLET_PLACED_API, data=payload,
                headers={'Content-Type': 'application/json'}, method='POST',
            )
            with urllib.request.urlopen(req, timeout=5) as resp:
                result = json.loads(resp.read())
            if not result.get('ok'):
                self.get_logger().warning(f"[팔레타이징] 배치 완료 보고 실패: {result.get('error')}")
                return False
            return True
        except Exception as e:
            self.get_logger().warning(f'[팔레타이징] 배치 완료 보고 실패: {e}')
            return False

    # ── 시퀀스 ───────────────────────────────────────────────────────────────

    @staticmethod
    def _mat_vec(R: list, v: tuple) -> tuple:
        return tuple(sum(float(R[row][col]) * v[col] for col in range(3)) for row in range(3))

    def _drawer_offset(self, slot_index: int) -> tuple | None:
        """Return handle-center offset from the marker in robot base axes."""
        if slot_index < 0 or slot_index > 5:
            self.get_logger().error(f'서랍 번호 범위 초과: {slot_index + 1} (1~6 필요)')
            return None

        row, col = divmod(slot_index, 2)
        reference_slot = int(self._cabinet_geometry['reference_drawer']) - 1
        ref_row, ref_col = divmod(reference_slot, 2)
        ref_x, ref_y, ref_z = self._cabinet_geometry['marker_to_reference_handle_mm']
        x = float(ref_x) + (ref_col - col) * float(self._cabinet_geometry['column_pitch_mm'])
        z = float(ref_z) + (ref_row - row) * float(self._cabinet_geometry['row_pitch_mm'])
        return x, float(ref_y), z

    def _get_drawer_target(self, slot_index: int) -> tuple | None:
        """Compute handle and flange targets from the top-view cabinet marker."""
        offset = self._drawer_offset(slot_index)
        if offset is None:
            return None

        self.get_logger().info(
            f'[서랍 {slot_index + 1}] 전면 기준 마커 좌표 취득 중...'
        )
        marker_id = int(self._cabinet_geometry['marker_id'])
        marker = self._get_marker(marker_id)
        if marker is None:
            self.get_logger().error(
                f'ID {marker_id} 마커 위치/회전 미수신'
            )
            return None

        marker_pos = (float(marker['x_mm']), float(marker['y_mm']), float(marker['z_mm']))
        handle = tuple(marker_pos[i] + offset[i] for i in range(3))
        drawer_correction = self._drawer_corrections.get(slot_index, (0.0, 0.0, 0.0))
        handle = tuple(handle[i] + drawer_correction[i] for i in range(3))

        pull_dir = tuple(float(v) for v in self._cabinet_geometry['pull_direction_base'])
        gripper_length = float(self._cabinet_geometry['gripper_length_mm'])
        approach_distance = float(self._cabinet_geometry['approach_mm'])
        contact_flange = tuple(handle[i] + pull_dir[i] * gripper_length for i in range(3))
        approach_flange = tuple(
            contact_flange[i] + pull_dir[i] * approach_distance for i in range(3)
        )

        self.get_logger().info(
            f'마커=({marker_pos[0]:.1f},{marker_pos[1]:.1f},{marker_pos[2]:.1f}) '
            f'베이스축 오프셋=({offset[0]:.1f},{offset[1]:.1f},{offset[2]:.1f}) '
            f'서랍별 보정=({drawer_correction[0]:+.2f},'
            f'{drawer_correction[1]:+.2f},{drawer_correction[2]:+.2f}) '
            f'손잡이=({handle[0]:.1f},{handle[1]:.1f},{handle[2]:.1f}) '
            f'접근 플랜지=({approach_flange[0]:.1f},{approach_flange[1]:.1f},{approach_flange[2]:.1f})'
        )
        return approach_flange, contact_flange, pull_dir

    def _pick_verify_and_deliver(
        self,
        drawer_index: int,
        radius_mm: float,
        contact: tuple,
        pull_dir: tuple,
        pull_target: tuple,
    ) -> bool:
        """8~18단계: 약품을 집어 OCR로 검수하고 배송 박스(또는 컨베이어 롤백)로 옮긴다.

        이 시점에는 이미 서랍이 열려 있다. False를 반환해도 서랍은 닫아야 하므로,
        호출 측(run_sequence)은 이 결과와 무관하게 항상 _close_drawer_and_home을 이어서 실행한다.
        """
        # ── [Vision] 약품 검출 로직 ──
        self.get_logger().info('[Vision] 약품 위치 측정 중...')
        box_pos = self._get_medicine_target()

        fallback_used = False
        if box_pos is None:
            fallback_box_pos = self._on_missing_medicine(contact, pull_dir)
            if fallback_box_pos is None:
                self.get_logger().error(
                    '약품 감지 실패 — 시퀀스를 종료합니다. '
                    '약품이 없으면 다음 단계로 진행하지 않습니다.'
                )
                self._step_info_pub.publish(String(data='Vision 실패 — 약품 미감지'))
                return False
            box_pos = fallback_box_pos
            fallback_used = True

        bx, by, bz = box_pos
        coord_str = (
            f"약품 감지(대체)" if fallback_used else "약품 감지"
        ) + f" (X:{bx:.1f}, Y:{by:.1f}, Z:{bz:.1f})"
        self.get_logger().info(coord_str)
        self._step_info_pub.publish(String(data=f'WAIT:{coord_str}'))

        # 8.5에서 쓸 약품 X,Y 목표 계산.
        # TCP 스윙으로 인한 그리퍼 끝단 Y 오차 수동 보정 (기존 -97mm + 추가 -23mm = -120mm)
        gripper_x_offset = -8.12
        gripper_y_offset = -104.0
        target_x = bx + gripper_x_offset
        target_y = by + gripper_y_offset

        # MoveC 시작 전 실제 TCP 전체(위치+자세) 기억
        if not self._wait_tcp():
            self.get_logger().error('MoveC 전 TCP 위치 미수신')
            return False
        pre_movec_pos = tuple(self._tcp[:6])  # (x, y, z, rx, ry, rz)
        cx, cy, cz = pre_movec_pos[0], pre_movec_pos[1], pre_movec_pos[2]

        # 8. MoveC 호 이동 (캐비닛 5번은 MoveJ 경유 이동으로 대체)
        if drawer_index == 4:   # 캐비닛 5번
            if not self._wait_for_step('8. MoveJ 경유 이동 (캐비닛 5번)'): return False
            if not self._move_j(
                [9.13, 27.07, 62.71, -42.23, 100.62, -95.49],
                profile='TRANSIT',
            ): return False
            if not self._move_j(
                [-16.97, 46.67, 32.19, 0.0, 101.14, -106.97],
                profile='TRANSIT',
            ): return False
            if not self._wait_tcp():
                self.get_logger().error('8번 MoveJ 경유 이동 후 TCP 위치 미수신')
                return False
            cx, cy, cz = self._tcp[0], self._tcp[1], self._tcp[2]
        else:
            if not self._wait_for_step(f'8. MoveC r={radius_mm:.0f} 호 이동'): return False
            end_pos = self._move_c(radius_mm, -1)
            if end_pos is None:
                self._step_info_pub.publish(String(data='MoveC 실패 — 작업 공간 초과 가능'))
                return False
            cx, cy, cz = end_pos

        # 8.5 약품 X, Y 위치로 정렬 (현재 높이 유지, target_x/target_y는 MoveC 전에 계산함)
        if not self._wait_for_step(f'8.5 약품 XY 정렬 (X:{target_x:.0f}, Y:{target_y:.0f})'): return False
        # 호 이동이 끝나면 카메라가 아래를 보는 자세(rx=90, ry=-180, rz=0)가 됨. 이 자세 유지.
        if not self._move_l(
            target_x, target_y, cz, 90.0, -180.0, 0.0,
            profile='VISION_ALIGN',
        ): return False
        cx, cy = target_x, target_y # 이제 현재 위치는 약품 바로 위

        # 9. Z 하강 (비전 좌표 + 그리퍼 길이 보정 + 수동 보정)
        # 그리퍼 길이 97mm + 안전거리 5mm 에, 추가로 12mm 더 깊게 하강 (-12mm 보정)
        gripper_z_offset = 97.0
        target_z = bz + 5.0 + gripper_z_offset - 13.87
        if drawer_index in (4, 5):   # 캐비닛 5~6번 — 바닥/서랍 구조물 충돌 방지용 Z 하한
            target_z = max(target_z, 278.0)
        if not self._wait_for_step(f'9. Z 하강 (목표 Z={target_z:.1f})'): return False
        if not self._move_l(
            cx, cy, target_z, 90.0, -180.0, 0.0,
            profile='CONTACT',
        ): return False
        cz = target_z

        # 10. 전자석 ON
        if not self._wait_for_step('10. 전자석 ON'): return False
        if not self._set_magnet(True): return False

        # 11~12. 픽업 후 후퇴 — 캐비닛 1~3번은 원호 후퇴, 4~6번은 기존 방식(단순 상승)
        if drawer_index in (0, 1, 2):   # 캐비닛 1~3번 — 원호 후퇴
            retreat_approach = (
                pull_target[0],
                pull_target[1] - 170.0,
                pull_target[2] + 160.0,
            )
            if not self._wait_for_step('11. 손잡이 기준 후퇴 접근점으로 이동'): return False
            if not self._move_l(*retreat_approach, 90.0, -180.0, 0.0, profile='LIFT'): return False

            # 12. 원호 후퇴 (test_moveC 150 2와 동일 동작) — Ry -180°→-90° 표준 포즈로 복귀
            if not self._wait_for_step('12. 원호 후퇴 (r=150mm)'): return False
            if not self._move_c_retreat(150.0): return False
        else:   # 캐비닛 4~6번 — 기존 방식. 두 구간 사이를 blend로 매끄럽게 이어붙인다.
            if not self._wait_for_step('11. Z +100mm 상승'): return False
            if not self._move_l(
                0, 0, 100,
                relative=True,
                blend_radius='SMALL',
                profile='LIFT',
            ): return False

            if not self._wait_for_step('12. Y+50 Z+50 이동'): return False
            if not self._move_l(
                0, 50, 50,
                relative=True,
                blend_radius='SMALL',
                profile='LIFT',
            ): return False

        # 13. MoveJ 카메라 앞 정렬
        if not self._wait_joints():
            self.get_logger().error('13번 전 관절 위치 미수신')
            return False
        pre_camera_joints = tuple(self._joints[:6])
        if not self._wait_for_step('13. MoveJ 카메라 앞 정렬'): return False
        if drawer_index in (0, 1):   # 캐비닛 1~2번 — 짧은 경로
            if not self._move_j(
                [1.67, 8.24, 82.41, 0.0, 0.0, -90.00],
                profile='OCR_APPROACH', blend_radius='SMALL',
            ): return False
        else:   # 캐비닛 3~6번 — 기존 경로
            if not self._move_j(
                [-12.78, -20.47, 78.55, 12.02, 67.58, -113.36],
                profile='OCR_APPROACH', blend_radius='SMALL',
            ): return False
            if not self._move_j(
                [3.74, 22.84, 111.01, 8.49, -78.55, -90.00],
                profile='OCR_APPROACH',
            ): return False
        if not self._move_j(
            [-2.21, 44.37, 61.78, -0.00, -107.20, -90.00],
            profile='OCR_APPROACH',
        ): return False

        # 13.5. OCR & JSON 파싱
        if not self._wait_for_step('13.5. OCR & JSON 파싱'): return False
        ocr_status = self._ocr_and_parse()

        # [핵심] OCR 결과 분기 — 관리자 확인 없이 자동으로 판단한다.
        #   MATCHED      → 배송 박스로 이동 (14~18)
        #   그 외(불일치) → 자동으로 컨베이어에 내려놓기
        ocr_rollback = False
        if ocr_status != 'MATCHED':
            self.get_logger().error(
                f'[🚨경고] OCR 결과: {ocr_status}. 자동으로 컨베이어에 내려놓습니다.'
            )
            self._step_info_pub.publish(String(data='ROLLBACK:원위치 복구 중'))
            ocr_rollback = True
            if not self._move_j(
                [1.14, 13.16, 122.67, 6.37, -107.20, -180.00],
                profile='OCR_ROLLBACK',
                blend_radius=2.0,
            ): return False
            if not self._move_j(
                [-1.49, -15.16, 83.61, 0.0, 21.20, -180.0],
                profile='OCR_ROLLBACK',
                blend_radius=2.0 ,
            ): return False
            if not self._move_j(
                [-80.0, -19.25, 57.07, 0.0, 90.0, -180.0],
                profile='OCR_ROLLBACK',
                blend_radius=2.0,
            ): return False
            if not self._move_j(
                [-90.0, 17.75, 82.87, 0.0, 80.0, -180.0],
                profile='OCR_ROLLBACK',
            ): return False
            time.sleep(1.0)
            if not self._set_magnet(False): return False
            time.sleep(0.5)
            self._set_inverter_freq(freq=3000)  # 컨베이어 주파수 30Hz
            self._set_inverter_run(True)  # 컨베이어 ON
            if not self._move_j(
                [-80.0, -19.25, 57.07, 0.0, 90.0, -180.0],
                profile='OCR_ROLLBACK',
                blend_radius=2.0,
            ): return False
            if not self._move_j(
                [0.0, -15.16, 83.61, 0.0, 21.20, -180.0],
                profile='OCR_ROLLBACK',
                blend_radius=2.0,
            ): return False
            if not self._move_j(
                [34.69, 0.76, 77.49, -57.51, 92.50, -180.00],
                profile='OCR_ROLLBACK',
                blend_radius=2.0,
            ): return False
            if not self._move_j(
                [32.18, 37.57, 77.48, -75.09, 118.85, -61.10],
                profile='OCR_ROLLBACK',
            ): return False
            self._set_inverter_run(False)  # 컨베이어 OFF
        else:
            self.get_logger().info('OCR 일치 확인. 배송 박스로 이동합니다.')

        if self._stop_requested: return False

        # 14~18: 배송 박스 전달 (롤백 시 건너뜀) — 팔레타이징 좌표(DB 연계 plan)로 배치한다.
        # 흐름(PALLETIZING.md P1~P7): 그리퍼 yaw 측정 → 박스 staging 자세 →
        # plan에서 다음 슬롯 조회 → 박스 마커로 theta_box/center 측정(캐시) →
        # 슬롯 좌표 계산 → XY+회전 정렬 → Z 하강 → 전자석 OFF → 상승 → 배치 보고 →
        # MoveC 전 위치로 복귀.


        if not ocr_rollback:
            from .palletizing_planner import compute_placement, next_slot, wrap_deg as _wrap_deg

            theta_item = self._get_grasped_yaw()   # P1
            if not self._wait_for_step('14. 병동 박스 staging 자세로 이동'): return False
            if not self._move_j(
                [31.04, 48.6, 38.63, 0.0, 92.77, -58.96], profile='TRANSIT',
            ): return False   # P2

            plan = self._fetch_pallet_plan()
            if plan is None:
                self._step_info_pub.publish(String(data='팔레타이징 실패 — plan 조회 불가'))
                return False
            box = plan['box']
            slot = next_slot(plan['layout'], self._last_ocr_medicine_name)
            if slot is None:
                self.get_logger().error('[팔레타이징] 배치할 슬롯 없음 (전부 배치됨 또는 plan 비어있음)')
                self._step_info_pub.publish(String(data='팔레타이징 실패 — 빈 슬롯 없음'))
                return False
            # 패킹 알고리즘이 이 슬롯에 90° 회전을 지정했는지 여부.
            # 현재는 그리퍼 물리 회전을 수행하지 않으므로 참고/로깅용 flag로만 사용한다.
            needs_rotation: bool = not bool(slot.get('rot_deg', 0.0))

            marker_id = box.get('aruco_marker_id')
            box_pose = self._get_box_pose(int(marker_id)) if marker_id is not None else None
            if box_pose is not None:
                theta_box, center = box_pose
            else:
                self.get_logger().warning(
                    f'[팔레타이징] 박스 마커 {marker_id} 미검출·캐시 없음 — DB origin fallback 사용'
                )
                theta_box, center = 0.0, tuple(box['origin'])

            enable_orientation = bool(self.get_parameter('enable_orientation_correction').value)
            bx, by, _unused_z, rz = compute_placement(
                slot, box, theta_box, center, theta_item,
                enable_orientation=enable_orientation,
            )
            # Z는 마커 측정값(노이즈가 큼)이 아니라 박스별 calibration 상수(place_floor_z_mm)를 쓴다.
            # place_floor_z_mm = '높이 0인 품목'이 바닥에 닿는 TCP Z (실측 calibration).
            # 거기서 이 품목의 적재높이(stack_h_mm, DB에서 받아온 medicine 치수 기반)만큼 덜 내려가고,
            # 이미 쌓인 다른 품목 위라면 z_offset_mm만큼 추가로 덜 내려간다.
            place_floor_z = float(self.get_parameter('place_floor_z_mm').value)
            stack_h_mm = float(slot.get('stack_h_mm', 0.0) or 0.0)
            z_offset_mm = float(slot.get('z_offset_mm', 0.0) or 0.0)
            place_z = place_floor_z - stack_h_mm - z_offset_mm
            clearance = float(self.get_parameter('approach_clearance_mm').value)
            approach_z = place_z + clearance

            self.get_logger().info(
                '[팔레타이징 좌표] '
                f"slot=#{slot['slot_idx']}({slot['medicine_name']}) "
                f"local=({slot['local_x']:.1f},{slot['local_y']:.1f}) "
                f"size={slot['w']:.1f}x{slot['h']:.1f} "
                f"rot_deg={slot.get('rot_deg', 0.0):.1f} needs_rotation={needs_rotation} "
                f"stack_h_mm={stack_h_mm:.1f} z_offset_mm={z_offset_mm:.1f} | "
                f"box_marker={marker_id}({'측정' if box_pose is not None else 'DB origin fallback'}) "
                f"theta_box={theta_box:.1f} center=({center[0]:.1f},{center[1]:.1f},{center[2]:.1f}) "
                f"theta_item={theta_item:.1f} | "
                f"place_floor_z_mm={place_floor_z:.1f} clearance={clearance:.1f} | "
                f"=> approach=({bx:.1f},{by:.1f},{approach_z:.1f}) "
                f"place=({bx:.1f},{by:.1f},{place_z:.1f}) rz={rz:.1f} (rot_deg 미적용)"
            )

            # staging 자세([31.04, 48.6, 38.63, 0.0, 92.77, -58.96])의 실측 TCP 위치를
            # 기준으로 슬롯까지 상대 이동량을 계산한다. self._tcp 읽기 타이밍 오차를
            # 피하기 위해 실측값을 고정 사용한다.
            _STG_X, _STG_Y, _STG_Z = 558.94, 336.38, 296.69
            dx = bx - _STG_X
            dy = by - _STG_Y
            dz = approach_z - _STG_Z

            # 14.5. 슬롯 상공 XY+Z 상대 이동 — 현재 자세(아래 바라보기) 유지
            if not self._wait_for_step(
                f"14.5 슬롯 #{slot['slot_idx']} 정렬 (X:{bx:.0f}, Y:{by:.0f})"
            ): return False
            if not self._move_l(dx, dy, dz, relative=True, profile='VISION_ALIGN'):
                return False

            # 14.6. needs_rotation 플래그가 True이면 Joint 6을 -90° 상대 회전
            if needs_rotation:
                if not self._wait_joints():
                    self.get_logger().error('[팔레타이징] 14.6 관절 위치 미수신 — 회전 생략')
                    return False
                rot_joints = list(self._joints[:6])
                rot_joints[5] -= 90.0
                if not self._wait_for_step(
                    f"14.6 J6 -90° 회전 ({self._joints[5]:.1f}° → {rot_joints[5]:.1f}°)"
                ): return False
                if not self._move_j(rot_joints, profile='VISION_ALIGN'):
                    return False

            # 15. Z 하강 — 현재 자세 그대로 Z만 내린다 (회전 여부와 무관하게 자세 유지)
            descent_mm = approach_z - place_z
            if not self._wait_for_step(f'15. Z 하강 ({descent_mm:.1f}mm)'): return False
            if not self._move_l(0, 0, -descent_mm, relative=True, profile='CONTACT'):
                return False

            # 16. 전자석 OFF (배치)
            if not self._wait_for_step('16. 전자석 OFF'): return False
            if not self._set_magnet(False): return False

            # 17. Z 상승
            if not self._wait_for_step('17. Z 상승'): return False
            if not self._move_l(
                0, 0, clearance, relative=True, profile='LIFT', blend_radius='SMALL',
            ): return False
            # 배치 완료 보고: 실패 시 최대 3회 재시도, 전부 실패하면 중단
            # (실패 무시 시 배치 배열의 두 번째 픽업이 같은 슬롯을 재선택하는 버그 방지)
            _mark_ok = False
            for _mark_try in range(3):
                if self._mark_pallet_placed(slot['slot_idx']):
                    _mark_ok = True
                    break
                self.get_logger().warning(
                    f'[팔레타이징] 슬롯 {slot["slot_idx"]} 배치 보고 실패 ({_mark_try + 1}/3회)'
                )
                time.sleep(0.3)
            if not _mark_ok:
                self.get_logger().error(
                    f'[팔레타이징] 슬롯 {slot["slot_idx"]} 배치 보고 3회 모두 실패 — 시퀀스 중단'
                )
                return False
            self._step_info_pub.publish(
                String(data=f"WAIT:슬롯 #{slot['slot_idx']} ({slot['medicine_name']}) 배치 완료")
            )

            # 18. MoveC 전 위치로 복귀
            if not self._wait_for_step('18. MoveC 전 위치로 복귀'): return False
            if not self._move_l(pre_movec_pos[0], pre_movec_pos[1], pre_movec_pos[2],
                                pre_movec_pos[3], pre_movec_pos[4], pre_movec_pos[5],
                                profile='TRANSIT',
                                blend_radius='SMALL'): return False

        return True

    def _close_drawer_and_home(
        self, pull_target: tuple, contact: tuple, pull_dir: tuple,
    ) -> bool:
        """19~24단계: 서랍을 닫고 홈으로 복귀한다.

        픽업/배송(_pick_verify_and_deliver)의 성공 여부와 무관하게 항상 호출되어야
        한다 — 그렇지 않으면 비전 실패 등으로 시퀀스가 중단될 때 서랍이 열린 채로
        방치된다.
        """
        # 19. 열린 서랍의 손잡이 위치로 재접근
        if not self._wait_for_step('19. 열린 서랍 손잡이 재접근'): return False
        if not self._move_l(*pull_target, 90.0, -90.0, 0.0, profile='APPROACH'): return False

        # 20. 전자석 ON
        if not self._wait_for_step('20. 전자석 ON'): return False
        if not self._set_magnet(True): return False

        # 21. 원래 손잡이 위치까지 200mm 밀어 서랍 닫기
        if not self._wait_for_step('21. 서랍 200mm 닫기'): return False
        if not self._move_l(*contact, 90.0, -90.0, 0.0, profile='CONTACT'): return False

        # 22. 전자석 OFF
        if not self._wait_for_step('22. 전자석 OFF'): return False
        if not self._set_magnet(False): return False

        # 23. 닫힌 손잡이에서 50mm 후퇴
        final_retreat = tuple(contact[i] + pull_dir[i] * 50.0 for i in range(3))
        if not self._wait_for_step('23. 손잡이에서 50mm 후퇴'): return False
        if not self._move_l(
            *final_retreat, 90.0, -90.0, 0.0,
            blend_radius='SMALL',
            profile='APPROACH',
        ): return False

        # 24. 최종 홈 정렬
        if not self._wait_for_step('24. 최종 홈 정렬'): return False
        if not self._home(): return False
        return True

    def run_sequence(self, drawer_index: int, radius_mm: float):
        self._is_running = True
        try:
            self._set_camera_active_drawer(drawer_index + 1)

            # 1. 홈
            self._set_plc_coil(0x20, True)   # M20 ON — 시퀀스 시작
            if not self._wait_for_step('1. 홈 이동'): return
            if not self._home(): return

            if not self._wait_tcp():
                self.get_logger().error('TCP 위치 미수신')
                return

            # 2. 탑뷰 마커에서 선택한 서랍의 안전 접근점 계산 및 이동
            if not self._wait_for_step(f'2. 서랍 {drawer_index + 1} 접근점으로 이동'): return
            result = self._get_drawer_target(drawer_index)
            if result is None: return
            approach, contact, pull_dir = result
            if not self._move_l_hybrid(
                *approach, 90.0, -90.0, 0.0,
                profile='APPROACH',
            ): return
            cx, cy, cz = approach

            # 3. 마커 기준 앞/뒤 방향을 반영해 손잡이에 접촉
            if not self._wait_for_step('3. 손잡이 접촉'): return
            if not self._move_l(*contact, 90.0, -90.0, 0.0, profile='CONTACT'): return
            cx, cy, cz = contact

            # 4. 전자석 ON (픽)
            if not self._wait_for_step('4. 전자석 ON'): return
            if not self._set_magnet(True): return

            # 5. 사물함 앞쪽으로 200mm 당김
            pull_distance = float(self._cabinet_geometry['pull_mm'])
            pull_target = tuple(contact[i] + pull_dir[i] * pull_distance for i in range(3))
            if not self._wait_for_step('5. 서랍 200mm 당김'): return
            if not self._move_l(*pull_target, 90.0, -90.0, 0.0, profile='CONTACT'): return
            cx, cy, cz = pull_target

            # 6. 전자석 OFF
            if not self._wait_for_step('6. 전자석 OFF'): return
            if not self._set_magnet(False): return

            # 7. 열린 서랍 손잡이에서 후퇴 — Y 방향으로 50mm 상대 이동
            if not self._wait_for_step('7. 손잡이에서 Y+50mm 후퇴'): return
            if not self._move_l(
                0, 50, 0,
                relative=True,
                profile='APPROACH',
            ): return

            # ── 8~18: 약품 픽업 → OCR 검수 → 배송(또는 롤백) ──
            # 이 시점부터는 서랍이 열려 있으므로, 실패(예외 포함)하더라도 반드시
            # 서랍을 닫고 홈으로 복귀시킨다(아래 _close_drawer_and_home은 결과와 무관하게 항상 실행).
            pick_ok = True
            try:
                pick_ok = self._pick_verify_and_deliver(
                    drawer_index, radius_mm, contact, pull_dir, pull_target,
                )
            except Exception as exc:
                self.get_logger().error(f'픽업/배송 중 예외 발생: {exc}')
                pick_ok = False

            # 19~24: 서랍 닫기 + 홈 복귀 — 픽업/배송 성공 여부와 무관하게 항상 시도
            close_ok = self._close_drawer_and_home(pull_target, contact, pull_dir)

            if not pick_ok or not close_ok:
                return

            self.get_logger().info('시퀀스 완료')
            self._wait_for_step('시퀀스 완료')
        except Exception as e:
            self.get_logger().error(f'Sequence error: {e}')
        finally:
            self._clear_camera_active_drawer()
            self._set_plc_coil(0x20, False)  # M20 OFF — 정상/중단/예외 모든 경로에서 종료 보장
            self._is_running = False
            self._step_info_pub.publish(String(data='IDLE'))


def main(args=None):
    if not _acquire_instance_lock():
        print(
            'motion_sequence is already running; refusing duplicate instance.',
            file=sys.stderr,
        )
        return 1

    rclpy.init(args=args)
    node = MotionSequenceNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # CLI 지원 (서랍 번호는 사용자 기준 1~6)
    if len(sys.argv) > 1 and sys.argv[1].isdigit():
        drawer_number = int(sys.argv[1])
        if drawer_number < 1 or drawer_number > 6:
            print(f'Invalid drawer number {drawer_number}; expected 1..6', file=sys.stderr)
            return 1
        drawer_index = drawer_number - 1
        radius = float(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_RADIUS
        node._step_mode = '--step' in sys.argv
        seq_thread = threading.Thread(target=node.run_sequence, args=(drawer_index, radius))
        seq_thread.start()

    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node._stop_requested = True
        node._next_step_event.set()
        thread = node._current_sequence_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=5.0)
        node._plc_safety_off()
        node.destroy_node()
        rclpy.shutdown()
