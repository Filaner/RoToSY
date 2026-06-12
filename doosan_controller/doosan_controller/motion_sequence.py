"""
Motion Sequence Node with Interactive (Step-by-Step) Control.

Usage:
  1. Persistent Node:
     ros2 run doosan_controller motion_sequence
  2. One-shot CLI:
     ros2 run doosan_controller motion_sequence <marker_id> [--step]
"""

import base64
import fcntl
import json
import math
import os
import sys
import threading
import time
import urllib.request

try:
    from groq import Groq as _Groq
    _GROQ_AVAILABLE = True
except ImportError:
    _GROQ_AVAILABLE = False

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Empty, String, Int32

from dsr_msgs2.srv import MoveCircle
from robot_arm_interfaces.action import MoveJ, MoveL
from robot_arm_interfaces.msg import RobotStatus
from robot_arm_interfaces.srv import Home
from rotosy_gripper_control.keyboard_electromagnet_gripper import KeyboardElectromagnetGripper

# ── 공통 파라미터 ────────────────────────────────────────────────────────────

CAMERA_API          = 'http://localhost:8000/camera/markers'
MEDICINE_DETECTION_API = 'http://localhost:8000/camera/detections'
CAMERA_SNAPSHOT_API = 'http://localhost:8000/camera/snapshot'
OCR_VERIFY_API      = 'http://localhost:8080/api/ocr/verify'
ROBOT_NS            = 'dsr01'
GROQ_API_KEY        = os.environ.get('GROQ_API_KEY', '')

VEL_MM           = 30.0
ACC_MM           = 60.0
VEL_DEG          = 30.0
ACC_DEG          = 60.0

DEFAULT_RADIUS   = 200.0   # MoveC 기본 반지름 (mm)
DEFAULT_RZ       = 0.0

# Top-view cabinet geometry. /motion/start still receives the existing slot
# index 0..5, while every slot is located from the fixed cabinet marker ID 1.
CABINET_MARKER_ID = 1
DRAWER_REFERENCE_SLOT = 1       # zero-based slot 1 == drawer 2
DRAWER_REFERENCE_OFFSET_MM = (80.0, 36.0, -60.0)
DRAWER_COLUMN_PITCH_MM = 227.0
DRAWER_ROW_PITCH_MM = 112.0
DRAWER_APPROACH_MM = 30.0
DRAWER_PULL_MM = 200.0
GRIPPER_LENGTH_MM = 97.0

_INSTANCE_LOCK_FD = None


def _acquire_instance_lock() -> bool:
    """Prevent duplicate sequence nodes from subscribing to command topics."""
    global _INSTANCE_LOCK_FD
    fd = os.open('/tmp/rotosy_motion_sequence.lock', os.O_CREAT | os.O_RDWR, 0o644)
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

    def __init__(self):
        super().__init__('motion_sequence_node')
        self._cb_group = ReentrantCallbackGroup()

        # Clients
        self._home_cli   = self.create_client(Home, '/arm/home', callback_group=self._cb_group)
        self._movel      = ActionClient(self, MoveL, '/arm/move_l', callback_group=self._cb_group)
        self._movej      = ActionClient(self, MoveJ, '/arm/move_j', callback_group=self._cb_group)
        self._cli_circle = self.create_client(
            MoveCircle, f'/{ROBOT_NS}/motion/move_circle', callback_group=self._cb_group
        )
        self._gripper = KeyboardElectromagnetGripper(self)

        # Subscribers
        self._status_sub = self.create_subscription(
            RobotStatus, '/arm/status', self._status_cb, 10
        )
        self._next_sub = self.create_subscription(
            Empty, '/motion/next_step', self._next_step_cb, 10, callback_group=self._cb_group
        )
        self._start_sub = self.create_subscription(
            Int32, '/motion/start', self._start_cb, 10, callback_group=self._cb_group
        )
        self._stop_sub = self.create_subscription(
            Empty, '/motion/stop', self._stop_cb, 10, callback_group=self._cb_group
        )
        self._reset_sub = self.create_subscription(
            Empty, '/motion/reset', self._reset_cb, 10, callback_group=self._cb_group
        )

        # Publishers
        self._step_info_pub = self.create_publisher(String, '/motion/step_info', 10)

        # State
        self._tcp    = None   # [x, y, z, rx, ry, rz] mm / deg
        self._joints = None   # [j1..j6] deg

        self._step_mode = False
        self._next_step_event = threading.Event()
        self._stop_requested = False
        self._current_sequence_thread = None
        self._is_running = False
        self._active_goal_handle = None
        self._state_lock = threading.Lock()

    # ── 콜백 ─────────────────────────────────────────────────────────────────

    def _status_cb(self, msg: RobotStatus):
        self._tcp    = list(msg.current_tcp)
        self._joints = list(msg.current_joints_deg)

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
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warning(f'Active goal cancel failed: {exc}')

        if not self._set_magnet(False):
            self.get_logger().error('Failed to turn gripper OFF while stopping sequence')

        thread = self._current_sequence_thread
        if thread is None or not thread.is_alive():
            self._finish_reset()
            return

        threading.Thread(
            target=self._wait_for_reset, args=(thread,), daemon=True
        ).start()

    def _wait_for_reset(self, thread):
        thread.join(timeout=10.0)
        if thread.is_alive():
            self.get_logger().warning('Sequence is still stopping')
            self._step_info_pub.publish(String(data='STOPPING'))
            return
        self._finish_reset()

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

        marker_id = msg.data
        self._step_mode = True
        self._stop_requested = False

        self.get_logger().info(f'Starting sequence for marker {marker_id} (Step Mode: {self._step_mode})')
        self._current_sequence_thread = threading.Thread(
            target=self.run_sequence, args=(marker_id, DEFAULT_RADIUS)
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

    # ── 동작 헬퍼 ────────────────────────────────────────────────────────────

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
                blend_radius: float = 0.0,
                vel_mm: float = VEL_MM, vel_deg: float = VEL_DEG,
                acc_mm: float = ACC_MM, acc_deg: float = ACC_DEG) -> bool:
        mode = '상대' if relative else '절대'
        self.get_logger().info(
            f'MoveL({mode}) → ({x:.1f}, {y:.1f}, {z:.1f})  rx={rx} ry={ry} rz={rz}'
        )
        self._movel.wait_for_server()
        goal = MoveL.Goal()
        goal.x, goal.y, goal.z     = float(x), float(y), float(z)
        goal.rx, goal.ry, goal.rz  = float(rx), float(ry), float(rz)
        goal.linear_velocity_mm_s   = vel_mm
        goal.angular_velocity_deg_s = vel_deg
        goal.linear_accel_mm_s2     = acc_mm
        goal.angular_accel_deg_s2   = acc_deg
        goal.blend_radius_mm        = float(blend_radius)
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

    def _move_j(self, joint_angles: list, blend_radius: float = 0.0) -> bool:
        self.get_logger().info(
            f'MoveJ → {[round(a, 1) for a in joint_angles]}'
            + (f'  blend={blend_radius}°' if blend_radius > 0 else '')
        )
        self._movej.wait_for_server()
        goal = MoveJ.Goal()
        goal.joint_angles_deg    = [float(a) for a in joint_angles]
        goal.velocity_deg_s      = VEL_DEG
        goal.acceleration_deg_s2 = ACC_DEG
        goal.blend_radius_mm     = float(blend_radius)

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

    def _set_magnet(self, enabled: bool) -> bool:
        """전자석 ON(enabled=True) / OFF(enabled=False)."""
        return self._gripper.set_gripper(enabled)

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
        BLEND = 20.0

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

            blend = 0.0 if i == N else BLEND

            self.get_logger().info(
                f'  [{i}/{N}] θ={theta_deg:.0f}°'
                f'  pos=({px:.1f},{py:.1f},{pz:.1f})'
                f'  ori=({r_x:.0f},{r_y:.0f},{r_z:.0f})'
            )

            if not self._move_l(px, py, pz, r_x, r_y, r_z, blend_radius=blend):
                self.get_logger().error(f'호 이동 실패 θ={theta_deg:.0f}°')
                return None

        end_pos = (x0, Cy, Cz + r)
        self.get_logger().info(f'호 이동 완료  end=({end_pos[0]:.1f},{end_pos[1]:.1f},{end_pos[2]:.1f})')
        return end_pos

    # ── OCR 파이프라인 ────────────────────────────────────────────────────────

    def _capture_image(self) -> bytes | None:
        """web_interface에서 현재 카메라 프레임을 JPEG bytes로 취득."""
        try:
            with urllib.request.urlopen(CAMERA_SNAPSHOT_API, timeout=5) as resp:
                return resp.read()
        except Exception as e:
            self.get_logger().error(f'[OCR] 스냅샷 취득 실패: {e}')
            return None

    def _ocr_and_parse(self) -> dict | None:
        """카메라 스냅샷 → Groq llama-4-scout (이미지 직접 전달) → JSON 파싱 → ROS 토픽 발행."""
        if not _GROQ_AVAILABLE:
            self.get_logger().error('[OCR] groq 패키지 미설치: pip install groq')
            return None

        self.get_logger().info('[OCR] 카메라 스냅샷 취득...')
        img = self._capture_image()
        if img is None:
            return None

        self.get_logger().info('[OCR] Groq llama-4-scout 비전 분석 중...')
        try:
            img_b64 = base64.b64encode(img).decode()
            client = _Groq(api_key=GROQ_API_KEY)
            chat = client.chat.completions.create(
                model='meta-llama/llama-4-scout-17b-16e-instruct',
                max_tokens=512,
                messages=[{
                    'role': 'user',
                    'content': [
                        {
                            'type': 'image_url',
                            'image_url': {'url': f'data:image/jpeg;base64,{img_b64}'},
                        },
                        {
                            'type': 'text',
                            'text': (
                                '이 약품 라벨에서 텍스트를 읽어서 아래 JSON 형식으로만 반환하세요. '
                                '다른 설명 없이 유효한 JSON만 출력하세요.\n'
                                '{\n'
                                '  "medicine_name": "약품명",\n'
                                '  "dosage": "용량",\n'
                                '  "instructions": "복용법",\n'
                                '  "patient_name": "환자명 또는 null",\n'
                                '  "prescription_date": "처방일 또는 null",\n'
                                '  "ward": "병동 또는 null",\n'
                                '  "raw_text": "이미지에서 읽은 텍스트 전체"\n'
                                '}'
                            ),
                        },
                    ],
                }],
            )
            text = chat.choices[0].message.content.strip()
            if '```' in text:
                text = text.split('```')[1]
                if text.startswith('json'):
                    text = text[4:]
                text = text.strip()
            result = json.loads(text)
        except Exception as e:
            self.get_logger().error(f'[OCR] Groq 비전 호출 실패: {e}')
            return None

        self.get_logger().info(f'[OCR] 파싱 결과: {result}')
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
        except Exception as e:
            self.get_logger().warn(f'[OCR] hospital_web 전송 실패 (계속 진행): {e}')

        return result

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
        """web_interface에서 YOLO로 검출된 약품(medicine) 또는 수액(water_pack)의 베이스 좌표를 취득."""
        for attempt in range(retries):
            try:
                with urllib.request.urlopen(MEDICINE_DETECTION_API, timeout=3) as resp:
                    data = json.loads(resp.read())

                # 'medicine' 또는 'water_pack' 클래스 중 유효한 좌표가 있는 대상을 모두 선택
                targets = [
                    d for d in data.get('detections', [])
                    if d['class_name'].lower() in ['medicine', 'water_pack']
                    and d.get('base_position_m') is not None
                ]

                if targets:
                    # 신뢰도 내림차순 정렬하여 가장 확실한 대상을 선택
                    targets.sort(key=lambda x: x['confidence'], reverse=True)
                    best = targets[0]['base_position_m']
                    self.get_logger().info(f"Target found: {targets[0]['class_name']} (conf: {targets[0]['confidence']:.2f})")
                    # m -> mm 변환
                    return best[0] * 1000.0, best[1] * 1000.0, best[2] * 1000.0
            except Exception as e:
                self.get_logger().warning(f'목표 좌표 취득 시도 {attempt+1}: {e}')
            time.sleep(0.5)
        return None

    # ── 시퀀스 ───────────────────────────────────────────────────────────────

    @staticmethod
    def _mat_vec(R: list, v: tuple) -> tuple:
        return tuple(sum(float(R[row][col]) * v[col] for col in range(3)) for row in range(3))

    def _drawer_offset(self, slot_index: int) -> tuple | None:
        """Return handle-center offset in the cabinet marker frame.

        Cabinet axes are +X left, +Y toward the robot/drawer pull direction,
        and +Z upward. Slot numbering remains row-major 0..5.
        """
        if slot_index < 0 or slot_index > 5:
            self.get_logger().error(f'서랍 번호 범위 초과: {slot_index} (0~5 필요)')
            return None

        row, col = divmod(slot_index, 2)
        ref_row, ref_col = divmod(DRAWER_REFERENCE_SLOT, 2)
        ref_x, ref_y, ref_z = DRAWER_REFERENCE_OFFSET_MM
        x = ref_x + (ref_col - col) * DRAWER_COLUMN_PITCH_MM
        z = ref_z - (row - ref_row) * DRAWER_ROW_PITCH_MM
        return x, ref_y, z

    def _get_drawer_target(self, slot_index: int) -> tuple | None:
        """Compute handle and flange targets from the top-view cabinet marker."""
        offset = self._drawer_offset(slot_index)
        if offset is None:
            return None

        self.get_logger().info(
            f'[서랍 {slot_index + 1}] 기준 마커 ID {CABINET_MARKER_ID} 좌표 취득 중...'
        )
        marker = self._get_marker(CABINET_MARKER_ID)
        if marker is None:
            self.get_logger().error(
                f'ID {CABINET_MARKER_ID} 마커 위치/회전 미수신'
            )
            return None

        R_marker = marker['rotation_matrix_base']
        # OpenCV marker axes: +X right, +Y marker top(back), +Z upward.
        # Cabinet axes requested here: +X left, +Y front/pull, +Z upward.
        R_cabinet = [
            [-float(R_marker[row][0]), -float(R_marker[row][1]), float(R_marker[row][2])]
            for row in range(3)
        ]
        world_offset = self._mat_vec(R_cabinet, offset)
        marker_pos = (float(marker['x_mm']), float(marker['y_mm']), float(marker['z_mm']))
        handle = tuple(marker_pos[i] + world_offset[i] for i in range(3))

        pull_dir = self._mat_vec(R_cabinet, (0.0, 1.0, 0.0))
        # Existing pose points the 97 mm gripper opposite the pull direction.
        contact_flange = tuple(handle[i] + pull_dir[i] * GRIPPER_LENGTH_MM for i in range(3))
        approach_flange = tuple(
            contact_flange[i] + pull_dir[i] * DRAWER_APPROACH_MM for i in range(3)
        )

        self.get_logger().info(
            f'마커=({marker_pos[0]:.1f},{marker_pos[1]:.1f},{marker_pos[2]:.1f}) '
            f'오프셋=({offset[0]:.1f},{offset[1]:.1f},{offset[2]:.1f}) '
            f'손잡이=({handle[0]:.1f},{handle[1]:.1f},{handle[2]:.1f}) '
            f'접근 플랜지=({approach_flange[0]:.1f},{approach_flange[1]:.1f},{approach_flange[2]:.1f})'
        )
        return approach_flange, contact_flange, pull_dir

    def run_sequence(self, marker_id: int, radius_mm: float):
        self._is_running = True
        try:
            # 1. 홈
            if not self._wait_for_step('1. 홈 이동'): return
            if not self._home(): return

            if not self._wait_tcp():
                self.get_logger().error('TCP 위치 미수신')
                return

            # 2. 탑뷰 마커에서 선택한 서랍의 안전 접근점 계산 및 이동
            if not self._wait_for_step(f'2. 서랍 {marker_id + 1} 접근점으로 이동'): return
            result = self._get_drawer_target(marker_id)
            if result is None: return
            approach, contact, pull_dir = result
            if not self._move_l(*approach, 90.0, -90.0, 0.0): return
            cx, cy, cz = approach

            # 3. 마커 기준 앞/뒤 방향을 반영해 손잡이에 접촉
            if not self._wait_for_step('3. 손잡이 접촉'): return
            if not self._move_l(*contact, 90.0, -90.0, 0.0): return
            cx, cy, cz = contact

            # 4. 전자석 ON (픽)
            if not self._wait_for_step('4. 전자석 ON'): return
            if not self._set_magnet(True): return

            # 5. 사물함 앞쪽으로 200mm 당김
            pull_target = tuple(contact[i] + pull_dir[i] * DRAWER_PULL_MM for i in range(3))
            if not self._wait_for_step('5. 서랍 200mm 당김'): return
            if not self._move_l(*pull_target, 90.0, -90.0, 0.0): return
            cx, cy, cz = pull_target

            # 6. 전자석 OFF
            if not self._wait_for_step('6. 전자석 OFF'): return
            if not self._set_magnet(False): return

            # 7. 열린 서랍 손잡이에서 30mm 후퇴
            released_target = tuple(
                pull_target[i] + pull_dir[i] * DRAWER_APPROACH_MM for i in range(3)
            )
            if not self._wait_for_step('7. 손잡이에서 30mm 후퇴'): return
            if not self._move_l(*released_target, 90.0, -90.0, 0.0): return
            cx, cy, cz = released_target

            # ── [Vision] 약품 검출 로직 삽입 ──
            self.get_logger().info('[Vision] 약품 위치 측정 중...')
            box_pos = self._get_medicine_target()

            if box_pos is not None:
                bx, by, bz = box_pos
                # 웹 인터페이스에 약품 좌표 표시 (WAIT/RUN 라벨 활용)
                coord_str = f"약품 감지 (X:{bx:.1f}, Y:{by:.1f}, Z:{bz:.1f})"
                self.get_logger().info(coord_str)
                self._step_info_pub.publish(String(data=f'WAIT:{coord_str}'))

                # MoveC 시작 전 실제 TCP 전체(위치+자세) 기억
                if not self._wait_tcp():
                    self.get_logger().error('MoveC 전 TCP 위치 미수신')
                    return
                pre_movec_pos = tuple(self._tcp[:6])  # (x, y, z, rx, ry, rz)
                cx, cy, cz = pre_movec_pos[0], pre_movec_pos[1], pre_movec_pos[2]

                # 8. MoveC 호 이동
                if not self._wait_for_step(f'8. MoveC r={radius_mm:.0f} 호 이동'): return
                end_pos = self._move_c(radius_mm, -1)
                if end_pos is None:
                    self._step_info_pub.publish(String(data='MoveC 실패 — 작업 공간 초과 가능'))
                    return
                cx, cy, cz = end_pos

                # 8.5 약품 X, Y 위치로 정렬 (현재 높이 유지)
                # TCP 스윙으로 인한 그리퍼 끝단 Y 오차 수동 보정 (기존 -97mm + 추가 -23mm = -120mm)
                gripper_x_offset = 0.0
                gripper_y_offset = -120.0
                target_x = bx + gripper_x_offset
                target_y = by + gripper_y_offset

                if not self._wait_for_step(f'8.5 약품 XY 정렬 (X:{target_x:.0f}, Y:{target_y:.0f})'): return
                # 호 이동이 끝나면 카메라가 아래를 보는 자세(rx=90, ry=-180, rz=0)가 됨. 이 자세 유지.
                if not self._move_l(target_x, target_y, cz, 90.0, -180.0, 0.0): return
                cx, cy = target_x, target_y # 이제 현재 위치는 약품 바로 위

                # 9. Z 하강 (비전 좌표 + 그리퍼 길이 보정 + 수동 보정)
                # 그리퍼 길이 97mm + 안전거리 5mm 에, 추가로 12mm 더 깊게 하강 (-12mm 보정)
                gripper_z_offset = 97.0
                target_z = bz + 5.0 + gripper_z_offset - 12.0
                if not self._wait_for_step(f'9. Z 하강 (목표 Z={target_z:.1f})'): return
                if not self._move_l(cx, cy, target_z, 90.0, -180.0, 0.0): return
                cz = target_z

                # 10. 전자석 ON
                if not self._wait_for_step('10. 전자석 ON'): return
                if not self._set_magnet(True): return

                # 11. Z +100mm 상승
                if not self._wait_for_step('11. Z +100mm 상승'): return
                if not self._move_l(0, 0, 100, relative=True): return
                cz += 100

                # 12. Y+50 Z+50 이동
                if not self._wait_for_step('12. Y+50 Z+50 이동'): return
                if not self._move_l(0, 50, 50, relative=True): return
                cy += 50
                cz += 50

                # 13. MoveJ 카메라 앞 정렬
                if not self._wait_for_step('13. MoveJ 카메라 앞 정렬'): return
                if not self._move_j([20.44, 53.45, 49.27, 158.82, 97.28, -25.6]): return

                # 13.5. OCR & JSON 파싱
                if not self._wait_for_step('13.5. OCR & JSON 파싱'): return
                self._ocr_and_parse()

                # 14. MoveJ 병동 박스 정렬
                if not self._wait_for_step('14. MoveJ 병동 박스 정렬'): return
                if not self._move_j([31.04, 48.6, 38.63, 0.0, 92.77, 121.04]): return

                # 15. Z -150mm 하강
                if not self._wait_for_step('15. Z -150mm 하강'): return
                if not self._move_l(0, 0, -150, relative=True): return
                cz -= 150

                # 16. 전자석 OFF (배치)
                if not self._wait_for_step('16. 전자석 OFF'): return
                if not self._set_magnet(False): return

                # 17. Z +150mm 상승
                if not self._wait_for_step('17. Z +150mm 상승'): return
                if not self._move_l(0, 0, 150, relative=True): return
                cz += 150

                # 18. MoveC 전 위치로 복귀
                if not self._wait_for_step('18. MoveC 전 위치로 복귀'): return
                if not self._move_l(pre_movec_pos[0], pre_movec_pos[1], pre_movec_pos[2],
                                    pre_movec_pos[3], pre_movec_pos[4], pre_movec_pos[5]): return
                cx, cy, cz = pre_movec_pos[0], pre_movec_pos[1], pre_movec_pos[2]

            else:
                self.get_logger().warn('약품 감지 실패 — 파싱 시퀀스 건너뛰고 서랍 닫기로 이동')
                self._step_info_pub.publish(String(data='Vision 실패 — 약품 미감지'))

            # 19. 열린 서랍의 손잡이 위치로 재접근
            if not self._wait_for_step('19. 열린 서랍 손잡이 재접근'): return
            if not self._move_l(*pull_target, 90.0, -90.0, 0.0): return
            cx, cy, cz = pull_target

            # 20. 전자석 ON
            if not self._wait_for_step('20. 전자석 ON'): return
            if not self._set_magnet(True): return

            # 21. 원래 손잡이 위치까지 200mm 밀어 서랍 닫기
            if not self._wait_for_step('21. 서랍 200mm 닫기'): return
            if not self._move_l(*contact, 90.0, -90.0, 0.0): return
            cx, cy, cz = contact

            # 22. 전자석 OFF
            if not self._wait_for_step('22. 전자석 OFF'): return
            if not self._set_magnet(False): return

            # 23. 닫힌 손잡이에서 50mm 후퇴
            final_retreat = tuple(contact[i] + pull_dir[i] * 50.0 for i in range(3))
            if not self._wait_for_step('23. 손잡이에서 50mm 후퇴'): return
            if not self._move_l(*final_retreat, 90.0, -90.0, 0.0): return
            cx, cy, cz = final_retreat

            # 24. 최종 홈 정렬
            if not self._wait_for_step('24. 최종 홈 정렬'): return
            if not self._home(): return

            self.get_logger().info('시퀀스 완료')
            self._wait_for_step('시퀀스 완료')
        except Exception as e:
            self.get_logger().error(f'Sequence error: {e}')
        finally:
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

    # CLI 지원
    if len(sys.argv) > 1 and sys.argv[1].isdigit():
        marker_id = int(sys.argv[1])
        radius = float(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_RADIUS
        node._step_mode = '--step' in sys.argv
        seq_thread = threading.Thread(target=node.run_sequence, args=(marker_id, radius))
        seq_thread.start()

    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
