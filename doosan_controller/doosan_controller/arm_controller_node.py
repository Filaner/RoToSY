"""
Doosan E0509 arm controller node.

Phases implemented:
  1. Servo ON/OFF   → /arm/servo_on   (robot_arm_interfaces/srv/ServoOn)
  2. MoveJ          → /arm/move_j     (robot_arm_interfaces/action/MoveJ)
  2. MoveL          → /arm/move_l     (robot_arm_interfaces/action/MoveL)
  3. WaitDuration   → /arm/wait       (robot_arm_interfaces/srv/WaitDuration)

Publishes:
  /arm/status  (robot_arm_interfaces/msg/RobotStatus)
  /arm/ready   (std_msgs/msg/Bool)

Connects upstream to:
  dsr_bringup2 under namespace <robot_ns> (default "dsr01")
  Services: <ns>/motion/move_joint, <ns>/motion/move_line,
            <ns>/system/servo_off,  <ns>/system/set_robot_control,
            <ns>/system/get_robot_state
  Subscribes: <ns>/joint_states (real-time joint angles)
            <ns>/system/get_current_pose (10 Hz background poll → actual TCP)
"""

import threading
import time
from typing import Optional

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException

from std_msgs.msg import Bool
from sensor_msgs.msg import JointState

# Custom interfaces
from robot_arm_interfaces.action import MoveJ, MoveL
from robot_arm_interfaces.srv import ServoOn, WaitDuration, Jog, Home, Teaching, EStop, Recover
from robot_arm_interfaces.msg import RobotStatus

# Doosan DSR service interfaces
from dsr_msgs2.srv import (
    MoveJoint,
    MoveLine,
    MoveHome,
    MoveStop,
    ServoOff,
    SetRobotControl,
    SetRobotMode,
    GetRobotState,
    GetCurrentPose,
    SetSafeStopResetType,
    Jog as DsrJog,
)

# ─── Doosan state constants (mirrors DRFC.py) ─────────────────────────────────
STATE_INITIALIZING = 0
STATE_STANDBY      = 1
STATE_MOVING       = 2
STATE_SAFE_OFF     = 3
STATE_TEACHING     = 4
STATE_SAFE_STOP    = 5
STATE_RECOVERY     = 8
STATE_SAFE_STOP2   = 9
STATE_SAFE_OFF2    = 10
STATE_NOT_READY    = 15

# SetRobotControl values (DRFC.py CONTROL_*)
CONTROL_ENABLE_OPERATION   = 1   # NOT_READY / INITIALIZING → STANDBY
CONTROL_RESET_SAFET_STOP   = 2   # SAFE_STOP  → STANDBY
CONTROL_SERVO_ON           = 3   # SAFE_OFF   → STANDBY
CONTROL_RECOVERY_SAFE_STOP = 4   # SAFE_STOP2 → RECOVERY
CONTROL_RECOVERY_SAFE_OFF  = 5   # SAFE_OFF2  → RECOVERY
CONTROL_RESET_RECOVERY     = 7   # RECOVERY   → STANDBY

STATE_STR = {
    STATE_INITIALIZING: 'INITIALIZING',
    STATE_STANDBY:      'STANDBY',
    STATE_MOVING:       'MOVING',
    STATE_SAFE_OFF:     'SAFE_OFF',
    STATE_TEACHING:     'TEACHING',
    STATE_SAFE_STOP:    'SAFE_STOP',
    STATE_RECOVERY:     'RECOVERY',
    STATE_SAFE_STOP2:   'SAFE_STOP2',
    STATE_SAFE_OFF2:    'SAFE_OFF2',
    STATE_NOT_READY:    'NOT_READY',
}

SRV_TIMEOUT = 10.0  # seconds to wait for a DSR service to become available


class ArmControllerNode(Node):
    """
    Core controller for the Doosan E0509.

    Uses MultiThreadedExecutor so action execute-callbacks can block on DSR
    service calls without starving other callbacks.  A threading.Lock ensures
    only one motion executes at a time (MoveJ/MoveL are serialised).
    """

    def __init__(self) -> None:
        super().__init__('arm_controller')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('robot_ns',         'dsr01')
        self.declare_parameter('status_rate_hz',   10.0)
        self.declare_parameter('motion_timeout',   60.0)
        self.declare_parameter('servo_on_retries', 3)

        ns             = self.get_parameter('robot_ns').value
        status_hz      = self.get_parameter('status_rate_hz').value
        self._motion_timeout = self.get_parameter('motion_timeout').value
        self._servo_retries  = self.get_parameter('servo_on_retries').value

        self.get_logger().info(f'robot_ns = {ns!r}  status_rate = {status_hz} Hz')

        # ── Callback groups ──────────────────────────────────────────────────
        # ReentrantCallbackGroup allows multiple action execute-callbacks to
        # overlap; the _motion_lock below enforces sequential execution.
        self._action_cbg  = ReentrantCallbackGroup()
        self._service_cbg = MutuallyExclusiveCallbackGroup()
        self._timer_cbg   = MutuallyExclusiveCallbackGroup()
        # E-Stop uses its own Reentrant group so it is never blocked by other callbacks
        self._estop_cbg   = ReentrantCallbackGroup()

        # ── Motion serialisation lock ────────────────────────────────────────
        # Prevents two MoveJ/MoveL goals from executing simultaneously.
        self._motion_lock = threading.Lock()

        # ── Cached robot state (updated by the status timer) ────────────────
        self._robot_state   = STATE_NOT_READY
        self._teaching_mode = False   # True when SetRobotMode(MANUAL) is active

        self._current_joints = [0.0] * 6
        self._current_tcp    = [0.0] * 6

        # ── DSR service clients ──────────────────────────────────────────────
        def _cli(srv_type, name):
            return self.create_client(srv_type, f'{ns}/{name}')

        self._cli_move_joint     = _cli(MoveJoint,       'motion/move_joint')
        self._cli_move_line      = _cli(MoveLine,        'motion/move_line')
        self._cli_move_stop      = _cli(MoveStop,        'motion/move_stop')
        self._cli_servo_off      = _cli(ServoOff,        'system/servo_off')
        self._cli_set_control         = _cli(SetRobotControl,    'system/set_robot_control')
        self._cli_set_mode            = _cli(SetRobotMode,        'system/set_robot_mode')
        self._cli_get_state           = _cli(GetRobotState,       'system/get_robot_state')
        self._cli_safe_stop_reset     = _cli(SetSafeStopResetType,'system/set_safe_stop_reset_type')
        self._cli_get_pose       = _cli(GetCurrentPose,  'system/get_current_pose')
        self._cli_dsr_jog   = _cli(DsrJog,   'motion/jog')
        self._cli_move_home = _cli(MoveHome, 'motion/move_home')

        # ── Publishers ───────────────────────────────────────────────────────
        self._pub_status = self.create_publisher(RobotStatus, '/arm/status', 10)
        self._pub_ready  = self.create_publisher(Bool,        '/arm/ready',  10)
        # /arm/tcp_pose is published by tcp_monitor (TF2-based, real-time during motion)

        # ── DSR joint state subscriber (real-time, rad → deg) ────────────────
        self.create_subscription(
            JointState,
            f'{ns}/joint_states',
            self._joint_state_cb,
            10,
        )

        # ── Service servers ──────────────────────────────────────────────────
        self._srv_servo_on = self.create_service(
            ServoOn, '/arm/servo_on',
            self._servo_on_callback,
            callback_group=self._service_cbg,
        )
        self._srv_recovery = self.create_service(
            Recover, '/arm/safety_recovery',
            self._safety_recovery_callback,
            callback_group=self._service_cbg,
        )
        self._srv_wait = self.create_service(
            WaitDuration, '/arm/wait',
            self._wait_callback,
            callback_group=self._service_cbg,
        )
        self._srv_jog = self.create_service(
            Jog, '/arm/jog',
            self._jog_callback,
            callback_group=self._service_cbg,
        )
        self._srv_home = self.create_service(
            Home, '/arm/home',
            self._home_callback,
            callback_group=self._service_cbg,
        )
        self._srv_teaching = self.create_service(
            Teaching, '/arm/teaching',
            self._teaching_callback,
            callback_group=self._service_cbg,
        )
        self._srv_estop = self.create_service(
            EStop, '/arm/estop',
            self._estop_callback,
            callback_group=self._estop_cbg,
        )

        # ── Action servers ───────────────────────────────────────────────────
        self._as_movej = ActionServer(
            self, MoveJ, '/arm/move_j',
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            execute_callback=self._execute_movej,
            callback_group=self._action_cbg,
        )
        self._as_movel = ActionServer(
            self, MoveL, '/arm/move_l',
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            execute_callback=self._execute_movel,
            callback_group=self._action_cbg,
        )

        # ── Status timer ─────────────────────────────────────────────────────
        self._status_timer = self.create_timer(
            1.0 / status_hz,
            self._publish_status,
            callback_group=self._timer_cbg,
        )

        # ── TCP polling thread (actual pose from DSR, 10 Hz) ─────────────────
        self._tcp_poll_thread = threading.Thread(
            target=self._tcp_poll_loop, daemon=True
        )
        self._tcp_poll_thread.start()

        self.get_logger().info('ArmControllerNode ready.')

    # ══════════════════════════════════════════════════════════════════════════
    # Internal helpers
    # ══════════════════════════════════════════════════════════════════════════

    def _call_service_sync(self, client, request, timeout: float = SRV_TIMEOUT):
        """
        Block the calling thread until the service responds or timeout expires.
        Safe to call from action execute-callbacks (runs in a worker thread
        under MultiThreadedExecutor, so the main spin loop is not blocked).
        """
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(f'Service {client.srv_name!r} unavailable')
            return None

        event    = threading.Event()
        result   = [None]

        def _cb(future):
            result[0] = future.result()
            event.set()

        client.call_async(request).add_done_callback(_cb)
        event.wait(timeout=timeout)
        return result[0]

    def _get_robot_state_now(self) -> int:
        resp = self._call_service_sync(self._cli_get_state, GetRobotState.Request())
        return resp.robot_state if resp else STATE_NOT_READY

    def _tcp_poll_loop(self) -> None:
        """Background daemon: polls system/get_current_pose at 10 Hz.
        Silently skips when the service is not yet available (startup, motion blocking).
        """
        req = GetCurrentPose.Request()
        req.space_type = 1  # ROBOT_SPACE_TASK
        while rclpy.ok():
            try:
                if not self._cli_get_pose.wait_for_service(timeout_sec=0.0):
                    time.sleep(0.5)
                    continue
                event  = threading.Event()
                result = [None]

                def _cb(future, _r=result, _e=event):
                    _r[0] = future.result()
                    _e.set()

                self._cli_get_pose.call_async(req).add_done_callback(_cb)
                event.wait(timeout=0.3)
                resp = result[0]
                if resp and resp.success:
                    self._current_tcp = list(resp.pos)
            except Exception:
                pass
            time.sleep(0.1)

    def _joint_state_cb(self, msg: JointState) -> None:
        """Update joint angle cache from DSR's real-time joint_states topic.
        Maps by name because DSR publishes in non-sequential order (J1,J2,J4,J5,J3,J6).
        """
        if len(msg.position) < 6 or len(msg.name) < 6:
            return
        name_to_rad = dict(zip(msg.name, msg.position))
        try:
            joints_rad = [
                name_to_rad['joint_1'],
                name_to_rad['joint_2'],
                name_to_rad['joint_3'],
                name_to_rad['joint_4'],
                name_to_rad['joint_5'],
                name_to_rad['joint_6'],
            ]
        except KeyError:
            return
        self._current_joints = [math.degrees(r) for r in joints_rad]

    def _is_moving(self) -> bool:
        return self._robot_state == STATE_MOVING

    def _is_ready(self) -> bool:
        return self._robot_state in (STATE_STANDBY, STATE_MOVING)

    # ══════════════════════════════════════════════════════════════════════════
    # Status timer
    # ══════════════════════════════════════════════════════════════════════════

    def _publish_status(self) -> None:
        # During motion the feedback thread owns state polling to avoid
        # concurrent GetRobotState calls that corrupt each other's responses.
        if self._motion_lock.locked():
            state = self._robot_state
        else:
            polled = self._get_robot_state_now()
            if polled is not None:
                self._robot_state = polled
            state = self._robot_state

        msg = RobotStatus()
        msg.header.stamp       = self.get_clock().now().to_msg()
        msg.robot_state        = state
        msg.robot_state_str    = STATE_STR.get(state, str(state))
        msg.servo_on        = (state in (STATE_STANDBY, STATE_MOVING))
        msg.is_moving       = (state == STATE_MOVING)
        msg.teaching_mode   = self._teaching_mode

        msg.current_joints_deg = self._current_joints
        msg.current_tcp        = self._current_tcp
        self._pub_status.publish(msg)

        ready_msg      = Bool()
        ready_msg.data = (state == STATE_STANDBY)
        self._pub_ready.publish(ready_msg)

    # ══════════════════════════════════════════════════════════════════════════
    # Phase 1 — Servo ON/OFF
    # ══════════════════════════════════════════════════════════════════════════

    def _servo_on_callback(self, request: ServoOn.Request,
                           response: ServoOn.Response) -> ServoOn.Response:
        if request.enable:
            response.success, response.message = self._do_servo_on()
        else:
            response.success, response.message = self._do_servo_off(request.stop_type)

        response.robot_state_after = self._get_robot_state_now()
        return response

    def _do_servo_on(self):
        """
        SAFE_OFF → STANDBY 전환 (서보 ON).

        안전 정지(SAFE_STOP / SAFE_STOP2 / SAFE_OFF2 / RECOVERY) 상태에서는
        서보 ON을 거부하고 /arm/safety_recovery 서비스를 사용하도록 안내.
        """
        log   = self.get_logger()
        state = self._get_robot_state_now()
        log.info(f'[ServoON] 현재 상태: {STATE_STR.get(state, state)} ({state})')

        if state == STATE_STANDBY:
            return True, 'Already in STANDBY'

        if state == STATE_MOVING:
            return True, 'Robot is moving (already servo-on)'

        # 안전 정지 계열 → 서보 ON 불가, 안전복구 버튼 유도
        if state in (STATE_SAFE_STOP, STATE_SAFE_STOP2, STATE_SAFE_OFF2, STATE_RECOVERY):
            msg = (
                f'서보 ON 불가: 현재 상태 {STATE_STR.get(state, state)} — '
                '⚡ 안전 복구 버튼을 사용하세요'
            )
            log.warn(msg)
            return False, msg

        if state not in (STATE_SAFE_OFF, STATE_NOT_READY, STATE_INITIALIZING):
            msg = f'서보 ON 불가 상태: {STATE_STR.get(state, state)}'
            log.warn(msg)
            return False, msg

        return self._recover_safe_off()

    def _set_robot_control(self, control_val: int, label: str) -> bool:
        """SetRobotControl 서비스 호출 헬퍼."""
        req = SetRobotControl.Request()
        req.robot_control = control_val
        resp = self._call_service_sync(self._cli_set_control, req, timeout=5.0)
        if resp is None:
            self.get_logger().error(f'[ServoON] SetRobotControl({label}) 응답 없음')
            return False
        if not resp.success:
            self.get_logger().warn(f'[ServoON] SetRobotControl({label}) 실패 — 계속 진행')
        else:
            self.get_logger().info(f'[ServoON] SetRobotControl({label}) 전송 완료')
        return True

    def _wait_for_state(self, target: int, timeout_sec: float = 20.0) -> int:
        """target 상태가 될 때까지 최대 timeout_sec 동안 폴링. 실제 상태 반환."""
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            s = self._get_robot_state_now()
            if s == target:
                return s
            time.sleep(0.5)
        return self._get_robot_state_now()

    def _recover_safe_off(self):
        """SAFE_OFF → AUTONOMOUS 모드 + SERVO_ON → STANDBY."""
        log = self.get_logger()
        log.info('[ServoON] SAFE_OFF 복구 시작')

        mode_req            = SetRobotMode.Request()
        mode_req.robot_mode = 1   # ROBOT_MODE_AUTONOMOUS
        mode_resp = self._call_service_sync(self._cli_set_mode, mode_req, timeout=5.0)
        if mode_resp is None:
            log.warn('[ServoON] SetRobotMode 응답 없음 — 계속 진행')
        elif mode_resp.success:
            log.info('[ServoON] 로봇 모드 → AUTONOMOUS')
        else:
            log.warn('[ServoON] SetRobotMode 실패 — 계속 진행 (이미 AUTO일 수 있음)')

        self._set_robot_control(CONTROL_SERVO_ON, 'SERVO_ON')

        final = self._wait_for_state(STATE_STANDBY, timeout_sec=15.0)
        if final == STATE_STANDBY:
            self._robot_state = STATE_STANDBY
            log.info('[ServoON] SAFE_OFF 복구 성공 → STANDBY')
            return True, 'Servo ON successful'
        return False, f'SAFE_OFF 복구 실패 — 최종 상태: {STATE_STR.get(final, final)}'

    def _recover_safe_stop(self):
        """SAFE_STOP → SetSafeStopResetType(0) + CONTROL_RESET_SAFET_STOP → STANDBY."""
        log = self.get_logger()
        log.info('[ServoON] SAFE_STOP 복구 시작 (SetSafeStopResetType + CONTROL_RESET_SAFET_STOP)')

        rst_req             = SetSafeStopResetType.Request()
        rst_req.reset_type  = 0   # SAFE_STOP_RESET_TYPE_DEFAULT = PROGRAM_STOP
        rst_resp = self._call_service_sync(self._cli_safe_stop_reset, rst_req, timeout=5.0)
        if rst_resp is None:
            log.warn('[ServoON] SetSafeStopResetType 응답 없음 — 계속 진행')
        else:
            log.info('[ServoON] SetSafeStopResetType(DEFAULT) 완료')

        self._set_robot_control(CONTROL_RESET_SAFET_STOP, 'RESET_SAFE_STOP')

        final = self._wait_for_state(STATE_STANDBY, timeout_sec=15.0)
        if final == STATE_STANDBY:
            self._robot_state = STATE_STANDBY
            log.info('[ServoON] SAFE_STOP 복구 성공 → STANDBY')
            return True, 'SAFE_STOP recovery successful'
        return False, f'SAFE_STOP 복구 실패 — 최종 상태: {STATE_STR.get(final, final)} (비상정지 버튼/안전 회로 확인 필요)'

    def _recover_deep(self, control_val: int, label: str):
        """
        SAFE_STOP2 / SAFE_OFF2 심층 복구:
          Step 1: CONTROL_RECOVERY_SAFE_STOP(4) 또는 CONTROL_RECOVERY_SAFE_OFF(5)
                  → 상태가 RECOVERY(8)로 전환될 때까지 대기
          Step 2: CONTROL_RESET_RECOVERY(7) → STANDBY
        """
        log = self.get_logger()
        log.info(f'[ServoON] {label} 심층 복구 시작')

        self._set_robot_control(control_val, f'RECOVERY_{label}')

        mid = self._wait_for_state(STATE_RECOVERY, timeout_sec=20.0)
        if mid != STATE_RECOVERY:
            msg = f'{label} 복구 실패 — RECOVERY 진입 못함 (상태: {STATE_STR.get(mid, mid)})'
            log.error(f'[ServoON] {msg}')
            return False, msg
        log.info(f'[ServoON] {label} → RECOVERY 진입 성공, RESET_RECOVERY 전송')

        return self._reset_recovery()

    def _reset_recovery(self):
        """RECOVERY → CONTROL_RESET_RECOVERY(7) → STANDBY."""
        log = self.get_logger()
        self._set_robot_control(CONTROL_RESET_RECOVERY, 'RESET_RECOVERY')

        final = self._wait_for_state(STATE_STANDBY, timeout_sec=20.0)
        if final == STATE_STANDBY:
            self._robot_state = STATE_STANDBY
            log.info('[ServoON] RECOVERY 복구 성공 → STANDBY')
            return True, 'Recovery successful → STANDBY'
        return False, f'RECOVERY 복구 실패 — 최종 상태: {STATE_STR.get(final, final)}'

    def _do_servo_off(self, stop_type: int):
        req           = ServoOff.Request()
        req.stop_type = stop_type
        resp          = self._call_service_sync(self._cli_servo_off, req)
        if resp is None:
            return False, 'servo_off service call failed (timeout)'
        return resp.success, 'Servo OFF successful' if resp.success else 'Servo OFF failed'

    def _safety_recovery_callback(
        self,
        request: Recover.Request,
        response: Recover.Response,
    ) -> Recover.Response:
        """
        /arm/safety_recovery 서비스 핸들러.

        현재 로봇 상태에 따라 적절한 복구 시퀀스를 수행해 STANDBY로 전환.
          SAFE_STOP  → SetSafeStopResetType(0) + CONTROL_RESET_SAFET_STOP(2)
          SAFE_STOP2 → CONTROL_RECOVERY_SAFE_STOP(4) → RECOVERY → CONTROL_RESET_RECOVERY(7)
          SAFE_OFF2  → CONTROL_RECOVERY_SAFE_OFF(5)  → RECOVERY → CONTROL_RESET_RECOVERY(7)
          RECOVERY   → CONTROL_RESET_RECOVERY(7)
        """
        log   = self.get_logger()
        state = self._get_robot_state_now()
        log.info(f'[SafetyRecovery] 요청 수신 — 현재 상태: {STATE_STR.get(state, state)} ({state})')

        if state in (STATE_STANDBY, STATE_MOVING):
            response.success = True
            response.message = f'복구 불필요 — 이미 정상 상태: {STATE_STR.get(state, state)}'
            response.robot_state_after = state
            return response

        if state == STATE_SAFE_STOP:
            ok, msg = self._recover_safe_stop()
        elif state == STATE_SAFE_STOP2:
            ok, msg = self._recover_deep(CONTROL_RECOVERY_SAFE_STOP, 'SAFE_STOP2')
        elif state == STATE_SAFE_OFF2:
            ok, msg = self._recover_deep(CONTROL_RECOVERY_SAFE_OFF, 'SAFE_OFF2')
        elif state == STATE_RECOVERY:
            ok, msg = self._reset_recovery()
        else:
            ok  = False
            msg = (
                f'안전복구 대상 아님: 현재 상태 {STATE_STR.get(state, state)} — '
                '비상정지 해제 또는 컨트롤러 재시작 필요'
            )
            log.error(f'[SafetyRecovery] {msg}')

        response.success = ok
        response.message = msg
        response.robot_state_after = self._get_robot_state_now()
        return response

    def _jog_callback(self, request: Jog.Request,
                      response: Jog.Response) -> Jog.Response:
        """
        Handles joint jogging commands.
        """
        self.get_logger().info(f'JOG RECV: joint={request.joint_index} speed={request.speed}')
        state = self._get_robot_state_now()
        if state not in (STATE_STANDBY, STATE_MOVING):
            response.success = False
            response.message = f'Cannot jog in state: {STATE_STR.get(state, state)}'
            self.get_logger().warn(response.message)
            return response

        if not (0 <= request.joint_index <= 5):
            response.success = False
            response.message = f'Invalid joint index: {request.joint_index}'
            self.get_logger().error(response.message)
            return response

        dsr_req = DsrJog.Request()
        dsr_req.jog_axis       = request.joint_index
        dsr_req.move_reference = 0
        dsr_req.speed          = float(request.speed)

        self.get_logger().info(f'Calling DSR Jog: axis={dsr_req.jog_axis} speed={dsr_req.speed}')
        resp = self._call_service_sync(self._cli_dsr_jog, dsr_req)
        
        if resp is None:
            response.success = False
            response.message = 'DSR jog service timeout'
            self.get_logger().error(response.message)
        else:
            response.success = resp.success
            response.message = 'OK' if resp.success else 'DSR jog rejected'
            self.get_logger().info(f'DSR Jog Response: success={resp.success}')
        
        return response

    def _home_callback(self, request: Home.Request,
                       response: Home.Response) -> Home.Response:
        """
        Handles move_home commands with custom angles: [0, 0, 90, 0, 90, 0]
        """
        state = self._get_robot_state_now()
        if state not in (STATE_STANDBY, STATE_MOVING):
            response.success = False
            response.message = f'Cannot move home in state: {STATE_STR.get(state, state)}'
            return response

        acquired = self._motion_lock.acquire(timeout=2.0)
        if not acquired:
            response.success = False
            response.message = 'Robot is busy with another motion'
            return response

        try:
            # Custom Home Pose: [0, 0, 90, 0, 90, 0]
            req = MoveJoint.Request()
            req.pos = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
            req.vel = 30.0
            req.acc = 60.0
            req.time = 0.0
            req.radius = 0.0
            req.mode = 0 # Absolute
            req.sync_type = 0 # SYNC

            self._robot_state = STATE_MOVING
            resp = self._call_service_sync(self._cli_move_joint, req, timeout=30.0)
            self._robot_state = STATE_STANDBY
            
            if resp is None:
                response.success = False
                response.message = 'move_joint service timeout'
            else:
                response.success = resp.success
                response.message = 'Home move complete' if resp.success else 'move_joint failed'
            return response
        finally:
            self._motion_lock.release()

    def _teaching_callback(self, request: Teaching.Request,
                           response: Teaching.Response) -> Teaching.Response:
        """
        enable=True  → SetRobotMode(MANUAL=0)      : 직접 교시 모드 진입
        enable=False → SetRobotMode(AUTONOMOUS=1)  : 자율 운전 모드 복귀
        """
        mode_req            = SetRobotMode.Request()
        mode_req.robot_mode = 0 if request.enable else 1

        resp = self._call_service_sync(self._cli_set_mode, mode_req, timeout=5.0)
        if resp is None:
            response.success = False
            response.message = 'SetRobotMode service timeout'
        else:
            response.success = resp.success
            if resp.success:
                self._teaching_mode = request.enable
                response.message = '직접 교시 모드 활성화' if request.enable else '직접 교시 모드 해제'
            else:
                response.message = 'SetRobotMode 실패'

        response.robot_state_after = self._get_robot_state_now()
        return response

    # ══════════════════════════════════════════════════════════════════════════
    # Action goal / cancel callbacks (shared by MoveJ and MoveL)
    # ══════════════════════════════════════════════════════════════════════════

    def _goal_callback(self, goal_request):
        state = self._get_robot_state_now()
        # Only accept motion goals if the robot is already in STANDBY or MOVING (Servo ON)
        if state in (STATE_STANDBY, STATE_MOVING):
            return GoalResponse.ACCEPT
        
        self.get_logger().warn(
            f'Rejecting goal: state={STATE_STR.get(state, state)} — '
            'Servo is OFF. Please turn it ON via the interface first.'
        )
        return GoalResponse.REJECT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel requested — will stop after current motion step')
        return CancelResponse.ACCEPT

    def _exit_teaching_if_active(self) -> None:
        """동작 명령 실행 전 MANUAL 모드 해제 후 AUTONOMOUS 복귀."""
        if not self._teaching_mode:
            return
        self.get_logger().info('Teaching mode 자동 해제 → AUTONOMOUS 복귀')
        mode_req            = SetRobotMode.Request()
        mode_req.robot_mode = 1   # AUTONOMOUS
        self._call_service_sync(self._cli_set_mode, mode_req, timeout=5.0)
        self._teaching_mode = False

    # ══════════════════════════════════════════════════════════════════════════
    # Phase 2 — MoveJ execute callback
    # ══════════════════════════════════════════════════════════════════════════

    def _execute_movej(self, goal_handle) -> MoveJ.Result:
        g   = goal_handle.request
        log = self.get_logger()
        
        # 기본값 보정 (0일 경우 이동 안 함 방지)
        vel = g.velocity_deg_s if g.velocity_deg_s > 0 else 20.0
        acc = g.acceleration_deg_s2 if g.acceleration_deg_s2 > 0 else 40.0

        log.info(f'MoveJ goal: joints={list(g.joint_angles_deg)} vel={vel} acc={acc}')

        self._exit_teaching_if_active()

        result = MoveJ.Result()
        t_start = time.time()

        acquired = self._motion_lock.acquire(timeout=self._motion_timeout)
        if not acquired:
            msg = 'Could not acquire motion lock'
            log.error(msg)
            result.success = False
            result.message = msg
            goal_handle.abort()
            return result

        try:
            req            = MoveJoint.Request()
            req.pos        = list(g.joint_angles_deg)
            req.vel        = vel
            req.acc        = acc
            req.time       = 0.0
            req.radius     = g.blend_radius_mm
            req.mode       = 1 if g.relative else 0
            req.blend_type = 0

            # blend_radius > 0 → ASYNC(비동기): DSR이 관절 이동을 시작하자마자 반환.
            # 이후 바로 MoveL을 전송하면 DSR이 내부적으로 블렌딩 처리.
            # blend_radius == 0 → SYNC(동기): 완료까지 블로킹 (기존 동작 유지).
            blend_mode = g.blend_radius_mm > 0.0
            req.sync_type = 1 if blend_mode else 0

            self._robot_state = STATE_MOVING

            resp = self._call_service_sync(self._cli_move_joint, req, timeout=self._motion_timeout)

            if resp is None or not resp.success:
                result.success = False
                result.message = 'move_joint failed'
                goal_handle.abort()
                return result

            elapsed = time.time() - t_start

            if blend_mode:
                # ASYNC 모드: DSR이 동작 중이므로 lock을 즉시 해제해
                # 다음 MoveL이 바로 큐에 들어갈 수 있도록 함
                log.info(f'MoveJ blending (radius={g.blend_radius_mm}°) — MoveL 즉시 전송 가능')
                self._motion_lock.release()
                goal_handle.succeed()
                result.success = True
                result.message = f'blending radius={g.blend_radius_mm}'
                result.execution_time_sec = elapsed
                return result

            # SYNC 모드: 완전 종료 후 반환
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = 'Cancelled'
                result.execution_time_sec = elapsed
                return result

            self._robot_state = STATE_STANDBY
            log.info(f'MoveJ completed in {elapsed:.2f} s')
            goal_handle.succeed()
            result.success = True
            result.message = 'Success'
            result.execution_time_sec = elapsed
            return result
        finally:
            # blend_mode에서는 위에서 이미 release했으므로 중복 방지
            if self._motion_lock.locked():
                self._motion_lock.release()

    # ══════════════════════════════════════════════════════════════════════════
    # Phase 2 — MoveL execute callback
    # ══════════════════════════════════════════════════════════════════════════

    def _execute_movel(self, goal_handle) -> MoveL.Result:
        g   = goal_handle.request
        log = self.get_logger()
        
        # 기본값 보정
        l_vel = g.linear_velocity_mm_s if g.linear_velocity_mm_s > 0 else 30.0
        a_vel = g.angular_velocity_deg_s if g.angular_velocity_deg_s > 0 else 30.0
        l_acc = g.linear_accel_mm_s2 if g.linear_accel_mm_s2 > 0 else 60.0
        a_acc = g.angular_accel_deg_s2 if g.angular_accel_deg_s2 > 0 else 60.0

        log.info(f'MoveL goal: pos=({g.x:.1f},{g.y:.1f},{g.z:.1f}) vel={l_vel}')

        self._exit_teaching_if_active()

        result  = MoveL.Result()
        t_start = time.time()

        acquired = self._motion_lock.acquire(timeout=self._motion_timeout)
        if not acquired:
            msg = 'Could not acquire motion lock'
            log.error(msg)
            result.success = False
            result.message = msg
            goal_handle.abort()
            return result

        try:
            req            = MoveLine.Request()
            req.pos        = [g.x, g.y, g.z, g.rx, g.ry, g.rz]
            req.vel        = [l_vel, a_vel]
            req.acc        = [l_acc, a_acc]
            req.time       = 0.0
            req.radius     = g.blend_radius_mm
            req.ref        = g.reference_frame
            req.mode       = 1 if g.relative else 0
            req.blend_type = 0
            req.sync_type  = 0   # SYNC: 완료까지 블로킹 → 동작 순서 보장

            self._robot_state = STATE_MOVING

            done_event = threading.Event()
            def feedback_thread():
                while not done_event.is_set():
                    if goal_handle.is_cancel_requested:
                        self._request_move_stop()
                        break
                    fb = MoveL.Feedback()
                    fb.current_x   = self._current_tcp[0]
                    fb.current_y   = self._current_tcp[1]
                    fb.current_z   = self._current_tcp[2]
                    fb.current_rx  = self._current_tcp[3]
                    fb.current_ry  = self._current_tcp[4]
                    fb.current_rz  = self._current_tcp[5]
                    fb.robot_state = 'MOVING'
                    goal_handle.publish_feedback(fb)
                    time.sleep(0.1)

            fb_t = threading.Thread(target=feedback_thread, daemon=True)
            fb_t.start()

            resp = self._call_service_sync(self._cli_move_line, req, timeout=self._motion_timeout)
            done_event.set()
            fb_t.join()

            elapsed = time.time() - t_start
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = 'Cancelled'
                result.execution_time_sec = elapsed
                return result

            if resp is None or not resp.success:
                result.success = False
                result.message = 'move_line failed'
                goal_handle.abort()
                return result

            self._robot_state = STATE_STANDBY
            log.info(f'MoveL completed in {elapsed:.2f} s')
            goal_handle.succeed()
            result.success = True
            result.message = 'Success'
            result.execution_time_sec = elapsed
            return result
        finally:
            self._motion_lock.release()


    # ══════════════════════════════════════════════════════════════════════════
    # Shared polling loop (MoveJ + MoveL)
    # ══════════════════════════════════════════════════════════════════════════

    def _poll_until_done(self, goal_handle, label: str, t_start: float):
        """
        Spin at 10 Hz until the robot stops MOVING or a cancel/timeout occurs.
        Publishes action feedback at every iteration.
        Returns the appropriate Result object.
        """
        log      = self.get_logger()
        timeout  = self._motion_timeout
        feedback = MoveJ.Feedback() if label == 'MoveJ' else MoveL.Feedback()

        # Brief startup delay: DSR ASYNC call may not have started the motion yet
        time.sleep(0.15)

        while True:
            elapsed = time.time() - t_start

            # Refresh pose cache for feedback
            self._update_pose_cache()
            state = self._get_robot_state_now()
            self._robot_state = state

            # ── Cancellation ────────────────────────────────────────────────
            if goal_handle.is_cancel_requested:
                log.info(f'{label}: cancel requested — stopping robot')
                self._request_move_stop()
                goal_handle.canceled()
                r = MoveJ.Result() if label == 'MoveJ' else MoveL.Result()
                r.success             = False
                r.message             = 'Cancelled by client'
                r.execution_time_sec  = elapsed
                return r

            # ── Timeout ─────────────────────────────────────────────────────
            if elapsed > timeout:
                log.error(f'{label}: motion timeout after {timeout:.1f} s')
                self._request_move_stop()
                goal_handle.abort()
                r = MoveJ.Result() if label == 'MoveJ' else MoveL.Result()
                r.success             = False
                r.message             = f'Timeout after {timeout:.1f} s'
                r.execution_time_sec  = elapsed
                return r

            # ── Publish feedback ────────────────────────────────────────────
            if label == 'MoveJ':
                feedback.current_joints_deg = self._current_joints
                feedback.robot_state        = STATE_STR.get(state, str(state))
                # Rough progress: time-based heuristic when state is MOVING
                feedback.progress_pct = min(elapsed / max(timeout * 0.5, 1.0) * 100, 99.0)
            else:
                tcp = self._current_tcp
                feedback.current_x   = tcp[0]
                feedback.current_y   = tcp[1]
                feedback.current_z   = tcp[2]
                feedback.current_rx  = tcp[3]
                feedback.current_ry  = tcp[4]
                feedback.current_rz  = tcp[5]
                feedback.robot_state = STATE_STR.get(state, str(state))
                feedback.progress_pct = min(elapsed / max(timeout * 0.5, 1.0) * 100, 99.0)

            goal_handle.publish_feedback(feedback)

            # ── Done? ────────────────────────────────────────────────────────
            if state == STATE_STANDBY:
                log.info(f'{label}: completed in {elapsed:.2f} s')
                goal_handle.succeed()
                r = MoveJ.Result() if label == 'MoveJ' else MoveL.Result()
                r.success            = True
                r.message            = 'Motion completed'
                r.execution_time_sec = elapsed
                return r

            time.sleep(0.1)   # 10 Hz

    def _request_move_stop(self) -> None:
        """Request the DSR driver to stop the current motion immediately."""
        req           = MoveStop.Request()
        req.stop_mode = 0   # STOP_TYPE_QUICK
        self._call_service_sync(self._cli_move_stop, req, timeout=3.0)

    def _estop_callback(self, request: EStop.Request,
                        response: EStop.Response) -> EStop.Response:
        """비상정지: 즉시 동작 중단 → 서보 OFF. 어떤 상태에서도 호출 가능."""
        self.get_logger().warn('!!! EMERGENCY STOP !!!')
        self._request_move_stop()
        ok, msg = self._do_servo_off(stop_type=0)   # QUICK_STO
        self._teaching_mode = False
        self._robot_state   = STATE_SAFE_OFF
        response.success = ok
        response.message = msg
        return response

    # ══════════════════════════════════════════════════════════════════════════
    # Phase 3 — WaitDuration service
    # ══════════════════════════════════════════════════════════════════════════

    def _wait_callback(self, request: WaitDuration.Request,
                       response: WaitDuration.Response) -> WaitDuration.Response:
        """
        Blocks the calling service thread for request.duration_sec without
        freezing the ROS2 executor (MultiThreadedExecutor keeps spinning).
        """
        duration = max(0.0, request.duration_sec)
        self.get_logger().info(f'WaitDuration: sleeping {duration:.3f} s')
        t0 = time.time()
        # threading.Event.wait() is interruptible and does not block the GIL
        threading.Event().wait(timeout=duration)
        actual = time.time() - t0
        response.success             = True
        response.actual_duration_sec = actual
        return response


# ──────────────────────────────────────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmControllerNode()

    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
