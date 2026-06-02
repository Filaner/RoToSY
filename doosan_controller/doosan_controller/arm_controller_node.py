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
    Jog as DsrJog,
    SetSafeStopResetType,
    GetLastAlarm,
)

# ─── Doosan state constants (mirrors DRFC.py) ─────────────────────────────────
STATE_INITIALIZING = 0
STATE_STANDBY      = 1
STATE_MOVING       = 2
STATE_SAFE_OFF     = 3
STATE_TEACHING     = 4
STATE_SAFE_STOP    = 5
STATE_NOT_READY    = 15

CONTROL_ENABLE_OPERATION  = 1   # NOT_READY / INITIALIZING → STANDBY
CONTROL_RESET_SAFET_STOP  = 2   # SAFE_STOP 리셋
CONTROL_RESET_SAFET_OFF   = 3   # SAFE_OFF → STANDBY
CONTROL_SERVO_ON          = 3   # alias

STATE_STR = {
    STATE_INITIALIZING: 'INITIALIZING',
    STATE_STANDBY:      'STANDBY',
    STATE_MOVING:       'MOVING',
    STATE_SAFE_OFF:     'SAFE_OFF',
    STATE_TEACHING:     'TEACHING',
    STATE_SAFE_STOP:    'SAFE_STOP',
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
        self.declare_parameter('robot_ns',             'dsr01')
        self.declare_parameter('status_rate_hz',       10.0)
        self.declare_parameter('motion_timeout',       60.0)   # max seconds per move
        self.declare_parameter('servo_on_retries',     3)
        self.declare_parameter('auto_recovery_enabled', True)

        ns             = self.get_parameter('robot_ns').value
        status_hz      = self.get_parameter('status_rate_hz').value
        self._motion_timeout        = self.get_parameter('motion_timeout').value
        self._servo_retries         = self.get_parameter('servo_on_retries').value
        self._auto_recovery_enabled = self.get_parameter('auto_recovery_enabled').value

        self.get_logger().info(
            f'robot_ns = {ns!r}  status_rate = {status_hz} Hz  '
            f'auto_recovery = {self._auto_recovery_enabled}'
        )

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
        self._robot_state    = STATE_NOT_READY
        self._prev_robot_state = STATE_NOT_READY   # 상태 전환 감지용
        self._teaching_mode  = False   # True when SetRobotMode(MANUAL) is active

        self._current_joints = [0.0] * 6
        self._current_tcp    = [0.0] * 6

        # ── 안전복구 이벤트 / 스레드 ─────────────────────────────────────
        self._recovery_event        = threading.Event()
        self._intentional_servo_off = False   # 사용자가 의도적으로 Servo OFF 한 경우 True
        self._recovery_thread = threading.Thread(
            target=self._auto_recovery_loop, daemon=True
        )

        # ── DSR service clients ──────────────────────────────────────────────
        def _cli(srv_type, name):
            return self.create_client(srv_type, f'{ns}/{name}')

        self._cli_move_joint     = _cli(MoveJoint,       'motion/move_joint')
        self._cli_move_line      = _cli(MoveLine,        'motion/move_line')
        self._cli_move_stop      = _cli(MoveStop,        'motion/move_stop')
        self._cli_servo_off      = _cli(ServoOff,        'system/servo_off')
        self._cli_set_control    = _cli(SetRobotControl, 'system/set_robot_control')
        self._cli_set_mode       = _cli(SetRobotMode,    'system/set_robot_mode')
        self._cli_get_state      = _cli(GetRobotState,   'system/get_robot_state')
        self._cli_get_pose       = _cli(GetCurrentPose,  'system/get_current_pose')
        self._cli_dsr_jog             = _cli(DsrJog,               'motion/jog')
        self._cli_move_home           = _cli(MoveHome,             'motion/move_home')
        self._cli_set_safe_stop_reset = _cli(SetSafeStopResetType, 'system/set_safe_stop_reset_type')
        self._cli_get_last_alarm      = _cli(GetLastAlarm,          'system/get_last_alarm')

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
            callback_group=self._estop_cbg,   # 전용 Reentrant: 항상 즉시 처리
        )
        self._srv_recover = self.create_service(
            Recover, '/arm/recover',
            self._recover_callback,
            callback_group=self._estop_cbg,   # E-Stop과 동일 그룹: 어떤 상태에서도 즉시 처리
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

        # ── 자동복구 스레드 ──────────────────────────────────────────────────
        self._recovery_thread.start()

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

        # SAFE_STOP / SAFE_OFF 전환 감지 → 자동복구 트리거
        # 반드시 정상 운전 중(STANDBY/MOVING)에서 폴트 상태로 전환된 경우에만 발동.
        # 기동 시 NOT_READY → SAFE_OFF 전환은 정상 시퀀스이므로 무시.
        # 사용자가 의도적으로 Servo OFF 한 경우에는 자동복구 제외.
        if (self._prev_robot_state in (STATE_STANDBY, STATE_MOVING) and
                state in (STATE_SAFE_STOP, STATE_SAFE_OFF) and
                self._auto_recovery_enabled and
                not self._intentional_servo_off):
            self.get_logger().warn(
                f'[SafeRecovery] 상태 전환 감지: '
                f'{STATE_STR.get(self._prev_robot_state, self._prev_robot_state)} '
                f'→ {STATE_STR.get(state, state)} — 자동복구 예약'
            )
            self._recovery_event.set()
        self._intentional_servo_off = False   # 매 주기 초기화
        self._prev_robot_state = state

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
            self._intentional_servo_off = True   # 의도적 OFF — 자동복구 억제
            response.success, response.message = self._do_servo_off(request.stop_type)

        response.robot_state_after = self._get_robot_state_now()
        return response

    def _do_servo_on(self):
        """
        SAFE_OFF → STANDBY 전환 시퀀스:
          1. SetRobotMode(AUTONOMOUS)  — MANUAL 모드 해제
          2. SetRobotControl(SERVO_ON) — 서보 전원 인가
          3. STANDBY 대기 (최대 15초)
        """
        state = self._get_robot_state_now()

        if state == STATE_STANDBY:
            return True, 'Already in STANDBY'

        if state == STATE_SAFE_STOP:
            msg = '[SAFE_STOP] 안전 조건 해제 필요 — 로봇 컨트롤러를 재시작하세요.'
            self.get_logger().warn(msg)
            return False, msg

        if state not in (STATE_SAFE_OFF, STATE_NOT_READY, STATE_INITIALIZING):
            msg = f'서보 ON 불가 상태: {STATE_STR.get(state, state)}'
            self.get_logger().warn(msg)
            return False, msg

        log = self.get_logger()
        log.info(f'Servo ON 시퀀스 시작 (현재 상태: {STATE_STR.get(state, state)})')

        # ── Step 1: AUTONOMOUS 모드로 전환 ────────────────────────────────
        # MANUAL 모드에서는 원격 서보 ON이 거부됨 → 먼저 AUTO 모드로 전환
        mode_req              = SetRobotMode.Request()
        mode_req.robot_mode   = 1   # ROBOT_MODE_AUTONOMOUS
        mode_resp = self._call_service_sync(self._cli_set_mode, mode_req, timeout=5.0)
        if mode_resp is None:
            log.warn('SetRobotMode 서비스 응답 없음 — 계속 진행')
        elif mode_resp.success:
            log.info('로봇 모드 → AUTONOMOUS')
        else:
            log.warn('SetRobotMode 실패 — 계속 진행 (이미 AUTO일 수 있음)')

        # ── Step 2: 서보 ON ───────────────────────────────────────────────
        ctrl_req              = SetRobotControl.Request()
        ctrl_req.robot_control = CONTROL_SERVO_ON
        self._call_service_sync(self._cli_set_control, ctrl_req, timeout=5.0)
        log.info('SetRobotControl(SERVO_ON) 전송 완료')

        # ── Step 3: STANDBY 대기 (최대 15초) ─────────────────────────────
        deadline = time.time() + 15.0
        while time.time() < deadline:
            s = self._get_robot_state_now()
            if s == STATE_STANDBY:
                self._robot_state = STATE_STANDBY
                log.info('Servo ON 성공 → STANDBY (하얀불 확인)')
                return True, 'Servo ON successful'
            time.sleep(0.5)

        final_state = self._get_robot_state_now()
        msg = (
            f'Servo ON 실패 — 최종 상태: {STATE_STR.get(final_state, final_state)}\n'
            '  가능한 원인:\n'
            '  ① 로봇 컨트롤러 웹UI(http://110.120.1.52)에서 External Control 활성화 필요\n'
            '  ② 안전 회로 이상 — 비상정지 버튼 상태 확인\n'
            '  ③ dsr_hardware2 초기화 실패 — launch 재시작'
        )
        log.error(msg)
        return False, msg

    def _do_servo_off(self, stop_type: int):
        req           = ServoOff.Request()
        req.stop_type = stop_type
        resp          = self._call_service_sync(self._cli_servo_off, req)
        if resp is None:
            return False, 'servo_off service call failed (timeout)'
        return resp.success, 'Servo OFF successful' if resp.success else 'Servo OFF failed'

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
            req.sync_type  = 0   # SYNC: 완료까지 블로킹 → 동작 순서 보장

            self._robot_state = STATE_MOVING

            done_event = threading.Event()
            def feedback_thread():
                while not done_event.is_set():
                    if goal_handle.is_cancel_requested:
                        self._request_move_stop()
                        break
                    fb = MoveJ.Feedback()
                    fb.current_joints_deg = self._current_joints
                    fb.robot_state        = 'MOVING'
                    goal_handle.publish_feedback(fb)
                    time.sleep(0.1)

            fb_t = threading.Thread(target=feedback_thread, daemon=True)
            fb_t.start()

            resp = self._call_service_sync(self._cli_move_joint, req, timeout=self._motion_timeout)
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
                result.message = 'move_joint failed'
                goal_handle.abort()
                return result

            self._robot_state = STATE_STANDBY
            log.info(f'MoveJ completed in {elapsed:.2f} s')
            goal_handle.succeed()
            result.success = True
            result.message = 'Success'
            result.execution_time_sec = elapsed
            return result
        finally:
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
    # Safety recovery
    # ══════════════════════════════════════════════════════════════════════════

    def _auto_recovery_loop(self) -> None:
        """SAFE_STOP/SAFE_OFF 전환 시 자동복구 시퀀스를 실행하는 백그라운드 스레드."""
        while rclpy.ok():
            self._recovery_event.wait()
            self._recovery_event.clear()
            if not self._auto_recovery_enabled:
                continue
            time.sleep(1.5)   # 상태 안정화 대기
            self.get_logger().warn('=== [SafeRecovery] 자동 안전복구 시퀀스 실행 ===')
            success, msg = self._do_recovery(go_to_teaching=True)
            level = self.get_logger().info if success else self.get_logger().error
            level(f'[SafeRecovery] {msg}')

    def _do_recovery(self, go_to_teaching: bool = False) -> tuple:
        """
        SAFE_STOP / SAFE_OFF → STANDBY 복구 시퀀스.
        SAFE_STOP: SetSafeStopResetType(0) → SetRobotControl(RESET_SAFET_STOP=2)
        SAFE_OFF : SetRobotMode(AUTO) → SetRobotControl(RESET_SAFET_OFF=3)
        성공 시 go_to_teaching=True 이면 MANUAL 모드 진입.
        """
        log   = self.get_logger()
        state = self._get_robot_state_now()

        log.warn(f'[SafeRecovery] 현재 상태: {STATE_STR.get(state, state)}')

        if state == STATE_STANDBY:
            if go_to_teaching:
                return self._enter_teaching_mode()
            return True, '이미 STANDBY 상태'

        if state not in (STATE_SAFE_STOP, STATE_SAFE_OFF, STATE_NOT_READY):
            return False, f'복구 불가 상태: {STATE_STR.get(state, state)}'

        # 마지막 알람 조회 (참고용 로그)
        alarm_resp = self._call_service_sync(
            self._cli_get_last_alarm, GetLastAlarm.Request(), timeout=3.0
        )
        if alarm_resp and alarm_resp.success:
            a = alarm_resp.log_alarm
            log.warn(
                f'[SafeRecovery] 마지막 알람 — level={a.level} group={a.group} '
                f'index={a.index} params={list(a.param)}'
            )

        if state == STATE_SAFE_STOP:
            # Step 1: 안전정지 리셋 타입 설정 (PROGRAM_STOP)
            rst_req            = SetSafeStopResetType.Request()
            rst_req.reset_type = 0
            self._call_service_sync(self._cli_set_safe_stop_reset, rst_req, timeout=5.0)
            log.info('[SafeRecovery] SetSafeStopResetType(0) 전송')

            # Step 2: CONTROL_RESET_SAFET_STOP
            ctrl_req               = SetRobotControl.Request()
            ctrl_req.robot_control = CONTROL_RESET_SAFET_STOP
            self._call_service_sync(self._cli_set_control, ctrl_req, timeout=5.0)
            log.info('[SafeRecovery] SetRobotControl(RESET_SAFET_STOP) 전송')

        else:  # SAFE_OFF / NOT_READY
            # Step 1: AUTONOMOUS 모드 전환
            mode_req            = SetRobotMode.Request()
            mode_req.robot_mode = 1
            self._call_service_sync(self._cli_set_mode, mode_req, timeout=5.0)
            log.info('[SafeRecovery] SetRobotMode(AUTONOMOUS) 전송')

            # Step 2: 서보 ON (RESET_SAFET_OFF)
            ctrl_req               = SetRobotControl.Request()
            ctrl_req.robot_control = CONTROL_RESET_SAFET_OFF
            self._call_service_sync(self._cli_set_control, ctrl_req, timeout=5.0)
            log.info('[SafeRecovery] SetRobotControl(RESET_SAFET_OFF) 전송')

        # STANDBY 대기 (최대 15초)
        deadline = time.time() + 15.0
        while time.time() < deadline:
            s = self._get_robot_state_now()
            if s == STATE_STANDBY:
                self._robot_state = STATE_STANDBY
                log.info('[SafeRecovery] 복구 성공 → STANDBY')
                if go_to_teaching:
                    return self._enter_teaching_mode()
                return True, '안전복구 성공'
            time.sleep(0.5)

        final = self._get_robot_state_now()
        return False, (
            f'안전복구 실패 — 최종 상태: {STATE_STR.get(final, final)}\n'
            '  하드웨어 안전 조건(비상정지 버튼, 안전 펜스) 확인 필요'
        )

    def _enter_teaching_mode(self) -> tuple:
        """서보 ON 상태에서 MANUAL 모드로 진입."""
        mode_req            = SetRobotMode.Request()
        mode_req.robot_mode = 0   # MANUAL
        resp = self._call_service_sync(self._cli_set_mode, mode_req, timeout=5.0)
        if resp and resp.success:
            self._teaching_mode = True
            self.get_logger().info(
                '[SafeRecovery] 직접 교시 모드 진입 — '
                '로봇 위치 확인 후 /arm/teaching enable=false 로 해제하세요'
            )
            return True, '안전복구 완료 → 직접 교시 모드 진입'
        return False, 'SetRobotMode(MANUAL) 실패'

    def _recover_callback(self, request: Recover.Request,
                          response: Recover.Response) -> Recover.Response:
        """수동 안전복구 트리거 (/arm/recover)."""
        # 알람 문자열 수집
        alarm_str  = ''
        alarm_resp = self._call_service_sync(
            self._cli_get_last_alarm, GetLastAlarm.Request(), timeout=3.0
        )
        if alarm_resp and alarm_resp.success:
            a = alarm_resp.log_alarm
            alarm_str = f'level={a.level} group={a.group} index={a.index} params={list(a.param)}'

        success, msg = self._do_recovery(go_to_teaching=request.go_to_teaching)
        response.success          = success
        response.message          = msg
        response.robot_state_after = self._get_robot_state_now()
        response.last_alarm       = alarm_str
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
