"""
ROS2 bridge node that runs in a background thread and exposes robot state
to the FastAPI layer via a thread-safe snapshot dict.

Future integration points:
  - Vision: subscribe to /vision/object_pose here and merge into _state
  - DB: publish events from here or let routers call a DB service client
"""

import threading
import asyncio
import time
from typing import Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Bool, Float64MultiArray, String, Int32, Empty

# Custom interfaces
from robot_arm_interfaces.msg import PlcCommand, RobotStatus
from robot_arm_interfaces.srv import ServoOn, Jog, Home, Teaching, EStop, Recover
from robot_arm_interfaces.action import MoveJ, MoveL
from dsr_msgs2.srv import SetToolDigitalOutput, GetToolDigitalOutput

class RobotBridgeNode(Node):
    """ROS2 node that bridges robot state to the FastAPI application."""

    def __init__(self):
        super().__init__('web_server_bridge')
        self._lock = threading.Lock()
        self._command_lock = asyncio.Lock()
        self._state = {
            'robot_state':     -1,
            'robot_state_str': 'DISCONNECTED',
            'servo_on':        False,
            'is_moving':       False,
            'arm_ready':       False,
            'teaching_mode':   False,
            'current_joints_deg': [0.0]*6,
            'current_tcp':     [0.0]*6,
            'current_tcp_rt':  [0.0]*6,
            'error_code':      0,
            'error_message':   '',
            'magnet_on':       False,
            'safety_recovery_needed':   False,
            'seq_step':        'IDLE',
            'tmp_step':        'IDLE',
            'inverter_running': False,
            'inverter_freq':    0,
        }

        # Subscribers
        self.create_subscription(RobotStatus, '/arm/status', self._status_cb, 10)
        self.create_subscription(Bool, '/arm/ready', self._ready_cb, 10)
        self.create_subscription(Float64MultiArray, '/arm/tcp_pose', self._tcp_rt_cb, 10)
        self.create_subscription(String, '/motion/step_info', self._seq_step_cb, 10)
        self.create_subscription(String, '/temp_motion/step_info', self._tmp_step_cb, 10)

        # Clients (for REST API use)
        self.cli_servo    = self.create_client(ServoOn,    '/arm/servo_on')
        self.cli_recovery = self.create_client(Recover,    '/arm/safety_recovery')
        self.cli_jog      = self.create_client(Jog,        '/arm/jog')
        self.cli_home     = self.create_client(Home,       '/arm/home')
        self.cli_teaching = self.create_client(Teaching,   '/arm/teaching')
        self.cli_estop    = self.create_client(EStop,      '/arm/estop')
        self.cli_magnet   = self.create_client(SetToolDigitalOutput, '/dsr01/io/set_tool_digital_output')
        self.cli_mag_get  = self.create_client(GetToolDigitalOutput, '/dsr01/io/get_tool_digital_output')

        # Action Clients
        self.act_move_j   = ActionClient(self, MoveJ, '/arm/move_j')
        self.act_move_l   = ActionClient(self, MoveL, '/arm/move_l')

        # Publishers
        self.pub_plc       = self.create_publisher(PlcCommand, '/plc_command', 10)
        self.pub_seq_start = self.create_publisher(Int32, '/motion/start', 10)
        self.pub_seq_next  = self.create_publisher(Empty, '/motion/next_step', 10)
        self.pub_seq_stop  = self.create_publisher(Empty, '/motion/stop', 10)
        self.pub_seq_reset = self.create_publisher(Empty, '/motion/reset', 10)

        self.pub_tmp_start = self.create_publisher(Int32, '/temp_motion/start', 10)
        self.pub_tmp_next  = self.create_publisher(Empty, '/temp_motion/next_step', 10)
        self.pub_tmp_stop  = self.create_publisher(Empty, '/temp_motion/stop', 10)

        # Magnet state polling timer (1Hz)
        self.create_timer(1.0, self._poll_magnet)

    # 안전복구가 필요한 상태 (서보 ON 불가, ⚡ 안전 복구 버튼 활성화)
    _RECOVERY_STATES = frozenset({5, 8, 9, 10})  # SAFE_STOP, RECOVERY, SAFE_STOP2, SAFE_OFF2

    def _status_cb(self, msg: RobotStatus):
        with self._lock:
            self._state['robot_state']            = msg.robot_state
            self._state['robot_state_str']        = msg.robot_state_str
            self._state['servo_on']               = msg.servo_on
            self._state['is_moving']              = msg.is_moving
            self._state['teaching_mode']          = msg.teaching_mode
            self._state['current_joints_deg']     = list(msg.current_joints_deg)
            self._state['current_tcp']            = list(msg.current_tcp)
            self._state['error_code']             = msg.error_code
            self._state['error_message']          = msg.error_message
            self._state['safety_recovery_needed'] = msg.robot_state in self._RECOVERY_STATES

    def _ready_cb(self, msg: Bool):
        with self._lock:
            self._state['arm_ready'] = msg.data

    def _seq_step_cb(self, msg: String):
        with self._lock:
            self._state['seq_step'] = msg.data

    def _tmp_step_cb(self, msg: String):
        with self._lock:
            self._state['tmp_step'] = msg.data

    def _poll_magnet(self):
        if not self.cli_mag_get.service_is_ready():
            return
        req = GetToolDigitalOutput.Request()
        req.index = 1
        future = self.cli_mag_get.call_async(req)
        # Note: We don't block here as this is called from the timer callback
        future.add_done_callback(self._magnet_resp_cb)

    def _magnet_resp_cb(self, future):
        try:
            res = future.result()
            if res and res.success:
                with self._lock:
                    self._state['magnet_on'] = (res.value == 1)
        except Exception:
            pass

    def _tcp_rt_cb(self, msg: Float64MultiArray):
        if len(msg.data) >= 6:
            with self._lock:
                self._state['current_tcp_rt'] = list(msg.data[:6])

    def get_state(self) -> dict:
        """Return a shallow copy of the current robot state (thread-safe)."""
        with self._lock:
            return dict(self._state)

    async def call_safety_recovery(self) -> dict:
        """안전복구 서비스 호출 (/arm/safety_recovery)."""
        if self._command_lock.locked():
            return {'success': False, 'message': 'Robot is busy with another command'}
        async with self._command_lock:
            req = Recover.Request()
            req.go_to_teaching = False
            return await self._run_srv(self.cli_recovery, req)

    async def call_servo(self, enable: bool) -> dict:
        if self._command_lock.locked():
            return {'success': False, 'message': 'Robot is busy with another command'}
        async with self._command_lock:
            req = ServoOn.Request()
            req.enable = enable
            return await self._run_srv(self.cli_servo, req)

    async def call_jog(self, joint_index: int, speed: float) -> dict:
        req = Jog.Request()
        req.joint_index = joint_index
        req.speed = speed
        if speed == 0:
            return await self._run_srv(self.cli_jog, req)
        if self._command_lock.locked():
            return {'success': False, 'message': 'Robot is busy'}
        async with self._command_lock:
            return await self._run_srv(self.cli_jog, req)

    async def call_movej(self, joints: list, vel: float, acc: float) -> dict:
        if self._command_lock.locked():
            return {'success': False, 'message': 'Robot is busy'}
        async with self._command_lock:
            if not await self._wait_for_action(self.act_move_j):
                return {'success': False, 'message': 'MoveJ action server unavailable'}
            goal = MoveJ.Goal()
            goal.joint_angles_deg = [float(j) for j in joints]
            goal.velocity_deg_s = float(vel)
            goal.acceleration_deg_s2 = float(acc)
            return await self._run_action(self.act_move_j, goal)

    async def call_movel(self, pose: list, vel: float, acc: float) -> dict:
        if self._command_lock.locked():
            return {'success': False, 'message': 'Robot is busy'}
        async with self._command_lock:
            if not await self._wait_for_action(self.act_move_l):
                return {'success': False, 'message': 'MoveL action server unavailable'}
            goal = MoveL.Goal()
            goal.x, goal.y, goal.z, goal.rx, goal.ry, goal.rz = [float(p) for p in pose]
            goal.linear_velocity_mm_s = float(vel)
            goal.linear_accel_mm_s2 = float(acc)
            return await self._run_action(self.act_move_l, goal)

    async def call_home(self) -> dict:
        if self._command_lock.locked():
            return {'success': False, 'message': 'Robot is busy'}
        async with self._command_lock:
            req = Home.Request()
            req.target = 0
            return await self._run_srv(self.cli_home, req)

    async def call_teaching(self, enable: bool) -> dict:
        req = Teaching.Request()
        req.enable = enable
        return await self._run_srv(self.cli_teaching, req)

    async def call_estop(self) -> dict:
        return await self._run_srv(self.cli_estop, EStop.Request())

    async def call_jog_cart_step(self, axis: int, direction: int, step: float) -> bool:
        tcp = self.get_state().get('current_tcp', [0.0] * 6)
        if len(tcp) < 6 or all(v == 0.0 for v in tcp[:3]):
            return False
        target = [float(v) for v in tcp]
        target[axis] += step * direction
        result = await self.call_movel(target, 50.0, 100.0)
        return bool(result.get('success'))

    def start_sequence(self, marker_id: int) -> None:
        self.pub_seq_start.publish(Int32(data=marker_id))

    def next_step(self) -> None:
        self.pub_seq_next.publish(Empty())

    def stop_sequence(self) -> None:
        self.pub_seq_stop.publish(Empty())
        with self._lock:
            self._state['seq_step'] = 'RESETTING'

    def reset_sequence(self) -> None:
        self.pub_seq_reset.publish(Empty())
        with self._lock:
            self._state['seq_step'] = 'RESETTING'

    def start_temp_sequence(self, marker_id: int) -> None:
        self.pub_tmp_start.publish(Int32(data=marker_id))

    def next_temp_step(self) -> None:
        self.pub_tmp_next.publish(Empty())

    def stop_temp_sequence(self) -> None:
        self.pub_tmp_stop.publish(Empty())

    async def call_magnet(self, enabled: bool) -> dict:
        req = SetToolDigitalOutput.Request()
        req.index = 1
        req.value = 1 if enabled else 0
        result = await self._run_srv(self.cli_magnet, req)
        if result.get('success'):
            self._pub_plc('PLC', 'COIL', 0x23, int(enabled), 1)
        return result

    def _pub_plc(self, target: str, command: str, address: int, value: int, slave_id: int):
        msg = PlcCommand()
        msg.target   = target
        msg.command  = command
        msg.address  = address
        msg.value    = value
        msg.slave_id = slave_id
        self.pub_plc.publish(msg)

    def publish_inverter_freq(self, freq: int) -> None:
        """인버터 목표 주파수 설정 (0~6000, 단위 0.01 Hz)."""
        self._pub_plc('INVERTER', 'REGISTER', 4, freq, 2)
        with self._lock:
            self._state['inverter_freq'] = freq

    def publish_inverter_run(self, run: bool) -> None:
        """인버터 RUN/STOP 명령 + M21 PLC 코일 연동."""
        self._pub_plc('INVERTER', 'REGISTER', 5, 2 if run else 1, 2)
        self._pub_plc('PLC', 'COIL', 0x21, int(run), 1)
        with self._lock:
            self._state['inverter_running'] = run

    async def _wait_for_action(self, client) -> bool:
        if client.server_is_ready():
            return True
        loop = asyncio.get_running_loop()
        return await loop.run_in_executor(
            None, lambda: client.wait_for_server(timeout_sec=2.0)
        )

    async def _run_action(self, client, goal) -> dict:
        send_future = client.send_goal_async(goal)
        while not send_future.done():
            await asyncio.sleep(0.02)
        handle = send_future.result()
        if not handle.accepted:
            return {'success': False, 'message': 'Goal rejected by server'}
        result_future = handle.get_result_async()
        while not result_future.done():
            await asyncio.sleep(0.05)
        result = result_future.result().result
        return {
            'success': bool(result.success),
            'message': str(result.message),
            'time': float(result.execution_time_sec),
        }

    async def _run_srv(self, client, request) -> dict:
        if not client.service_is_ready():
            loop = asyncio.get_running_loop()
            ready = await loop.run_in_executor(
                None, lambda: client.wait_for_service(timeout_sec=2.0)
            )
            if not ready:
                return {
                    'success': False,
                    'message': f'Service {client.srv_name} not available',
                }
        try:
            future = client.call_async(request)
            while not future.done():
                await asyncio.sleep(0.02)
            result = future.result()
            if result is None:
                return {'success': False, 'message': 'Service returned None'}
            return {
                'success': bool(result.success),
                'message': str(getattr(result, 'message', 'OK')),
            }
        except Exception as exc:
            self.get_logger().error(
                f'Service call exception ({client.srv_name}): {exc}'
            )
            return {'success': False, 'message': f'Internal Error: {exc}'}


# ── Module-level singleton ────────────────────────────────────────────────────

_node:        Optional[RobotBridgeNode]      = None
_executor:    Optional[rclpy.executors.Executor] = None
_spin_thread: Optional[threading.Thread]     = None


def init_ros() -> RobotBridgeNode:
    """Initialize rclpy, create the bridge node, and start spinning."""
    global _node, _executor, _spin_thread
    rclpy.init()
    _node     = RobotBridgeNode()
    _executor = SingleThreadedExecutor()
    _executor.add_node(_node)
    _spin_thread = threading.Thread(target=_executor.spin, daemon=True)
    _spin_thread.start()
    return _node


def shutdown_ros() -> None:
    """Cleanly shut down the executor and rclpy context."""
    global _node, _executor
    if _executor:
        _executor.shutdown(timeout_sec=2.0)
    if _node:
        _node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def get_node() -> Optional[RobotBridgeNode]:
    """Return the active bridge node, or None if not yet initialized."""
    return _node
