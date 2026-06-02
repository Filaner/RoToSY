"""
ROS2 bridge node that runs in a background thread and exposes robot state
to the FastAPI layer via a thread-safe snapshot dict.

Future integration points:
  - Vision: subscribe to /vision/object_pose here and merge into _state
  - DB: publish events from here or let routers call a DB service client
"""

import threading
import asyncio
from typing import Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Bool, Float64MultiArray

# Custom interfaces
from robot_arm_interfaces.msg import RobotStatus
from robot_arm_interfaces.srv import ServoOn, Jog, Home, Teaching, EStop, Recover
from robot_arm_interfaces.action import MoveJ, MoveL


class RobotBridgeNode(Node):
    """Subscribes to arm topics and maintains a thread-safe state snapshot."""

    def __init__(self):
        super().__init__('web_interface_node')
        self._lock = threading.Lock()
        self._command_lock = asyncio.Lock()
        self._state: dict = {
            'robot_state':        -1,
            'robot_state_str':    'UNKNOWN',
            'servo_on':           False,
            'is_moving':          False,
            'teaching_mode':      False,
            'arm_ready':          False,
            'current_joints_deg': [0.0] * 6,
            'current_tcp':        [0.0] * 6,  # DSR-native coords — use for MoveL input
            'current_tcp_rt':     [0.0] * 6,  # TF2-derived real-time — display only
            'error_code':         0,
            'error_message':      '',
        }

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(RobotStatus,      '/arm/status',   self._status_cb,   10)
        self.create_subscription(Bool,             '/arm/ready',    self._ready_cb,    10)
        self.create_subscription(Float64MultiArray, '/arm/tcp_pose', self._tcp_pose_cb, 10)

        # ── Service Clients ──────────────────────────────────────────────────
        self._cli_servo    = self.create_client(ServoOn,  '/arm/servo_on')
        self._cli_jog      = self.create_client(Jog,      '/arm/jog')
        self._cli_home     = self.create_client(Home,     '/arm/home')
        self._cli_teaching = self.create_client(Teaching, '/arm/teaching')
        self._cli_estop    = self.create_client(EStop,    '/arm/estop')
        self._cli_recover  = self.create_client(Recover,  '/arm/recover')

        # ── Action Clients ───────────────────────────────────────────────────
        self._ac_movej = ActionClient(self, MoveJ, '/arm/move_j')
        self._ac_movel = ActionClient(self, MoveL, '/arm/move_l')

        self.get_logger().info('Web interface ROS2 node started.')

    async def call_servo(self, enable: bool) -> dict:
        """Asynchronously call the /arm/servo_on service with a lock."""
        if self._command_lock.locked():
            return {'success': False, 'message': 'Robot is busy with another command'}

        async with self._command_lock:
            req = ServoOn.Request()
            req.enable = enable
            return await self._run_srv(self._cli_servo, req)

    async def call_jog(self, joint_index: int, speed: float) -> dict:
        """Asynchronously call the /arm/jog service."""
        # Note: We don't necessarily want to block jogging with command_lock 
        # if the user wants to stop (speed=0) while something else is happening,
        # but usually jogging is the only active command.
        # Let's NOT lock for speed=0 (stop) so it always works.
        if speed == 0:
            req = Jog.Request()
            req.joint_index = joint_index
            req.speed = 0.0
            return await self._run_srv(self._cli_jog, req)
        
        if self._command_lock.locked():
            return {'success': False, 'message': 'Robot is busy'}

        async with self._command_lock:
            req = Jog.Request()
            req.joint_index = joint_index
            req.speed = speed
            return await self._run_srv(self._cli_jog, req)

    async def call_movej(self, joints: list, vel: float, acc: float) -> dict:
        """Execute a MoveJ action."""
        if self._command_lock.locked():
            return {'success': False, 'message': 'Robot is busy'}

        async with self._command_lock:
            if not self._ac_movej.wait_for_server(timeout_sec=2.0):
                return {'success': False, 'message': 'MoveJ action server unavailable'}

            goal = MoveJ.Goal()
            goal.joint_angles_deg = [float(j) for j in joints]
            goal.velocity_deg_s = float(vel)
            goal.acceleration_deg_s2 = float(acc)
            
            return await self._run_action(self._ac_movej, goal)

    async def call_movel(self, pose: list, vel: float, acc: float) -> dict:
        """Execute a MoveL action."""
        if self._command_lock.locked():
            return {'success': False, 'message': 'Robot is busy'}

        async with self._command_lock:
            if not self._ac_movel.wait_for_server(timeout_sec=2.0):
                return {'success': False, 'message': 'MoveL action server unavailable'}

            goal = MoveL.Goal()
            goal.x, goal.y, goal.z, goal.rx, goal.ry, goal.rz = [float(p) for p in pose]
            goal.linear_velocity_mm_s = float(vel)
            goal.linear_accel_mm_s2 = float(acc)
            
            return await self._run_action(self._ac_movel, goal)

    async def call_home(self) -> dict:
        """Execute a Home move."""
        if self._command_lock.locked():
            return {'success': False, 'message': 'Robot is busy'}

        async with self._command_lock:
            req = Home.Request()
            req.target = 0 # Mechanical home
            return await self._run_srv(self._cli_home, req)

    async def call_teaching(self, enable: bool) -> dict:
        """Toggle direct teaching (manual) mode."""
        req = Teaching.Request()
        req.enable = enable
        return await self._run_srv(self._cli_teaching, req)

    async def call_estop(self) -> dict:
        """Emergency stop: halt motion and disable servo immediately."""
        return await self._run_srv(self._cli_estop, EStop.Request())

    async def call_jog_cart_step(self, axis: int, direction: int, step: float) -> bool:
        """현재 TCP에서 axis 방향으로 step(mm 또는 deg) 만큼 MoveL."""
        state = self.get_state()
        tcp = state.get('current_tcp', [0.0] * 6)
        if len(tcp) < 6 or all(v == 0.0 for v in tcp[:3]):
            return False

        target = [float(v) for v in tcp]
        target[axis] += step * direction

        if not self._ac_movel.wait_for_server(timeout_sec=0.5):
            return False

        goal = MoveL.Goal()
        goal.x, goal.y, goal.z, goal.rx, goal.ry, goal.rz = target
        goal.linear_velocity_mm_s = 50.0
        goal.linear_accel_mm_s2   = 100.0

        send_fut = self._ac_movel.send_goal_async(goal)
        while not send_fut.done():
            await asyncio.sleep(0.01)

        handle = send_fut.result()
        if not handle.accepted:
            return False

        res_fut = handle.get_result_async()
        while not res_fut.done():
            await asyncio.sleep(0.02)

        return bool(res_fut.result().result.success)

    async def call_recover(self) -> dict:
        """Trigger safety recovery: SAFE_STOP/SAFE_OFF → STANDBY (servo ON)."""
        req = Recover.Request()
        req.go_to_teaching = False
        return await self._run_srv(self._cli_recover, req)

    async def _run_action(self, client, goal) -> dict:
        """Helper to run a ROS2 action and return the result."""
        # Send goal
        send_goal_future = client.send_goal_async(goal)
        while not send_goal_future.done():
            await asyncio.sleep(0.02)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            return {'success': False, 'message': 'Goal rejected by server'}
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        while not result_future.done():
            await asyncio.sleep(0.05)
        
        result = result_future.result().result
        return {
            'success': bool(result.success),
            'message': str(result.message),
            'time': float(result.execution_time_sec)
        }

    async def _run_srv(self, client, request) -> dict:
        """Wait for service and poll the rclpy future in an asyncio-friendly way."""
        self.get_logger().info(f'Calling service {client.srv_name}...')
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f'Service {client.srv_name} not available')
            return {'success': False, 'message': f'Service {client.srv_name} not available'}
        
        try:
            future = client.call_async(request)
            # Poll the ROS future without blocking the asyncio loop
            # Faster polling (20ms) for better responsiveness in jogging
            while not future.done():
                await asyncio.sleep(0.02)

            result = future.result()

            if result is None:
                return {'success': False, 'message': 'Service returned None'}
            
            # Ensure we return a clean dict with strings, not ROS types
            return {
                'success': bool(result.success),
                'message': str(getattr(result, 'message', 'OK'))
            }
        except Exception as e:
            self.get_logger().error(f'Service call exception ({client.srv_name}): {e}')
            return {'success': False, 'message': f'Internal Error: {str(e)}'}

    def _status_cb(self, msg: RobotStatus) -> None:
        with self._lock:
            update = {
                'robot_state':        msg.robot_state,
                'robot_state_str':    msg.robot_state_str,
                'servo_on':           msg.servo_on,
                'is_moving':          msg.is_moving,
                'teaching_mode':      msg.teaching_mode,
                'current_joints_deg': list(msg.current_joints_deg),
                'error_code':         msg.error_code,
                'error_message':      msg.error_message,
            }
            # current_tcp: DSR-native coords from GetCurrentPose.
            # Same coordinate convention as move_line → always use this for MoveL sync.
            dsr_tcp = list(msg.current_tcp)
            if any(v != 0.0 for v in dsr_tcp):
                update['current_tcp'] = dsr_tcp
            self._state.update(update)

    def _ready_cb(self, msg: Bool) -> None:
        with self._lock:
            self._state['arm_ready'] = msg.data

    def _tcp_pose_cb(self, msg: Float64MultiArray) -> None:
        # TF2-derived real-time values — display only, NOT used for MoveL input.
        # DSR-native current_tcp (from status_cb) is the correct source for MoveL.
        if len(msg.data) >= 6:
            with self._lock:
                self._state['current_tcp_rt'] = list(msg.data[:6])

    def get_state(self) -> dict:
        """Return a shallow copy of the current robot state (thread-safe)."""
        with self._lock:
            return dict(self._state)


# ── Module-level singleton ────────────────────────────────────────────────────

_node:        Optional[RobotBridgeNode]      = None
_executor:    Optional[MultiThreadedExecutor] = None
_spin_thread: Optional[threading.Thread]     = None


def init_ros() -> RobotBridgeNode:
    """Initialize rclpy, create the bridge node, and start spinning."""
    global _node, _executor, _spin_thread
    rclpy.init()
    _node     = RobotBridgeNode()
    _executor = MultiThreadedExecutor()
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
