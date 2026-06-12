"""
ROS2 bridge for AMR and door subsystems.
Runs in a background daemon thread; exposes thread-safe state via get_state().

Topic conventions (adjust to match the AMR simulator):
  Subscribe:
    /amr/status   std_msgs/String   → IDLE | DELIVERING | ARRIVED | RETURNING | CHARGING | ERROR
    /amr/battery  std_msgs/Float32  → 0.0–100.0
    /amr/pose     geometry_msgs/Pose2D → x, y, theta
    /door/status  std_msgs/String   → CLOSED | OPEN | ERROR

  Services (Trigger):
    /amr/dispatch        → send AMR on mission
    /amr/cancel          → cancel current mission
    /amr/return_to_base  → return AMR to dock
    /door/open           → open ward door
    /door/close          → close ward door
"""

import threading
import time
import math
import random
from datetime import datetime
from typing import Optional

_lock  = threading.Lock()
_state = {
    'amr': {
        'status':      'IDLE',
        'battery':     100.0,
        'pose':        {'x': 0.0, 'y': 0.0, 'theta': 0.0},
        'destination': '',
        'online':      False,
        'last_seen':   None,
    },
    'door': {
        'status':    'CLOSED',
        'online':    False,
        'last_seen': None,
    },
    'nodes': {
        'arm_controller': {'status': 'UNKNOWN', 'last_seen': None},
        'amr_controller': {'status': 'UNKNOWN', 'last_seen': None},
        'vision_node':    {'status': 'UNKNOWN', 'last_seen': None},
    },
    'arduino': {
        'temperature': None,
        'humidity':    None,
        'status':      'OFFLINE',
        'is_alert':    False,
        'last_seen':   None,
        'online':      False,
    },
}

_ros_available   = False
_node            = None
_executor        = None
_thread          = None
_mock_sensor_thr = None


def get_state() -> dict:
    with _lock:
        import copy
        s = copy.deepcopy(_state)
    _refresh_node_health(s)
    return s


# ── ROS2 init ─────────────────────────────────────────────────────────────────

def init() -> bool:
    global _ros_available, _node, _executor, _thread
    try:
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        rclpy.init()
        _node     = _AMRBridgeNode()
        _executor = MultiThreadedExecutor()
        _executor.add_node(_node.node)   # pass the real rclpy Node
        _thread   = threading.Thread(target=_executor.spin, daemon=True)
        _thread.start()
        _ros_available = True
        print('[ros_bridge] ROS2 initialized — AMR/door topics active')
        return True
    except Exception as e:
        print(f'[ros_bridge] ROS2 unavailable ({e}) — running in mock mode')
        _ros_available = False
        _start_mock_sensor()
        return False


def shutdown() -> None:
    global _executor, _node
    if _executor:
        try:
            _executor.shutdown(timeout_sec=2.0)
        except Exception:
            pass
    if _node:
        try:
            _node.destroy_node()
        except Exception:
            pass
    try:
        import rclpy
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass


# ── AMR / Door commands ───────────────────────────────────────────────────────

async def dispatch(destination: str = '') -> dict:
    if not _ros_available or _node is None:
        with _lock:
            _state['amr']['status']      = 'DELIVERING'
            _state['amr']['destination'] = destination
        return {'success': True, 'message': '[mock] AMR dispatched'}
    return await _node.call_trigger('/amr/dispatch')


async def goto(x: float, y: float, theta: float = 0.0, label: str = '') -> dict:
    """raw 좌표(map frame)로 Nav2 골을 전송한다. (테스트/직접 이동용)"""
    if not _ros_available or _node is None:
        with _lock:
            _state['amr']['status']      = 'DELIVERING'
            _state['amr']['destination'] = label or f'({x:.2f}, {y:.2f})'
        return {'success': True, 'message': '[mock] goto (ROS 미연결)'}
    return await _node.navigate_to(x, y, theta, label)


async def stop() -> dict:
    """진행 중인 Nav2 이동을 취소한다."""
    if not _ros_available or _node is None:
        with _lock:
            _state['amr']['status'] = 'IDLE'
        return {'success': True, 'message': '[mock] stop'}
    return await _node.navigate_cancel()


async def cancel_mission() -> dict:
    if not _ros_available or _node is None:
        with _lock:
            _state['amr']['status'] = 'RETURNING'
        return {'success': True, 'message': '[mock] AMR mission cancelled'}
    return await _node.call_trigger('/amr/cancel')


async def return_to_base() -> dict:
    if not _ros_available or _node is None:
        with _lock:
            _state['amr']['status'] = 'RETURNING'
        return {'success': True, 'message': '[mock] AMR returning'}
    return await _node.call_trigger('/amr/return_to_base')


async def door_open() -> dict:
    if not _ros_available or _node is None:
        with _lock:
            _state['door']['status'] = 'OPEN'
        return {'success': True, 'message': '[mock] Door opened'}
    return await _node.call_trigger('/door/open')


async def door_close() -> dict:
    if not _ros_available or _node is None:
        with _lock:
            _state['door']['status'] = 'CLOSED'
        return {'success': True, 'message': '[mock] Door closed'}
    return await _node.call_trigger('/door/close')


# ── Node health helper ────────────────────────────────────────────────────────

def update_node_seen(node_name: str) -> None:
    with _lock:
        if node_name in _state['nodes']:
            _state['nodes'][node_name]['last_seen'] = datetime.now().isoformat()


def _refresh_node_health(s: dict) -> None:
    now = datetime.now()
    for name, info in s['nodes'].items():
        ls = info.get('last_seen')
        if ls is None:
            info['status'] = 'UNKNOWN'
        else:
            age = (now - datetime.fromisoformat(ls)).total_seconds()
            if age < 3:
                info['status'] = 'ONLINE'
            elif age < 10:
                info['status'] = 'DEGRADED'
            else:
                info['status'] = 'OFFLINE'


# ── ROS2 Node class ───────────────────────────────────────────────────────────

class _AMRBridgeNode:
    """
    Thin wrapper that creates a proper rclpy Node internally and delegates
    executor management via the `.node` property.
    """
    def __init__(self):
        from std_msgs.msg import String, Float32
        import rclpy

        # Create a real rclpy Node so the executor can access .subscriptions etc.
        self.node = rclpy.create_node('hospital_web_amr_bridge')

        self.node.create_subscription(String,  '/amr/status',  self._amr_status_cb,  10)
        self.node.create_subscription(Float32, '/amr/battery', self._amr_battery_cb, 10)
        self.node.create_subscription(String,  '/door/status', self._door_status_cb, 10)

        try:
            from geometry_msgs.msg import Pose2D
            self.node.create_subscription(Pose2D, '/amr/pose', self._amr_pose_cb, 10)
        except Exception:
            pass

        # Nav2 위치추정 결과(/amcl_pose)로 로봇 실시간 위치를 받는다.
        try:
            from geometry_msgs.msg import PoseWithCovarianceStamped
            self.node.create_subscription(
                PoseWithCovarianceStamped, '/amcl_pose', self._amcl_pose_cb, 10)
        except Exception:
            pass

        # Nav2 navigate_to_pose 액션 클라이언트 — 웹에서 받은 좌표로 골을 전송한다.
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient
        self._nav_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
        self._nav_goal_handle = None

        self.node.get_logger().info('AMR bridge node started')

    def destroy_node(self):
        self.node.destroy_node()

    # ── Nav2 navigation ──────────────────────────────────────────────────────

    async def navigate_to(self, x: float, y: float, theta: float = 0.0,
                          label: str = '') -> dict:
        import asyncio
        import math
        from nav2_msgs.action import NavigateToPose

        if not self._nav_client.wait_for_server(timeout_sec=3.0):
            return {'success': False,
                    'message': 'Nav2 navigate_to_pose 액션 서버 없음 — 시뮬레이션이 실행 중인지 확인'}

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp    = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        half = float(theta) * 0.5
        goal.pose.pose.orientation.z = math.sin(half)
        goal.pose.pose.orientation.w = math.cos(half)

        send_future = self._nav_client.send_goal_async(goal)
        while not send_future.done():
            await asyncio.sleep(0.02)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return {'success': False, 'message': 'Nav2가 골을 거부했습니다'}

        self._nav_goal_handle = handle
        with _lock:
            _state['amr']['status']      = 'DELIVERING'
            _state['amr']['destination'] = label or f'({x:.2f}, {y:.2f})'
            _state['amr']['online']      = True
            _state['amr']['last_seen']   = datetime.now().isoformat()
        handle.get_result_async().add_done_callback(self._nav_result_cb)
        return {'success': True, 'message': f'이동 시작 → ({x:.2f}, {y:.2f})'}

    def _nav_result_cb(self, future) -> None:
        from action_msgs.msg import GoalStatus
        status = future.result().status
        with _lock:
            if status == GoalStatus.STATUS_SUCCEEDED:
                _state['amr']['status'] = 'ARRIVED'
            elif status == GoalStatus.STATUS_CANCELED:
                _state['amr']['status'] = 'IDLE'
            else:
                _state['amr']['status'] = 'ERROR'
            _state['amr']['last_seen'] = datetime.now().isoformat()

    async def navigate_cancel(self) -> dict:
        import asyncio
        if self._nav_goal_handle is None:
            return {'success': True, 'message': '취소할 이동 없음'}
        cancel_future = self._nav_goal_handle.cancel_goal_async()
        while not cancel_future.done():
            await asyncio.sleep(0.02)
        with _lock:
            _state['amr']['status'] = 'IDLE'
        return {'success': True, 'message': '이동 취소됨'}

    def _amcl_pose_cb(self, msg) -> None:
        import math
        q = msg.pose.pose.orientation
        theta = math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))
        with _lock:
            _state['amr']['pose'] = {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'theta': theta,
            }
            _state['amr']['online']    = True
            _state['amr']['last_seen'] = datetime.now().isoformat()

    def _amr_status_cb(self, msg) -> None:
        with _lock:
            _state['amr']['status']    = msg.data
            _state['amr']['online']    = True
            _state['amr']['last_seen'] = datetime.now().isoformat()
        update_node_seen('amr_controller')

    def _amr_battery_cb(self, msg) -> None:
        with _lock:
            _state['amr']['battery'] = float(msg.data)

    def _amr_pose_cb(self, msg) -> None:
        with _lock:
            _state['amr']['pose'] = {'x': msg.x, 'y': msg.y, 'theta': msg.theta}

    def _door_status_cb(self, msg) -> None:
        with _lock:
            _state['door']['status']    = msg.data
            _state['door']['online']    = True
            _state['door']['last_seen'] = datetime.now().isoformat()

    def _arduino_temp_cb(self, msg) -> None:
        with _lock:
            _state['arduino']['temperature'] = round(float(msg.data), 1)
            _state['arduino']['online']      = True
            _state['arduino']['last_seen']   = datetime.now().isoformat()
        _check_sensor_alert()

    def _arduino_humi_cb(self, msg) -> None:
        with _lock:
            _state['arduino']['humidity']  = round(float(msg.data), 1)
            _state['arduino']['online']    = True
            _state['arduino']['last_seen'] = datetime.now().isoformat()
        _check_sensor_alert()

    async def call_trigger(self, srv_name: str) -> dict:
        import asyncio
        from std_srvs.srv import Trigger
        cli = self.node.create_client(Trigger, srv_name)
        if not cli.wait_for_service(timeout_sec=2.0):
            return {'success': False, 'message': f'Service {srv_name} not available'}
        future = cli.call_async(Trigger.Request())
        while not future.done():
            await asyncio.sleep(0.02)
        res = future.result()
        return {'success': bool(res.success), 'message': str(res.message)}


# ── Arduino 상태 헬퍼 ─────────────────────────────────────────────────────────

def _check_sensor_alert() -> None:
    with _lock:
        t = _state['arduino']['temperature']
        h = _state['arduino']['humidity']
        if t is None or h is None:
            return
        is_alert = not (15.0 <= t <= 25.0) or not (40.0 <= h <= 70.0)
        _state['arduino']['is_alert'] = is_alert
        _state['arduino']['status']   = 'WARNING' if is_alert else 'NORMAL'


def update_arduino_reading(temperature: float, humidity: float) -> None:
    """외부에서 센서 값 갱신 (ROS2/mock 공통)."""
    from . import sensor_db as sdb
    sdb.insert_reading(temperature, humidity)
    with _lock:
        _state['arduino']['temperature'] = round(temperature, 1)
        _state['arduino']['humidity']    = round(humidity, 1)
        _state['arduino']['online']      = True
        _state['arduino']['last_seen']   = datetime.now().isoformat()
    _check_sensor_alert()


# ── Mock 센서 루프 ────────────────────────────────────────────────────────────

def _start_mock_sensor() -> None:
    global _mock_sensor_thr
    _mock_sensor_thr = threading.Thread(target=_mock_sensor_loop, daemon=True)
    _mock_sensor_thr.start()
    print('[ros_bridge] Mock sensor loop started (30s interval)')


def _mock_sensor_loop() -> None:
    """ROS2가 없을 때 30초마다 현실적인 가상 센서 값 생성."""
    base_temp = 20.5
    base_humi = 55.0
    while True:
        now   = datetime.now()
        h_fac = math.sin(now.hour * math.pi / 12) * 1.5
        temp  = round(base_temp + h_fac + random.gauss(0, 0.2), 1)
        humi  = round(base_humi - h_fac * 1.2 + random.gauss(0, 0.8), 1)
        temp  = max(13.0, min(28.0, temp))
        humi  = max(30.0, min(80.0, humi))
        try:
            update_arduino_reading(temp, humi)
        except Exception:
            pass
        time.sleep(30)
