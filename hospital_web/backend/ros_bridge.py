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

        self.node.get_logger().info('AMR bridge node started')

    def destroy_node(self):
        self.node.destroy_node()

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
