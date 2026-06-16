"""
Launch the Doosan E0509 full stack:
  1. dsr_bringup2        — Doosan DSR hardware/virtual driver (TCP/IP → 110.120.1.52)
  2. plc_controller_node — Modbus RTU PLC/인버터 브리지        (delay 2 s)
  3. arm_controller      — our high-level controller node     (delay 3 s)
  4. tcp_monitor         — TF2 기반 TCP 포즈 퍼블리셔          (delay 5 s)
  5. web_server          — FastAPI 웹 인터페이스 (port 8000)   (delay 5 s)

Usage (real hardware):
  ros2 launch doosan_controller robot_controller.launch.py mode:=real

Usage (virtual / emulator):
  ros2 launch doosan_controller robot_controller.launch.py mode:=virtual
"""

import fcntl
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


_launch_lock_fd = None


def _acquire_launch_lock(_context):
    """Prevent two full stacks from controlling the same robot namespace."""
    global _launch_lock_fd

    lock_path = '/tmp/rotosy_robot_controller.lock'
    fd = os.open(lock_path, os.O_CREAT | os.O_RDWR, 0o644)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        os.close(fd)
        raise RuntimeError(
            'robot_controller.launch.py is already running. '
            'Stop the existing launch before starting another one.'
        ) from exc

    os.ftruncate(fd, 0)
    os.write(fd, str(os.getpid()).encode('ascii'))
    _launch_lock_fd = fd
    return []


_launch_lock_fd = None


def _acquire_launch_lock(_context):
    """Prevent two full stacks from controlling the same robot namespace."""
    global _launch_lock_fd

    lock_path = '/tmp/rotosy_robot_controller.lock'
    fd = os.open(lock_path, os.O_CREAT | os.O_RDWR, 0o644)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        os.close(fd)
        raise RuntimeError(
            'robot_controller.launch.py is already running. '
            'Stop the existing launch before starting another one.'
        ) from exc

    os.ftruncate(fd, 0)
    os.write(fd, str(os.getpid()).encode('ascii'))
    _launch_lock_fd = fd
    return []


def generate_launch_description() -> LaunchDescription:

    # ── Configurable arguments ───────────────────────────────────────────────
    args = [
        DeclareLaunchArgument(
            'host',  default_value='110.120.1.52',
            description='Doosan controller IP address',
        ),
        DeclareLaunchArgument(
            'port',  default_value='12345',
            description='Doosan controller TCP port',
        ),
        DeclareLaunchArgument(
            'mode',  default_value='real',
            description='Operation mode: "real" or "virtual"',
        ),
        DeclareLaunchArgument(
            'model', default_value='e0509',
            description='Robot model (e0509, m1013, …)',
        ),
        DeclareLaunchArgument(
            'name',  default_value='dsr01',
            description='ROS2 namespace / robot ID used by dsr_bringup2',
        ),
        DeclareLaunchArgument(
            'gui',   default_value='false',
            description='Launch RViz2',
        ),
        DeclareLaunchArgument(
            'status_rate_hz', default_value='10.0',
            description='Hz rate for /arm/status publisher',
        ),
        DeclareLaunchArgument(
            'motion_timeout', default_value='60.0',
            description='Maximum seconds allowed for a single MoveJ/MoveL',
        ),
        DeclareLaunchArgument(
            'plc_port', default_value='/dev/ttyUSB0',
            description='Modbus RTU 시리얼 포트 (PLC/인버터 연결)',
        ),
    ]

    # ── dsr_bringup2 (handles all TCP/IP communication to the robot) ─────────
    dsr_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('dsr_bringup2'),
                'launch',
                'dsr_bringup2_rviz.launch.py',
            ])
        ]),
        launch_arguments={
            'host':  LaunchConfiguration('host'),
            'port':  LaunchConfiguration('port'),
            'mode':  LaunchConfiguration('mode'),
            'model': LaunchConfiguration('model'),
            'name':  LaunchConfiguration('name'),
            'gui':   LaunchConfiguration('gui'),
        }.items(),
    )

    # ── plc_controller_node ──────────────────────────────────────────────────
    # 시리얼 포트만 필요, 로봇 서비스와 무관 → 즉시 기동.
    _plc_config = os.path.join(
        get_package_share_directory('plc_controller'), 'config', 'plc.yaml'
    )
    plc_controller = TimerAction(
        period=0.0,
        actions=[
            Node(
                package    = 'plc_controller',
                executable = 'plc_controller_node',
                name       = 'plc_controller_node',
                output     = 'screen',
                parameters = [
                    _plc_config,
                    {'port': LaunchConfiguration('plc_port')},
                ],
            )
        ],
    )

    # ── arm_controller node ──────────────────────────────────────────────────
    # 2 s — dsr_bringup2 서비스 등록 보통 1~1.5 s, 내부 wait_for_service로 보호.
    arm_controller = TimerAction(
        period=2.0,
        actions=[
            Node(
                package    = 'doosan_controller',
                executable = 'arm_controller',
                name       = 'arm_controller',
                output     = 'screen',
                parameters = [{
                    'robot_ns':        LaunchConfiguration('name'),
                    'status_rate_hz':  LaunchConfiguration('status_rate_hz'),
                    'motion_timeout':  LaunchConfiguration('motion_timeout'),
                    'servo_on_retries': 3,
                }],
            )
        ],
    )

    # ── tcp_monitor node ─────────────────────────────────────────────────────
    # 3.5 s — 토픽 구독만, 늦게 오는 메시지는 자동 반영.
    tcp_monitor = TimerAction(
        period=3.5,
        actions=[
            Node(
                package    = 'doosan_controller',
                executable = 'tcp_monitor',
                name       = 'tcp_monitor',
                output     = 'screen',
            )
        ],
    )

    # ── web_server node ──────────────────────────────────────────────────────
    # 3.5 s — /arm/status, /arm/tcp_pose 구독, 연결 전 상태는 WebSocket으로 표시.
    web_server = TimerAction(
        period=3.5,
        actions=[
            Node(
                package    = 'web_interface',
                executable = 'web_server',
                name       = 'web_server',
                output     = 'screen',
            )
        ],
    )

    # ── motion_sequence_node ─────────────────────────────────────────────────
    # 4.5 s — arm_controller(2 s 기동 + ~0.5 s 초기화) 기준.
    motion_sequence = TimerAction(
        period=4.5,
        actions=[
            Node(
                package    = 'doosan_controller',
                executable = 'motion_sequence',
                name       = 'motion_sequence_node',
                output     = 'screen',
            )
        ],
    )

    # ── temp_sequence_node ───────────────────────────────────────────────────
    # 4.5 s — motion_sequence와 동일 조건.
    temp_sequence = TimerAction(
        period=4.5,
        actions=[
            Node(
                package    = 'doosan_controller',
                executable = 'temp_sequence',
                name       = 'temp_sequence_node',
                output     = 'screen',
            )
        ],
    )

    return LaunchDescription(
        args + [
            OpaqueFunction(function=_acquire_launch_lock),
            dsr_bringup,
            plc_controller,
            arm_controller,
            tcp_monitor,
            web_server,
            motion_sequence,
            temp_sequence,
        ]
    )
