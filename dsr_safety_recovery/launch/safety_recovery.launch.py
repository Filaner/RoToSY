"""
Launch the DSR Safety Recovery node.

Usage:
  ros2 launch dsr_safety_recovery safety_recovery.launch.py
  ros2 launch dsr_safety_recovery safety_recovery.launch.py robot_ns:=dsr01
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument(
            'robot_ns',
            default_value='dsr01',
            description='DSR robot namespace (used for /dsr01/system/set_robot_mode)',
        ),
    ]

    safety_recovery = Node(
        package='dsr_safety_recovery',
        executable='safety_recovery',
        name='safety_recovery_node',
        output='screen',
        parameters=[{
            'robot_ns': LaunchConfiguration('robot_ns'),
        }],
    )

    return LaunchDescription(args + [safety_recovery])
