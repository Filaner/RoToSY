from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_ns',
            default_value='dsr01',
            description='Doosan DSR namespace / robot ID.',
        ),
        DeclareLaunchArgument(
            'tool_do_index',
            default_value='1',
            description='Flange digital output index for the electromagnet gripper.',
        ),
        Node(
            package='rotosy_gripper_control',
            executable='keyboard_electromagnet_gripper',
            name='keyboard_electromagnet_gripper',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'robot_ns': LaunchConfiguration('robot_ns'),
                'tool_do_index': LaunchConfiguration('tool_do_index'),
            }],
        ),
    ])
