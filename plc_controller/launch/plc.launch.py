from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('plc_controller')

    return LaunchDescription([
        DeclareLaunchArgument('plc_ip',   default_value='192.168.1.10'),
        DeclareLaunchArgument('plc_port', default_value='502'),

        Node(
            package='plc_controller',
            executable='plc_controller_node',
            name='plc_controller_node',
            parameters=[
                os.path.join(pkg, 'config', 'plc.yaml'),
            ],
            output='screen',
        ),
    ])
