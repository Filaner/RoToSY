from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('camera_name', default_value='rotosy_camera'),
        DeclareLaunchArgument('base_topic', default_value='/rotosy/camera'),
        DeclareLaunchArgument('serial_no', default_value=''),
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('fps', default_value='30'),
        DeclareLaunchArgument('align_depth_to_color', default_value='true'),

        Node(
            package='rotosy_calibration',
            executable='realsense_topic_publisher',
            name='realsense_topic_publisher',
            output='screen',
            parameters=[{
                'camera_name': LaunchConfiguration('camera_name'),
                'base_topic': LaunchConfiguration('base_topic'),
                'serial_no': LaunchConfiguration('serial_no'),
                'width': LaunchConfiguration('width'),
                'height': LaunchConfiguration('height'),
                'fps': LaunchConfiguration('fps'),
                'align_depth_to_color': LaunchConfiguration('align_depth_to_color'),
            }],
        ),
    ])
