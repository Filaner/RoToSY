from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config = PathJoinSubstitution([
        FindPackageShare('rotosy_calibration'),
        'config',
        'aruco_camera_calibration.yaml',
    ])

    return LaunchDescription([
        Node(
            package='rotosy_calibration',
            executable='aruco_camera_calibrator',
            name='aruco_camera_calibrator',
            output='screen',
            parameters=[config],
        ),
    ])
