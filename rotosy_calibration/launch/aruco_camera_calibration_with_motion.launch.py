from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    selected_motion_path = PathJoinSubstitution([
        FindPackageShare('rotosy_calibration'),
        'config',
        'optimized_calibration_motion.yaml',
    ])

    args = [
        DeclareLaunchArgument('host', default_value='110.120.1.52'),
        DeclareLaunchArgument('port', default_value='12345'),
        DeclareLaunchArgument('mode', default_value='real'),
        DeclareLaunchArgument('model', default_value='e0509'),
        DeclareLaunchArgument('name', default_value='dsr01'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('show_rviz', default_value='false'),
        DeclareLaunchArgument('status_rate_hz', default_value='10.0'),
        DeclareLaunchArgument('motion_timeout', default_value='60.0'),
        DeclareLaunchArgument('camera_namespace', default_value='camera'),
        DeclareLaunchArgument('camera_name', default_value='camera'),
        DeclareLaunchArgument('show_camera_view', default_value='true'),
        DeclareLaunchArgument('shutdown_when_done', default_value='true'),
        DeclareLaunchArgument(
            'motion_config',
            default_value=selected_motion_path,
            description='YAML file containing safe calibration MoveJ poses.',
        ),
    ]

    calibration_config = PathJoinSubstitution([
        FindPackageShare('rotosy_calibration'),
        'config',
        'aruco_camera_calibration.yaml',
    ])
    motion_config = LaunchConfiguration('motion_config')

    dsr_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('dsr_bringup2'),
                'launch',
                'dsr_bringup2_rviz.launch.py',
            ])
        ]),
        launch_arguments={
            'host': LaunchConfiguration('host'),
            'port': LaunchConfiguration('port'),
            'mode': LaunchConfiguration('mode'),
            'model': LaunchConfiguration('model'),
            'name': LaunchConfiguration('name'),
            'gui': LaunchConfiguration('gui'),
            'status_rate_hz': LaunchConfiguration('status_rate_hz'),
            'motion_timeout': LaunchConfiguration('motion_timeout'),
        }.items(),
    )

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py',
            ])
        ]),
        launch_arguments={
            'camera_namespace': LaunchConfiguration('camera_namespace'),
            'camera_name': LaunchConfiguration('camera_name'),
            'enable_color': 'true',
            'enable_depth': 'false',
            'publish_tf': 'false',
            'initial_reset': 'true',
        }.items(),
    )

    calibrator = TimerAction(
        period=14.0,
        actions=[
            Node(
                package='rotosy_calibration',
                executable='aruco_camera_calibrator',
                name='aruco_camera_calibrator',
                output='screen',
                parameters=[calibration_config, {'sample_mode': 'service'}],
            )
        ],
    )

    arm_controller = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='doosan_controller',
                executable='arm_controller',
                name='arm_controller',
                output='screen',
                parameters=[{
                    'robot_ns': LaunchConfiguration('name'),
                    'status_rate_hz': LaunchConfiguration('status_rate_hz'),
                    'motion_timeout': LaunchConfiguration('motion_timeout'),
                    'servo_on_retries': 3,
                }],
            )
        ],
    )

    motion_runner_node = Node(
        package='rotosy_calibration',
        executable='calibration_motion_runner',
        name='calibration_motion_runner',
        output='screen',
        parameters=[motion_config],
    )

    motion_runner = TimerAction(
        period=24.0,
        actions=[motion_runner_node],
    )

    image_view = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='rqt_image_view',
                executable='rqt_image_view',
                name='calibration_camera_view',
                output='screen',
                arguments=['/camera/camera/color/image_raw'],
                condition=IfCondition(LaunchConfiguration('show_camera_view')),
            )
        ],
    )

    shutdown_on_motion_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=motion_runner_node,
            on_exit=[
                EmitEvent(
                    event=Shutdown(reason='calibration motion runner finished'),
                    condition=IfCondition(LaunchConfiguration('shutdown_when_done')),
                )
            ],
        )
    )

    return LaunchDescription(
        args + [dsr_bringup, realsense, image_view, calibrator, arm_controller, motion_runner, shutdown_on_motion_exit]
    )
