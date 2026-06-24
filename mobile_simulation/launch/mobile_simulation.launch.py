import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _write_demo_map(context):
    del context
    out_dir = '/tmp/mobile_simulation'
    os.makedirs(out_dir, exist_ok=True)
    pgm_path = os.path.join(out_dir, 'mobile_map.pgm')
    yaml_path = os.path.join(out_dir, 'mobile_map.yaml')

    resolution = 0.05
    width = 240
    height = 160
    origin_x = -6.0
    origin_y = -4.0
    data = bytearray([254] * (width * height))

    def world_to_pixel(x, y):
        px = int((x - origin_x) / resolution)
        py = height - 1 - int((y - origin_y) / resolution)
        return px, py

    def rect(x1, y1, x2, y2):
        px1, py1 = world_to_pixel(min(x1, x2), max(y1, y2))
        px2, py2 = world_to_pixel(max(x1, x2), min(y1, y2))
        px1 = max(0, min(width - 1, px1))
        px2 = max(0, min(width - 1, px2))
        py1 = max(0, min(height - 1, py1))
        py2 = max(0, min(height - 1, py2))
        for py in range(py1, py2 + 1):
            start = py * width + px1
            data[start:start + (px2 - px1 + 1)] = bytes([0]) * (px2 - px1 + 1)

    rect(-6.0, 3.94, 6.0, 4.0)
    rect(-6.0, -4.0, 6.0, -3.94)
    rect(-6.0, -4.0, -5.94, 4.0)
    rect(5.94, -4.0, 6.0, 4.0)
    rect(-5.50, 1.45, -4.85, 1.55)   # 약재실 문 좌측벽 (문틈 1.1m로 확장)
    rect(-3.75, 1.45, -0.90, 1.55)   # 약재실 문 우측벽
    rect(-0.95, 1.50, -0.85, 4.00)
    rect(0.75, 1.65, 3.075, 1.75)    # A동 문 좌측벽 (문틈 1.1m로 확장)
    rect(4.175, 1.65, 5.25, 1.75)    # A동 문 우측벽
    rect(0.75, -1.75, 3.075, -1.65)  # B동 문 좌측벽 (문틈 1.1m로 확장)
    rect(4.175, -1.75, 5.25, -1.65)  # B동 문 우측벽
    rect(2.95, 1.75, 3.05, 3.95)
    rect(2.95, -3.95, 3.05, -1.75)

    # Static hospital fixtures and mobile obstacles represented in the Gazebo world.
    rect(-3.38, -2.29, -1.52, -2.01)  # nurse station front counter
    rect(-3.39, -3.21, -3.11, -2.15)  # nurse station side counter
    rect(-5.13, -2.88, -4.17, -2.52)  # waiting bench
    rect(-5.13, -3.48, -4.17, -3.12)  # waiting bench
    rect(-5.43, -2.28, -5.07, -1.92)  # plant pot
    rect(-2.86, 0.08, -2.24, 0.48)     # route tote stack
    rect(-1.50, -0.65, -0.85, -0.15)   # route supply cart
    rect(-0.65, 0.43, 0.25, 1.02)     # medicine cart in corridor
    rect(0.45, -1.26, 1.85, -0.58)    # stretcher
    rect(1.63, 0.56, 2.07, 1.00)      # equipment stand
    rect(4.65, 0.02, 5.25, 0.48)      # linen cart
    rect(-5.60, 3.10, -3.90, 3.40)    # drug storage back shelf
    rect(-5.60, 1.72, -5.30, 2.98)    # drug storage side shelf
    rect(-4.84, -0.84, -4.26, -0.44)  # staged medicine box
    rect(-4.24, -0.84, -3.66, -0.44)  # staged medicine box
    rect(3.86, 3.02, 4.94, 3.34)      # wing A medicine cabinet
    rect(1.16, 3.02, 2.24, 3.34)      # wing A medicine cabinet
    rect(4.72, 2.10, 5.18, 2.60)      # wing A cold storage
    rect(3.86, -3.34, 4.94, -3.02)    # wing B medicine cabinet
    rect(1.16, -3.34, 2.24, -3.02)    # wing B medicine cabinet
    rect(4.72, -2.60, 5.18, -2.10)    # wing B cold storage

    with open(pgm_path, 'wb') as f:
        f.write(f'P5\n{width} {height}\n255\n'.encode('ascii'))
        f.write(data)
    with open(yaml_path, 'w', encoding='utf-8') as f:
        f.write(
            f'image: {pgm_path}\n'
            f'mode: trinary\n'
            f'resolution: {resolution}\n'
            f'origin: [{origin_x}, {origin_y}, 0.0]\n'
            f'negate: 0\n'
            f'occupied_thresh: 0.65\n'
            f'free_thresh: 0.25\n'
        )
    return []


def generate_launch_description():
    os.environ.setdefault('TURTLEBOT3_MODEL', 'waffle')

    package_share = get_package_share_directory('mobile_simulation')
    gazebo_default_resource_path = '/usr/share/gazebo-11'
    gazebo_resource_path = f'{package_share}:{gazebo_default_resource_path}'
    existing_gazebo_resource_path = os.environ.get('GAZEBO_RESOURCE_PATH')
    if existing_gazebo_resource_path:
        gazebo_resource_path = f'{package_share}:{existing_gazebo_resource_path}:{gazebo_default_resource_path}'
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    turtlebot3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    turtlebot3_model = os.environ.get('TURTLEBOT3_MODEL', 'waffle')
    urdf_file_name = 'turtlebot3_' + turtlebot3_model + '.urdf'
    urdf_path = os.path.join(turtlebot3_gazebo_share, 'urdf', urdf_file_name)
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()
    turtlebot3_navigation_share = get_package_share_directory('turtlebot3_navigation2')

    existing_gazebo_model_path = os.environ.get('GAZEBO_MODEL_PATH')
    gazebo_model_path = os.path.join(package_share, 'models')
    if existing_gazebo_model_path:
        gazebo_model_path = f'{gazebo_model_path}:{existing_gazebo_model_path}'
    
    world = os.path.join(package_share, 'worlds', 'mobile_delivery.world')
    robot_sdf = os.path.join(package_share, 'models', 'mobile_delivery_robot', 'model.sdf')
    map_yaml = '/tmp/mobile_simulation/mobile_map.yaml'
    use_sim_time = LaunchConfiguration('use_sim_time')
    show_gazebo_client = LaunchConfiguration('show_gazebo_client')
    auto_start_demo = LaunchConfiguration('auto_start_demo')
    initial_x = LaunchConfiguration('initial_x')
    initial_y = LaunchConfiguration('initial_y')
    initial_yaw = LaunchConfiguration('initial_yaw')
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_yaw = LaunchConfiguration('goal_yaw')

    nav_params = os.path.join(package_share, 'config', 'nav2_params.yaml')

    # [TF 레이스 방지]
    # diff_drive(odom→base_footprint) TF가 흐른 뒤 nav2가 뜨도록 nav2_start_delay 적용.
    # AMCL 초기 위치는 nav2_params.yaml의 set_initial_pose:true 로 부팅 시 자동 설정됨.
    nav2_start_delay = 8.0

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('show_gazebo_client', default_value='true'),
        DeclareLaunchArgument('show_camera_view', default_value='true'),
        DeclareLaunchArgument('auto_start_demo', default_value='false'),
        DeclareLaunchArgument('initial_x', default_value='-4.30'),
        DeclareLaunchArgument('initial_y', default_value='2.5'),
        DeclareLaunchArgument('initial_yaw', default_value='-1.5708'),
        DeclareLaunchArgument('goal_x', default_value='4.20'),
        DeclareLaunchArgument('goal_y', default_value='2.72'),
        DeclareLaunchArgument('goal_yaw', default_value='1.5708'),
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle'),
        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', gazebo_resource_path),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path),
        OpaqueFunction(function=_write_demo_map),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': world}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros_share, 'launch', 'gzclient.launch.py')),
            condition=IfCondition(show_gazebo_client),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc,
            }],
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'mobile_delivery_robot',
                       '-file', robot_sdf,
                       '-x', initial_x,
                       '-y', initial_y,
                       '-z', '0.01',
                       '-Y', initial_yaw],
            output='screen'
        ),
        # Nav2 bringup: nav2_start_delay 후 시작 (Gazebo/diff_drive TF 안정화 대기)
        TimerAction(
            period=nav2_start_delay,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')),
                    launch_arguments={'use_sim_time': use_sim_time,
                                      'map': map_yaml,
                                      'params_file': nav_params,
                                      'autostart': 'True'}.items()
                ),
            ]
        ),
        # 단독 데모용 — auto_start_demo:=true 일 때만 하드코딩 골을 한 번 전송.
        Node(
            package='mobile_simulation',
            executable='demo_goal_sender',
            name='mobile_demo_goal_sender',
            output='screen',
            condition=IfCondition(auto_start_demo),
            parameters=[{'goal_x': goal_x,
                        'goal_y': goal_y,
                        'goal_yaw': goal_yaw},
                        {'use_sim_time': use_sim_time}]
        ),
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='rqt_image_view',
            arguments=['/camera/image_raw'],
            condition=IfCondition(LaunchConfiguration('show_camera_view')),
            output='screen'
        ),
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', os.path.join(package_share, 'resource', 'mobile_simulation.rviz')],
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     output='screen'
        # )
    ])
