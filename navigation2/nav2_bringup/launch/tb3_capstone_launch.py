#!/usr/bin/env python3
#
# TurtleBot3 Gazebo + Nav2 Capstone Launch
#

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
    GroupAction, SetEnvironmentVariable
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace

# ──────────── 0. 기본 경로 ────────────
tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
bringup_dir    = get_package_share_directory('nav2_bringup')
launch_dir     = os.path.join(bringup_dir, 'launch')

default_world  = os.path.join(bringup_dir, 'worlds', 'test_room.model')          # tb3_simulation 과 동일
# get_package_share_directory('nav2_bringup'), 'maps', 'map_filtered.yaml')       # map 변경
# get_package_share_directory('nav2_bringup'), 'maps', 'testmap_B_rotated.yaml')  # map 변경
default_map    = os.path.join(bringup_dir, 'maps', 'test_room.yaml')        # map 변경
default_params = os.path.join(bringup_dir, 'params', 'nav2_capstone_params.yaml')  # param 변경 예시
default_amcl   = os.path.join(bringup_dir, 'params', 'nav2_params.yaml')           # AMCL 포함본
default_rviz   = os.path.join(bringup_dir, 'rviz',  'nav2_default_view.rviz')      # rviz 변경 예시
default_sdf    = os.path.join(bringup_dir, 'worlds', 'waffle.model')               # tb3_simulation 과 동일
default_urdf   = os.path.join(bringup_dir, 'urdf',  'turtlebot3_waffle.urdf')

# ──────────── 1. Launch 인자 (tb3_simulation 스타일) ────────────
args = [
    ('namespace', ''), ('use_namespace', 'false'),
    ('slam', 'False'),
    ('map', default_map),
    ('use_sim_time', 'true'),
    ('params_file', default_params),
    ('amcl_params_file', default_amcl),     # ← AMCL 전용 YAML
    ('use_scan_filter', 'False'),           # ★ scan_filter_node 비활성화(기본값 False)
    ('autostart', 'true'),
    ('use_composition', 'False'),
    ('use_respawn', 'False'),
    ('rviz_config_file', default_rviz),
    ('use_simulator', 'True'),
    ('use_robot_state_pub', 'True'),
    ('use_rviz', 'True'),
    ('headless', 'False'),                  # GUI 표시
    ('world', default_world),
    ('robot_name', 'turtlebot3_waffle'),
    ('robot_sdf', default_sdf),
    # 로봇 초기 위치
    ('x_pose', '-4.00'), ('y_pose', '0.00'), ('z_pose', '0.01'),
    ('roll',  '0.00'), ('pitch','0.00'), ('yaw','0.00'),
]
declared = [DeclareLaunchArgument(k, default_value=v) for k, v in args]
lc = {k: LaunchConfiguration(k) for k, _ in args}

pose = {'x': lc['x_pose'], 'y': lc['y_pose'], 'z': lc['z_pose'],
        'R': lc['roll'],   'P': lc['pitch'], 'Y': lc['yaw']}

# ──────────── 2. Gazebo 서버·클라이언트 ────────────
start_gz_server = ExecuteProcess(
    condition=IfCondition(lc['use_simulator']),
    cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
         '-s', 'libgazebo_ros_factory.so', lc['world']],
    cwd=[launch_dir], output='screen')

start_gz_client = ExecuteProcess(
    condition=IfCondition(PythonExpression([lc['use_simulator'], ' and not ', lc['headless']])),
    cmd=['gzclient'], cwd=[launch_dir], output='screen')

# ──────────── 3. Robot State Publisher ────────────
with open(default_urdf, 'r') as f:
    robot_description = f.read()

robot_state_pub = Node(
    condition=IfCondition(lc['use_robot_state_pub']),
    package='robot_state_publisher', executable='robot_state_publisher',
    name='robot_state_publisher', namespace=lc['namespace'],
    output='screen',
    parameters=[{'use_sim_time': lc['use_sim_time'],
                 'robot_description': robot_description}],
    remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')])

# ──────────── 4. Gazebo 로봇 스폰 ────────────
spawn_entity = Node(
    package='gazebo_ros', executable='spawn_entity.py',
    output='screen',                             # 네임스페이스 생략 → 기본 ''
    arguments=['-entity', lc['robot_name'],
               '-file',   lc['robot_sdf'],
               '-robot_namespace', lc['namespace'],
               '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
               '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']])

# ──────────── 5. AMCL & Lifecycle (map→odom) ────────────
amcl_node = Node(
    package='nav2_amcl', executable='amcl', name='amcl',
    output='screen',
    parameters=[lc['amcl_params_file'], {'use_sim_time': lc['use_sim_time']}])

loc_manager = Node(
    package='nav2_lifecycle_manager', executable='lifecycle_manager',
    name='lifecycle_manager_localization', output='screen',
    parameters=[{'use_sim_time': lc['use_sim_time'],
                 'autostart': lc['autostart'],
                 'node_names': ['amcl']}])          # map_server 중복 관리 제거

# ──────────── 6. Nav2 Bring-up Include ────────────
nav2_include = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(launch_dir, 'nav2_bringup_launch.py')),
    launch_arguments={
        'namespace':       lc['namespace'],
        'use_namespace':   lc['use_namespace'],
        'map':             lc['map'],
        'use_sim_time':    lc['use_sim_time'],
        'params_file':     lc['params_file'],
        'use_scan_filter': lc['use_scan_filter'],   # ★ 파라미터 전달
        'autostart':       lc['autostart'],
        'use_composition': lc['use_composition'],
        'use_rviz':        lc['use_rviz'],
        'log_level':       'info'
    }.items())

# ──────────── 7. 런치 그룹 ────────────
launch_group = GroupAction([
    PushRosNamespace(lc['namespace'], condition=IfCondition(lc['use_namespace'])),
    start_gz_server, start_gz_client,
    spawn_entity, robot_state_pub,
    amcl_node, loc_manager,
    nav2_include
])

# ──────────── 8. LaunchDescription 반환 ────────────
def generate_launch_description():
    ld = LaunchDescription()
    for arg in declared:
        ld.add_action(arg)
    ld.add_action(launch_group)
    return ld
