import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes, PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml                 # Humble에서는 param_rewrites 필수

# ──────────── 기본 파일 경로 ────────────
default_params_file = os.path.join(                 # param 변경
    get_package_share_directory('nav2_bringup'), 'params', 'nav2_capstone_params.yaml')                   
    # get_package_share_directory('nav2_bringup'), 'params', 'nav2_params_costmap.yaml')

default_map_file = os.path.join(                    # map 변경
    # get_package_share_directory('nav2_bringup'), 'maps', 'map_filtered.yaml')       
    # get_package_share_directory('nav2_bringup'), 'maps', 'testmap_B.yaml')
    # get_package_share_directory('nav2_bringup'), 'maps', 'konkuk_map.yaml')
    # get_package_share_directory('nav2_bringup'), 'maps', 'no_tf_test_map.yaml')
    # get_package_share_directory('nav2_bringup'), 'maps', 'map_7.yaml')
    # get_package_share_directory('nav2_bringup'), 'maps', 'map_8.yaml')
    get_package_share_directory('nav2_bringup'), 'maps', 'map_s3.yaml')
    # get_package_share_directory('nav2_bringup'), 'maps', 'map_final.yaml')
    #get_package_share_directory('nav2_bringup'), 'maps', 'map_final1.yaml')
    # get_package_share_directory('nav2_bringup'), 'maps', 'map_final2.yaml')

default_rviz_config_file = os.path.join(            # rviz 변경
    # get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')    
    get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_capstone_view.rviz')


def generate_launch_description():
    # ──────────── 1. Launch 인자 ────────────
    namespace_arg       = DeclareLaunchArgument('namespace',           default_value='')
    use_namespace_arg   = DeclareLaunchArgument('use_namespace',       default_value='False')
    use_composition_arg = DeclareLaunchArgument('use_composition',     default_value='False')
    use_sim_time_arg    = DeclareLaunchArgument('use_sim_time',        default_value='False')  # use_sim_time 변경
    autostart_arg       = DeclareLaunchArgument('autostart',           default_value='true')
    params_file_arg     = DeclareLaunchArgument('params_file',         default_value=default_params_file)
    map_yaml_file_arg   = DeclareLaunchArgument('map',                 default_value=default_map_file)
    rviz_config_file_arg= DeclareLaunchArgument('rviz_config_file',    default_value=default_rviz_config_file)
    use_rviz_arg        = DeclareLaunchArgument('use_rviz',            default_value='True')
    log_level_arg       = DeclareLaunchArgument('log_level',           default_value='info')
    use_scan_filter_arg = DeclareLaunchArgument('use_scan_filter',     default_value='True')

    # ──────────── 2. LaunchConfiguration ────────────
    namespace        = LaunchConfiguration('namespace')
    use_namespace    = LaunchConfiguration('use_namespace')
    use_composition  = LaunchConfiguration('use_composition')
    use_sim_time     = LaunchConfiguration('use_sim_time')
    autostart        = LaunchConfiguration('autostart')
    params_file      = LaunchConfiguration('params_file')
    map_yaml_file    = LaunchConfiguration('map')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz         = LaunchConfiguration('use_rviz')
    log_level        = LaunchConfiguration('log_level')
    use_scan_filter  = LaunchConfiguration('use_scan_filter')

    # ──────────── 3. 파라미터 치환 ────────────
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites={'use_sim_time': use_sim_time,
                        'yaml_filename': map_yaml_file},
        convert_types=True)

    # ──────────── 4. Nav2 컨테이너 (컴포지션) ────────────
    nav2_container = ComposableNodeContainer(
        name='nav2_container',
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container_isolated',
        output='screen')

    # ──────────── 5. Map Server ────────────
    map_server_composable = ComposableNode(
        package='nav2_map_server',
        plugin='nav2_map_server::MapServer',
        name='map_server',
        parameters=[configured_params])

    load_map_server = LoadComposableNodes(
        condition=IfCondition(use_composition),
        composable_node_descriptions=[map_server_composable],
        target_container=nav2_container)

    map_server_node = Node(
        condition=UnlessCondition(use_composition),
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[configured_params])

    # ──────────── 6. navigation_launch.py Include ────────────
    bringup_dir = get_package_share_directory('nav2_bringup')
    navigation_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time'  : use_sim_time,
            'params_file'   : params_file,
            'autostart'     : autostart,
            'use_composition': use_composition,
            'container_name': 'nav2_container',
            'log_level'     : log_level,
            'use_scan_filter': use_scan_filter
        }.items())

    # ──────────── 7. Lifecycle Manager (단일) ────────────
    lifecycle_manager_nav = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart'   : autostart,
            'node_names'  : [
                'map_server',
                'controller_server',
                'smoother_server',
                'planner_server',
                'bt_navigator'
            ]
        }])

    # ──────────── 8. RViz ────────────
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={
            'namespace'    : namespace,
            'use_namespace': use_namespace,
            'rviz_config'  : rviz_config_file
        }.items())

    # ──────────── 9. Twist→Ackermann 변환 노드 ────────────
    twist_to_ackermann_node = Node(
        package='twist_to_ackermann',            # 실제 패키지명 확인
        executable='twist_to_ackermann_node',
        name='twist_to_ackermann_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=[('nav_vel', 'cmd_vel')])

    # ──────────── 10. 신규 유틸리티 노드 6종 ────────────
    obstacle_layer_toggler_node = Node(
        package='obstacle_layer_toggler',
        executable='toggle_obstacle_layer',
        name='obstacle_layer_toggler',
        output='screen')

    costmap_clear_timer_node = Node(
        package='costmap_clearing',
        executable='clear_costmap_timer',
        name='clear_costmap_timer',
        output='screen')
    
    scan_filter_node = Node(                                         # ← 실행 조건 추가
        package='scan_filter_pkg',
        executable='scan_filter_node',
        name='scan_filter_node',
        output='screen',
        remappings=[('/scan_input', '/scan')],
        condition=IfCondition(use_scan_filter))                      # ★ 추가
    
    hmi_angle_node = Node(
        package='ackermann_hmi',
        executable='hmi_angle_node',
        name='hmi_angle_node',
        output='screen')

    obstacle_controller_param_updater_node = Node(                  # ★ 추가
        package='obstacle_controller_param_updater',
        executable='obstacle_controller_param_updater',
        name='obstacle_controller_param_updater',
        output='screen')

    # crosswalk_supervisor_node = Node(                               # ★ 추가
    #     package='nav2_crosswalk_supervisor',
    #     executable='crosswalk_supervisor',
    #     name='crosswalk_supervisor',
    #     output='screen')

    # ──────────── 11. 그룹화 ────────────
    bringup_group = GroupAction([
        PushRosNamespace(namespace, condition=IfCondition(use_namespace)),
        nav2_container,
        load_map_server,
        map_server_node,
        navigation_include,
        lifecycle_manager_nav,
        rviz_cmd,
        twist_to_ackermann_node,
        obstacle_layer_toggler_node,
        costmap_clear_timer_node,
        scan_filter_node,          # ← 조건 적용
        hmi_angle_node,
        obstacle_controller_param_updater_node  # ★ 추가
        # crosswalk_supervisor_node                # ★ 추가
    ])

    # ──────────── 12. LaunchDescription 반환 ────────────
    return LaunchDescription([
        namespace_arg, use_namespace_arg, use_composition_arg, use_sim_time_arg,
        autostart_arg, params_file_arg, map_yaml_file_arg, rviz_config_file_arg,
        use_rviz_arg, log_level_arg, use_scan_filter_arg,
        bringup_group
    ])
