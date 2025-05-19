import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes, PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

# Humble에서는 RewrittenYaml 생성자에 필수 인자인 param_rewrites를 사용합니다.
from nav2_common.launch import RewrittenYaml

# 기본 파일 경로들
default_params_file = os.path.join(get_package_share_directory('nav2_bringup'),
                                   'params', 'nav2_params.yaml')
default_map_file = os.path.join(get_package_share_directory('nav2_bringup'),
                                'maps', 'turtlebot3_world.yaml')
default_rviz_config_file = os.path.join(get_package_share_directory('nav2_bringup'),
                                        'rviz', 'nav2_default_view.rviz')

def generate_launch_description():
    # 1. Launch Argument 선언
    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Robot namespace'
    )
    use_namespace_arg = DeclareLaunchArgument(
        'use_namespace', default_value='False',
        description='Apply namespace?'
    )
    # 여기서는 use_composition 기본값을 False로 설정하여 개별 노드를 확인할 수 있도록 함(원래 tb3_simulation_launch.py와 달리)
    declare_use_composition_arg = DeclareLaunchArgument(
        'use_composition', default_value='False',
        description='Use composed bringup (if False, nodes are launched individually)'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock'
    )
    autostart_arg = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup nav2'
    )
    params_file_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params_file,
        description='Path to Nav2 params YAML'
    )
    map_yaml_file_arg = DeclareLaunchArgument(
        'map', default_value=default_map_file,
        description='Path to map YAML'
    )
    # RViz 관련 인자들
    rviz_config_file_arg = DeclareLaunchArgument(
        'rviz_config_file', default_value=default_rviz_config_file,
        description='Path to RViz config file to use'
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='True',
        description='Whether to run RViz'
    )
    log_level_arg = DeclareLaunchArgument(          # 추가: 로그 레벨 설정
        'log_level', default_value='info',
        description='Console log level for additional nodes'
    )

    # 2. LaunchConfiguration 변수들
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_composition = LaunchConfiguration('use_composition')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    map_yaml_file = LaunchConfiguration('map')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')
    log_level = LaunchConfiguration('log_level')     # 추가

    # 3. RewrittenYaml: 파라미터 파일로부터 읽어오면서 'use_sim_time'과 'yaml_filename' 값을 치환.
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites={
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml_file
        },
        convert_types=True
    )

    # 4. Nav2 Container (컴포지션 사용 시)
    nav2_container = ComposableNodeContainer(
        name='nav2_container',
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container_isolated',
        output='screen'
    )

    # 5. Map Server
    # 원래 방식: map_server는 configured_params만 전달받음.
    map_server_composable = ComposableNode(
        package='nav2_map_server',
        plugin='nav2_map_server::MapServer',
        name='map_server',
        parameters=[configured_params],
    )
    load_map_server = LoadComposableNodes(
        condition=IfCondition(use_composition),
        composable_node_descriptions=[map_server_composable],
        target_container=nav2_container
    )
    map_server_node = Node(
        condition=UnlessCondition(use_composition),
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[configured_params]
    )

    # 6. Include navigation_launch.py (내비게이션 관련 노드들)
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    navigation_launch_file = os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
    navigation_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_file),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': use_composition,
            'container_name': 'nav2_container'
        }.items()
    )

    # 7. Lifecycle Manager: 모든 Nav2 라이프사이클 노드를 autostart에 따라 전환
    lifecycle_manager_nav = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'autostart': autostart,
                     'node_names': [
                         'controller_server',
                         'smoother_server',
                         'planner_server',
                         'behavior_server',
                         'bt_navigator',
                         'velocity_smoother',
                         'map_server'
                     ]}]
    )

    # 8. Include RViz launch: tb3_simulation_launch.py에서는 rviz_launch.py를 Include하여 RViz를 실행합니다.
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'rviz_config': rviz_config_file
        }.items()
    )

    twist_to_ackermann_node = Node(                # 추가: twist_to_ackermann_node 노드 실행
        package='twist_to_ackermann',               # 필요 시 실제 패키지명으로 수정
        executable='twist_to_ackermann_node',
        name='twist_to_ackermann_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=[('nav_vel', 'cmd_vel')]
    )

    # 9. 전체 브링업 그룹: 모든 액션들을 하나의 GroupAction으로 묶음
    bringup_group = GroupAction([
        PushRosNamespace(
            namespace=namespace,
            condition=IfCondition(use_namespace)
        ),
        nav2_container,
        load_map_server,
        map_server_node,
        navigation_include,
        lifecycle_manager_nav,
        twist_to_ackermann_node          # 추가
    ])

    return LaunchDescription([
        namespace_arg,
        use_namespace_arg,
        declare_use_composition_arg,
        use_sim_time_arg,
        autostart_arg,
        params_file_arg,
        map_yaml_file_arg,
        rviz_config_file_arg,
        use_rviz_arg,
        log_level_arg,                   # 추가
        bringup_group,
        rviz_cmd  # RViz 실행 액션 추가
    ])
