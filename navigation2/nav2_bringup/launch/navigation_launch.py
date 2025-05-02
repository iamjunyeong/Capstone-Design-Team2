import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # ──────────── 기본 LC 설정 ────────────
    bringup_dir     = get_package_share_directory('nav2_bringup')
    namespace       = LaunchConfiguration('namespace')
    use_sim_time    = LaunchConfiguration('use_sim_time')
    autostart       = LaunchConfiguration('autostart')
    params_file     = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name  = LaunchConfiguration('container_name')
    container_full  = (namespace, '/', container_name)
    use_respawn     = LaunchConfiguration('use_respawn')
    log_level       = LaunchConfiguration('log_level')

    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'bt_navigator'
    ]

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={'use_sim_time': use_sim_time,
                            'autostart': autostart},
            convert_types=True),
        allow_substs=True)

    # ──────────── 개별 노드 (컴포지션 X) ────────────
    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            Node(package='nav2_controller', executable='controller_server',
                 output='screen', respawn=use_respawn, respawn_delay=2.0,
                 parameters=[configured_params],
                 arguments=['--ros-args', '--log-level', log_level],
                 remappings=remappings),

            Node(package='nav2_smoother', executable='smoother_server',
                 name='smoother_server', output='screen',
                 respawn=use_respawn, respawn_delay=2.0,
                 parameters=[configured_params],
                 arguments=['--ros-args', '--log-level', log_level],
                 remappings=remappings),

            Node(package='nav2_planner', executable='planner_server',
                 name='planner_server', output='screen',
                 respawn=use_respawn, respawn_delay=2.0,
                 parameters=[configured_params],
                 arguments=['--ros-args', '--log-level', log_level],
                 remappings=remappings),

            Node(package='nav2_bt_navigator', executable='bt_navigator',
                 name='bt_navigator', output='screen',
                 respawn=use_respawn, respawn_delay=2.0,
                 parameters=[configured_params],
                 arguments=['--ros-args', '--log-level', log_level],
                 remappings=remappings),
        ])

    # ──────────── 컴포저블 노드 (컨테이너) ────────────
    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_full,
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[configured_params],
                remappings=remappings),

            ComposableNode(
                package='nav2_smoother',
                plugin='nav2_smoother::SmootherServer',
                name='smoother_server',
                parameters=[configured_params],
                remappings=remappings),

            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[configured_params],
                remappings=remappings),

            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[configured_params],
                remappings=remappings),
        ])

    # ──────────── Launch 옵션 선언 ────────────
    lc = LaunchDescription()
    lc.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))
    for arg in [
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml')),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_composition', default_value='False'),
        DeclareLaunchArgument('container_name', default_value='nav2_container'),
        DeclareLaunchArgument('use_respawn', default_value='False'),
        DeclareLaunchArgument('log_level', default_value='info'),
    ]:
        lc.add_action(arg)

    lc.add_action(load_nodes)
    lc.add_action(load_composable_nodes)
    return lc
