from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    param_file = LaunchConfiguration('param_file')
    return LaunchDescription([
        DeclareLaunchArgument(
            'param_file',
            default_value=PathJoinSubstitution([
                FindPackageShare("gb_localization"),
                "config",
                "factor_graph.yaml"
            ]),
            description='Path to the parameter file to use.'
        ),
        Node(
            package='gb_localization',
            executable='factor_graph_node',
            name='factor_graph_node',
            output='screen',
            parameters=[param_file],
            # remappings=[
            #     # ('/original/topic', '/remapped/topic')
            # ]
        )
    ])