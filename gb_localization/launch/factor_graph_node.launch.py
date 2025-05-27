from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gb_localization',
            executable='factor_graph_node',
            name='factor_graph_node',
            output='screen',
            # parameters=[  # 파라미터가 있다면 여기에 추가
            #     # {'param_name': value}
            # ],
            # remappings=[
            #     # ('/original/topic', '/remapped/topic')
            # ]
        )
    ])