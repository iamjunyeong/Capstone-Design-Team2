from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='application',  # 🔁 여기를 실제 패키지 이름으로 변경
            executable='stt_node',
            name='stt_node',
            output='screen'
        ),
        Node(
            package='application',
            executable='intent_node',
            name='intent_node',
            output='screen'
        ),
        Node(
            package='application',
            executable='tts_node',
            name='tts_node',
            output='screen'
        )
    ])

