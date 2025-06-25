from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    acm_number = LaunchConfiguration('acm_number')

    return LaunchDescription([
        DeclareLaunchArgument(
            'acm_number',
            default_value='2',
            description='ACM 디바이스 번호 (예: 2 -> /dev/ttyACM2)'
        ),

        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'micro-ros-agent', 'micro-ros-agent', 'serial',
                '--dev', ['/dev/ttyACM', acm_number],
                '-v6'
            ],
            output='screen'
        ),

        Node(
            package='application',
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
        ),
        Node(
            package='application',
            executable='button_node',
            name='button_node',
            # output='screen'
        ),
        # Node(
        #     package='application',
        #     executable='keyboard_node',
        #     name='keyboard_node',
        #     # output='screen'
        # ),
        Node(
            package='application',
            executable='hmi_planning_node',
            name='hmi_planning_node',
            output='screen'
        )
    ])
