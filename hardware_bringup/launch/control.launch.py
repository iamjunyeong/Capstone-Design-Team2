# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package='custom_micro_ros',
#             executable='serial_bridge_node',  # 설치된 entry point 이름
#             name='serial_bridge_node',
#             output='screen'
#         )
#     ])


from launch import LaunchDescription

def generate_launch_description():
    return LaunchDescription([])
