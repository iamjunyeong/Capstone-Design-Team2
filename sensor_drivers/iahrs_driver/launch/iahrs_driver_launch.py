from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="iahrs_driver",
            executable="driver",  # 실행되는 Python 클래스 내 이름
            name="iahrs_driver_node",
            output="screen",
            # parameters=[
            #     {"tf_prefix": ""},
            #     {"send_tf": True}

            # ]
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare("iahrs_driver"),
                    "config",
                    "iahrs_params.yaml"
                ])
            ]
        )
    ])
