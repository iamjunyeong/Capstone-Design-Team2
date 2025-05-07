from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vision_bringup', executable='obstacle_detector', name='obstacle_detector',
            parameters=[{'camera_topics':['/camera','/usb_cam1','/usb_cam2']}]
        )
    ])