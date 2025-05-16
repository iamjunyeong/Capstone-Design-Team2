from launch import LaunchDescription
from launch_ros.actions import Node

conda_python = '/home/ubuntu/miniforge3/envs/grad/bin/python'

def generate_launch_description():
    common = dict(prefix=conda_python, output='screen')

    return LaunchDescription([
        Node(package='vision_bringup', executable='slic_process',      **common),
        Node(package='vision_bringup', executable='pcd_process',       **common),
        Node(package='vision_bringup', executable='yolo_process',      **common),
        Node(package='vision_bringup', executable='obstacle_detector', **common),
    ])
