from launch import LaunchDescription
from launch_ros.actions import Node
import os

conda_python = '/home/ubuntu/miniforge3/envs/grad/bin/python'

def generate_launch_description():
    # Vision 관련 노드들 (YOLO, SLIC, PCD, Obstacle)
    common = dict(prefix=conda_python, output='screen')

    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        parameters=[{
            'enable_color': True,
            'enable_depth': True,
            'align_depth.enable': True,
            'enable_infra1': False,
            'enable_infra2': False
        }],
        output='screen'
    )

    perception_nodes = [    
        Node(package='vision_bringup', executable='slic_process',      **common),
        Node(package='vision_bringup', executable='pcd_process',       **common),
        Node(package='vision_bringup', executable='yolo_process',      **common),
        Node(package='vision_bringup', executable='obstacle_detector', **common),
        Node(package='vision_bringup', executable='obstacle_detector_crosswalk', **common),
    ]

    return LaunchDescription([
        realsense_node,
        *perception_nodes
    ])
