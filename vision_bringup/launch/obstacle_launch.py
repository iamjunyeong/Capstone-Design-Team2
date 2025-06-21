from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

conda_python = '/home/ubuntu/miniforge3/envs/grad/bin/python'

def generate_launch_description():
    # USB 카메라 노드 2개
    usb_cam_0 = Node(
        package='usb_cam', executable='usb_cam_node_exe',
        namespace='/usb_cam_0',
        parameters=['/home/ubuntu/capstone_ws/src/Capstone-Design-Team2/usb_cam/config/params_1.yaml'],
        output='screen'
    )

    usb_cam_1 = Node(
        package='usb_cam', executable='usb_cam_node_exe',
        namespace='/usb_cam_1',
        parameters=['/home/ubuntu/capstone_ws/src/Capstone-Design-Team2/usb_cam/config/params_2.yaml'],
        output='screen'
    )


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
        usb_cam_0, usb_cam_1, *perception_nodes
    ])
