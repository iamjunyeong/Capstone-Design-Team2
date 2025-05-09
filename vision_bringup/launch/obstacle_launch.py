from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    conda_python = '/home/loe/miniforge3/envs/grad/bin/python'
    pkg_path = os.path.join(
        os.getenv('HOME'),
        'workspace/github/Capstone-Design-Team2/vision_bringup/vision_bringup'
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=[conda_python, os.path.join(pkg_path, 'obstacle_detector.py')],
            output='screen'
        ),
        # ExecuteProcess(
        #     cmd=[conda_python, os.path.join(pkg_path, 'pcd_processs.py')],
        #     output='screen'
        # ),
        # ExecuteProcess(
        #     cmd=[conda_python, os.path.join(pkg_path, 'slic_process.py')],
        #     output='screen'
        # ),
        # 추가하고 싶다면 아래처럼 yolo_process.py도 등록 가능
        # ExecuteProcess(
        #     cmd=[conda_python, os.path.join(pkg_path, 'yolo_process.py')],
        #     output='screen'
        # ),
    ])
