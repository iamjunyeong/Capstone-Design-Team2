from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # iahrs_driver 패키지의 설치 경로를 가져옴
    iahrs_pkg_dir = get_package_share_directory('iahrs_driver')

    # 만약 설정 파일이 추가될 경우를 대비해 경로를 미리 정의 (예시)
    # param_file = os.path.join(iahrs_pkg_dir, 'config', 'iahrs.yaml')

    return LaunchDescription([
        Node(
            package='iahrs_driver',
            executable='driver',
            name='iahrs_driver_node',
            output='screen',
            parameters=[
                {"tf_prefix": ""},
                {"send_tf": True},
                # param_file  ← 향후 config 파일을 사용할 경우 리스트에 추가
            ]
        )
    ])
