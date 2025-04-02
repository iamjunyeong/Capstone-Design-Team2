from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 각 런치 파일 경로
    bringup_pkg = get_package_share_directory('hardware_bringup')

    sensor_launch = os.path.join(bringup_pkg, 'launch', 'sensors_bringup.launch.py')
    control_launch = os.path.join(bringup_pkg, 'launch', 'control.launch.py')
    vision_launch = os.path.join(bringup_pkg, 'launch', 'vision.launch.py')
    description_launch = os.path.join(bringup_pkg, 'launch', 'hardware_description.launch.py')
    localization_launch = os.path.join(bringup_pkg, 'launch', 'localization.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(sensor_launch)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(control_launch)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(vision_launch)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(description_launch)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(localization_launch)),
    ])
