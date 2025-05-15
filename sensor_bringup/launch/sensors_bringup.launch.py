from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    bringup_dir = get_package_share_directory('sensor_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    imu_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(launch_dir, 'imu.launch.py')
    ))

    gps_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(launch_dir, 'ublox_gps_node-launch.py')
    ))

    rtk_launcher = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(launch_dir, 'rtk_launcher.py')
    ))

    return LaunchDescription([
        imu_launch,
        gps_launch,
        TimerAction(
            period=3.0,
            actions=[rtk_launcher]
        )
    ])
