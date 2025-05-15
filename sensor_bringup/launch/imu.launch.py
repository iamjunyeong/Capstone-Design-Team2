#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node

# this is the function launch  system will look for
def generate_launch_description():

    iahrs_driver_node = Node(
        package='iahrs_driver', 
        executable='iahrs_driver',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("iahrs_driver"),
                "config",
                "iahrs_driver_param.yaml"
            ])
        ]
    )

    # create and return launch description object
    return LaunchDescription(
        [
            iahrs_driver_node
        ]
    )
