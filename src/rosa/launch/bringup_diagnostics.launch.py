#  Copyright (c) 2026. All rights reserved.
#  Launch file for ROSA Tool Diagnostics Bringup.
#  Usage: ros2 launch rosa bringup_diagnostics.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'run', 'rosa', 'rosa_diagnostics_bringup'],
            output='screen',
            name='rosa_diagnostics_bringup'
        )
    ])
