# Copyright (c) 2026. All rights reserved.
# Launch file for ROSA Command Center Gateway (AI-First)
# Starts rosbridge_websocket on port 9090, rosapi, the ROSA AI conversational agent, and the dedicated hardware diagnostic monitor.

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Rosbridge WebSocket server for React frontend communication on ws://localhost:9090
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{'port': 9090}],
            output='screen'
        ),
        # 2. ROS API helper node (allows ROSLIB.js to query active topic & service lists dynamically)
        Node(
            package='rosapi',
            executable='rosapi_node',
            name='rosapi_node',
            output='screen'
        ),
        # 3. Dedicated system & hardware diagnostic polling node (CPU, RAM, NVIDIA RTX 3060 VRAM)
        Node(
            package='rosa',
            executable='rosa_system_monitor',
            name='rosa_system_monitor',
            output='screen'
        ),
        # 4. ROSA AI conversational node (streaming tokens and tool execution traces)
        Node(
            package='rosa',
            executable='rosa_agent',
            name='rosa_agent',
            output='screen'
        ),
    ])
