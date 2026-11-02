import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time   = LaunchConfiguration('use_sim_time')
    payload_config = LaunchConfiguration('payload_config')
    
    # Path to the bringup launch file
    bringup_launch_path = PathJoinSubstitution([
        FindPackageShare('rover_bringup'), 'launch', 'bringup.launch.py'
    ])

    # Include the bringup (which starts robot_state_publisher)
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch_path),
        launch_arguments={
            'use_sim_time':              use_sim_time,
            'payload_config':            payload_config,
            'run_kinematics_bridge':     'false',
            'run_arm_kinematics_bridge': 'false',
            'run_kinematics':            'false',
        }.items()
    )

    # Start RViz
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('rover_description'), 'rviz', 'urdf.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Joint State Publisher GUI (for viewing standalone)
    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'payload_config',
            default_value=PathJoinSubstitution([FindPackageShare('rover_description'), 'config', 'payload.yaml']),
            description='Path to payload.yaml configuration file'
        ),
        bringup_cmd,
        jsp_gui_node,
        rviz_node
    ])
