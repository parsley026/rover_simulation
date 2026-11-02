import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Bringup for standalone robotic arm (alias wrapping submodule_bringup.launch.py).
    """
    pkg_rover_bringup = FindPackageShare('rover_bringup')

    use_sim_time              = LaunchConfiguration('use_sim_time')
    mount_height              = LaunchConfiguration('mount_height')
    run_arm_kinematics        = LaunchConfiguration('run_arm_kinematics')
    run_arm_kinematics_bridge = LaunchConfiguration('run_arm_kinematics_bridge')

    submodule_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_rover_bringup, 'launch', 'submodule_bringup.launch.py'])
        ),
        launch_arguments={
            'submodule':                 'robotic_arm',
            'use_sim_time':              use_sim_time,
            'mount_height':              mount_height,
            'run_arm_kinematics':        run_arm_kinematics,
            'run_arm_kinematics_bridge': run_arm_kinematics_bridge,
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',              default_value='true',  description='Use simulation clock'),
        DeclareLaunchArgument('mount_height',              default_value='0.8',   description='Z height of arm mount above world frame (m)'),
        DeclareLaunchArgument('run_arm_kinematics',        default_value='false', description='Launch rover_robot_arm_kinematics node'),
        DeclareLaunchArgument('run_arm_kinematics_bridge', default_value='false', description='Launch rover_robot_arm_kinematics_bridge'),
        submodule_bringup,
    ])
