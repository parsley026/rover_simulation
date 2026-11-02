import os
from typing import Any
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_context import LaunchContext
from launch_ros.actions import Node


def launch_setup(context: LaunchContext, *_, **__) -> list[Any]:
    """
    """
    use_sim_time              = LaunchConfiguration('use_sim_time')
    
    mount_height              = LaunchConfiguration('mount_height')
    
    include_sensors           = LaunchConfiguration('include_sensors')
    
    submodule                 = LaunchConfiguration('submodule').perform(context).strip()
    
    run_arm_kinematics        = LaunchConfiguration('run_arm_kinematics')
    run_arm_kinematics_bridge = LaunchConfiguration('run_arm_kinematics_bridge')

    pkg_rover_bringup     = get_package_share_directory('rover_bringup')
    pkg_rover_description = get_package_share_directory('rover_description')

    # - - - submodule bringup
    submodule_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_rover_bringup, 'launch', 'submodule_bringup.launch.py'])
        ),
        launch_arguments={
            'use_sim_time':              use_sim_time,
            
            'mount_height':              mount_height,
            
            'include_sensors':           include_sensors,
            
            'submodule':                 submodule,
            
            'run_arm_kinematics':        run_arm_kinematics,
            'run_arm_kinematics_bridge': run_arm_kinematics_bridge,
        }.items()
    )

    # - - - spawn entity
    spawn_entity = Node(
        package   ='ros_gz_sim',
        executable='create',
        arguments=[
            '-name',  submodule,
            '-topic', '/robot_description',
            '-x', '0', '-y', '0', '-z', '0',
        ],
        output='screen',
    )

    nodes = [submodule_bringup, spawn_entity]

    # - - - submodule-specific nodes
    if submodule == 'none':
        pass

    elif submodule == 'autonomy':
        
        # - - sensors bridge
        autonomy_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{
                'config_file': PathJoinSubstitution([pkg_rover_description, 'config', 'bridges', '_bridge_autonomy.yaml']),
                'use_sim_time': use_sim_time,
            }],
            output='screen',
        )

        if include_sensors.perform(context).lower() == 'true':
            nodes.append(autonomy_bridge)

    elif submodule == 'robotic_arm':

        controller_nodes = []
        # - - - ros2_control joint state broadcaster spawner
        arm_jsb_spawner = Node(
            package   ='controller_manager',
            executable='spawner',
            arguments =['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output    ='screen',
        )
        controller_nodes.append(arm_jsb_spawner)
        
        # - - - ros2_control robotic_arm controller spawner
        arm_controller_spawner = Node(
            package   ='controller_manager',
            executable='spawner',
            arguments =['arm_controller', '--controller-manager', '/controller_manager'],
            output    ='screen',
        )
        controller_nodes.append(arm_controller_spawner)

        # - - - ros2_control gripper controller spawner
        gripper_controller_spawner = Node(
            package   ='controller_manager',
            executable='spawner',
            arguments =['gripper_controller', '--controller-manager', '/controller_manager'],
            output    ='screen',
        )
        controller_nodes.append(gripper_controller_spawner)

        # - - sensors bridge
        wrist_camera_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{
                'config_file': PathJoinSubstitution([pkg_rover_description, 'config', 'bridges', '_bridge_arm.yaml']),
                'use_sim_time': use_sim_time,
            }],
            output='screen',
        )
        
        controllers_handler = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=controller_nodes,
            )
        )
        nodes.append(controllers_handler)

        if include_sensors.perform(context).lower() == 'true':
            nodes.append(wrist_camera_bridge)

    return nodes


def generate_launch_description() -> LaunchDescription:
    """
    """
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',              default_value='true',        description=''),
        
        DeclareLaunchArgument('mount_height',              default_value='0.5',         description=''),
        
        DeclareLaunchArgument('include_sensors',           default_value='true',        description=''),
        
        DeclareLaunchArgument('submodule',                 default_value='robotic_arm', description=''),

        DeclareLaunchArgument('run_arm_kinematics',        default_value='false',       description=''),
        DeclareLaunchArgument('run_arm_kinematics_bridge', default_value='false',       description=''),
        OpaqueFunction(function=launch_setup),
    ])
