import os
import yaml
from typing import Any
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_context import LaunchContext
from launch.conditions import IfCondition

def _check_float(s: str) -> bool:
    """
    """
    try:
        float(s)
        return True
    except ValueError:
        return False

def _parse_pose(s: str) -> list[str]:
    """
    """
    clean_string = s.strip().lower()
    coords = clean_string.split()
    if (not clean_string or len(coords) != 3 or 
        not _check_float(coords[0]) or not _check_float(coords[1]) or not _check_float(coords[2])):
        return ['0', '0', '1.5']
    return coords

def launch_setup(context: LaunchContext, *_, **__) -> list[Any]:
    """
    """
    use_sim_time = LaunchConfiguration('use_sim_time')

    local_bringup = LaunchConfiguration('local_bringup')
    
    entity_name = LaunchConfiguration('entity_name')
    pose = LaunchConfiguration('pose')
    model_coords = _parse_pose(pose.perform(context))

    mode = LaunchConfiguration('mode')

    payload_config = LaunchConfiguration('payload_config')
    
    run_kinematics = LaunchConfiguration('run_kinematics')
    run_kinematics_bridge = LaunchConfiguration('run_kinematics_bridge')

    run_arm_kinematics = LaunchConfiguration('run_arm_kinematics')
    run_arm_kinematics_bridge = LaunchConfiguration('run_arm_kinematics_bridge')

    payload_file = payload_config.perform(context)

    pkg_rover_bringup = get_package_share_directory('rover_bringup')

    payload_data = {}
    if os.path.exists(payload_file):
        try:
            with open(payload_file, 'r') as f:
                payload_data = yaml.safe_load(f) or {}
        except Exception:
            pass

    submodules = payload_data.get('submodules') or []

    submodule_ids = {s.get('id') for s in submodules if isinstance(s, dict) and 'id' in s}
    robotic_arm_state = 'robotic_arm' in submodule_ids

    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_rover_bringup, 'launch', 'bringup.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,

            'run_bridge': 'true',

            'entity_name': entity_name,

            'mode': mode,

            'run_kinematics': run_kinematics,
            'run_arm_kinematics': run_arm_kinematics,
            
            'run_kinematics_bridge': run_kinematics_bridge,
            'run_arm_kinematics_bridge': run_arm_kinematics_bridge,
            
            
            'payload_config': payload_config,
        }.items(),
        condition=IfCondition(local_bringup)
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", entity_name,
            "-topic", "/robot_description",
            "-x", model_coords[0],
            "-y", model_coords[1],
            "-z", model_coords[2],
        ],
        output="screen",
    )

    nodes = [robot_bringup, spawn_entity]

    # - - - ros2_control joint state broadcaster spawner
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # - - - ros2_control base controller spawner
    chassis_controllers_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'steer_front_left_controller',
            'steer_front_right_controller',
            'steer_rear_left_controller',
            'steer_rear_right_controller',

            'drive_front_left_controller',
            'drive_front_right_controller',
            'drive_rear_left_controller',
            'drive_rear_right_controller',

            '--controller-manager', '/controller_manager',
            '--activate-as-group',
        ],
        output='screen',
    )

    controller_nodes = [jsb_spawner, chassis_controllers_spawner]

    if robotic_arm_state == True:
        # - - - ros2_control robotic_arm controller spawner
        arm_controller_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['arm_controller', '--controller-manager', '/controller_manager'],
            output='screen',
        )

        # - - - ros2_control gripper controller spawner
        gripper_controller_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
            output='screen',
        )

        controller_nodes.extend([arm_controller_spawner, gripper_controller_spawner])        

    controllers_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=controller_nodes,
        )
    )
    nodes.append(controllers_handler)

    return nodes

def generate_launch_description() -> LaunchDescription:
    """
    """
    pkg_project_description = get_package_share_directory('rover_description')
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description=''),
        
        DeclareLaunchArgument('local_bringup', default_value='true', description=''),

        DeclareLaunchArgument('entity_name', default_value='rover', description=''),
        DeclareLaunchArgument('pose', default_value='0 0 1.5', description=''),

        DeclareLaunchArgument('mode', default_value='simulation', description=''),

        DeclareLaunchArgument('run_kinematics',        default_value='false', description=''),
        DeclareLaunchArgument('run_kinematics_bridge', default_value='true', description=''),

        DeclareLaunchArgument('run_arm_kinematics',        default_value='false', description=''),
        DeclareLaunchArgument('run_arm_kinematics_bridge', default_value='true', description=''),
        
        DeclareLaunchArgument('payload_config', default_value=PathJoinSubstitution([pkg_project_description, 'config', 'payload.yaml']), description=''),
        OpaqueFunction(function=launch_setup)
    ])