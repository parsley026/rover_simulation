from typing import Any

from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.launch_context import LaunchContext
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue

def _check_float(s: str) -> bool:
    try:
        float(s)
        return True
    except ValueError:
        return False

def _parse_pose(s: str) -> list[str]:
    clean_string = s.strip().lower()

    coords = clean_string.split(" ")
    if (not clean_string
        or not len(coords) == 3
        or not _check_float(coords[0])
        or not _check_float(coords[1])
        or not _check_float(coords[2])):
        return ['0', '0', '10']
    
    return coords

def launch_setup(context: LaunchContext, *_, **__) -> list[Any]:

    model_file = LaunchConfiguration('model-file')
    pose = LaunchConfiguration('pose')
    reset_robot_arg = LaunchConfiguration('reset-robot')

    model_file_desc = model_file.perform(context)
    model_coords = _parse_pose(pose.perform(context))

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{
            'use_sim_time': True,
            'robot_description': ParameterValue(Command(['xacro ', model_file_desc]), value_type=str)
        }]
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "rex",
            "-topic",
            "/robot_description",
            "-x",
            model_coords[0],
            "-y",
            model_coords[1],
            "-z",
            model_coords[2],
        ],
        output="screen",
    )

    reset_robot = Node(
        package='ros2cli',
        executable='ros2',
        arguments=[
            'service', 'call', 
            '/world/default/reset', 
            'ros_gz_sim_msgs/srv/ResetWorld', 
            '{all:true}'
        ],
        condition=IfCondition(reset_robot_arg),
        output='screen'
    )

    return [robot_state_publisher, spawn_entity, reset_robot]


def generate_launch_description() -> LaunchDescription:
    pkg_project_description = get_package_share_directory('rover_description')
    
    model_file_launch_arg = DeclareLaunchArgument(
        'model-file',
        default_value=PathJoinSubstitution([pkg_project_description, 'urdf', 'rex.urdf']),
        description='Path to URDF model file'
    )

    pose_launch_arg = DeclareLaunchArgument(
        'pose',
        default_value='0 0 0.36',
        description='XYZ coordinates of the running model'
    )

    reset_robot_arg = DeclareLaunchArgument(
        'reset-robot', 
        default_value='false',
        description='Reset the robot in Gazebo.'
    )

    opaque_func = OpaqueFunction(function=launch_setup)

    return LaunchDescription([model_file_launch_arg, pose_launch_arg, reset_robot_arg, opaque_func])

