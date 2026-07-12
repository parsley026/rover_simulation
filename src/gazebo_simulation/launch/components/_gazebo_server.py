from typing import Any
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_context import LaunchContext

def launch_setup(context: LaunchContext, *_, **__) -> list[Any]:
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_file = LaunchConfiguration('world-file')
    headless = LaunchConfiguration('headless')
    check_headless = headless.perform(context).strip().lower() in ('1', 'true')

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': ['-r -v4 ', '-s ' if check_headless else '', world_file]}.items(),
    )

    return [gazebo_sim]

def generate_launch_description() -> LaunchDescription:
    pkg_project_gazebo_worlds = get_package_share_directory('gazebo_worlds')

    return LaunchDescription([
        DeclareLaunchArgument('world-file', default_value=PathJoinSubstitution([pkg_project_gazebo_worlds, 'worlds', 'rex.sdf']), description='Path to SDF world file'),
        DeclareLaunchArgument('headless', default_value='false', description='Run without GUI'),
        OpaqueFunction(function=launch_setup)
    ])