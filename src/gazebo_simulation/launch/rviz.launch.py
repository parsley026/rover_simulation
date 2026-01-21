from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('gazebo_simulation')
    pkg_project_description = get_package_share_directory('rover_description')

    config_path = LaunchConfiguration('config-path')
    config_path_launch_arg = DeclareLaunchArgument(
        'config-path',
        default_value=PathJoinSubstitution([pkg_project_description, 'config', 'config.rviz']),
        description='Path to the RViz config file'
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
           PathJoinSubstitution([pkg_ros_gz_sim, 'launch', '_rviz.py'])
        ),
        launch_arguments={
            'config-file': config_path,
        }.items()
    )

    return LaunchDescription([
        config_path_launch_arg,
        rviz_launch
    ])