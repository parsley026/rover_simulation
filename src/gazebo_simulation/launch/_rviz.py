from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

def generate_launch_description() -> LaunchDescription:
    pkg_project_description = get_package_share_directory('rover_description')

    config_file = LaunchConfiguration('config-file')

    config_file_launch_arg = DeclareLaunchArgument(
        'config-file',
        default_value=PathJoinSubstitution([pkg_project_description, 'config', 'config.rviz']),
        description='Path to RViz config file for RViz2'
    )

    rviz = Node(
       package='rviz2',
       executable='rviz2',
       name='rviz2',
       output='log',
       arguments=['-d', config_file],
    )

    return LaunchDescription([
        config_file_launch_arg,
        rviz
    ])



