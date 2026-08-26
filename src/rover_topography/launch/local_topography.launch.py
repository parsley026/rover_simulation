import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('rover_topography')
    default_config = os.path.join(pkg_dir, 'config', 'topography_params.yaml')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Path to parameter YAML config file for rover topography nodes'
    )

    local_filters = os.path.join(pkg_dir, 'config', 'local_mapping_filters.yaml')

    local_node = Node(
        package='rover_topography',
        executable='topography_node',
        name='local_topography_node',
        output='screen',
        parameters=[LaunchConfiguration('config'), local_filters]
    )

    return LaunchDescription([
        config_arg,
        local_node
    ])
