import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('rover_topography')
    default_config = os.path.join(pkg_dir, 'config', 'topography_params.yaml')
    local_filters = os.path.join(pkg_dir, 'config', 'local_mapping_filters.yaml')
    global_filters = os.path.join(pkg_dir, 'config', 'global_mapping_filters.yaml')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Path to parameter YAML config file for rover topography nodes'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use ROS simulation clock if true'
    )

    local_node = Node(
        package='rover_topography',
        executable='topography_node',
        name='local_topography_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config'), 
            local_filters, 
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    global_node = Node(
        package='rover_topography',
        executable='topography_node',
        name='global_topography_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config'), 
            global_filters, 
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    return LaunchDescription([
        config_arg,
        use_sim_time_arg,
        local_node,
        global_node
    ])
