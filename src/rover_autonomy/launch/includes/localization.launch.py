import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    ekf_config_path = LaunchConfiguration('params_file').perform(context)

    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'),
        value_type=bool
    )

    return [
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            namespace='localization',
            output='screen',
            parameters=[ekf_config_path, {'use_sim_time': use_sim_time}],
        ),
    ]

def generate_launch_description():
    pkg_share = FindPackageShare('rover_autonomy')
    default_params_file = PathJoinSubstitution([pkg_share, 'config', 'localization', 'ukf_filter.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params_file, description=''),
        
        DeclareLaunchArgument('use_sim_time', default_value='false', description=''),
        OpaqueFunction(function=launch_setup)
    ])
