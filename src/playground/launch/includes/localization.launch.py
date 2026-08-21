import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    use_ukf = LaunchConfiguration('use_ukf')
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'

    ekf_config_path = LaunchConfiguration('ekf_config_file').perform(context)
    if ekf_config_path == 'default' or ekf_config_path == '':
        ekf_file = 'ekf_filter_sim.yaml' if use_sim_time else 'ekf_filter.yaml'
        ekf_config_path = os.path.join(get_package_share_directory('playground'), 'config', 'localization', ekf_file)

    ukf_config_path = LaunchConfiguration('ukf_config_file')

    return [
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            namespace='localization',
            output='screen',
            parameters=[ekf_config_path, {'use_sim_time': use_sim_time}],
            condition=UnlessCondition(use_ukf)
        ),
        
        Node(
            package='robot_localization',
            executable='ukf_node',
            name='ukf_filter_node',
            namespace='localization',
            output='screen',
            parameters=[ukf_config_path, {'use_sim_time': use_sim_time}],
            condition=IfCondition(use_ukf)
        ),
    ]

def generate_launch_description():
    pkg_share = FindPackageShare('playground')
    default_ukf = PathJoinSubstitution([pkg_share, 'config', 'localization', 'ukf_filter.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation clock if true'),
        DeclareLaunchArgument('use_ukf', default_value='false'),
        DeclareLaunchArgument('ekf_config_file', default_value='default', description='Path to EKF config file'),
        DeclareLaunchArgument('ukf_config_file', default_value=default_ukf, description='Path to UKF config file'),
        OpaqueFunction(function=launch_setup)
    ])
