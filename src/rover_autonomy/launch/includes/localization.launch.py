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

    localization_ns = LaunchConfiguration('localization_ns')
    camera_ns       = LaunchConfiguration('camera_ns')
    lidar_ns        = LaunchConfiguration('lidar_ns')

    # Override the generic odom0/odom1 aliases declared in ekf_filter.yaml
    # with the actual absolute dynamic topic paths resolved from launch arguments.
    ekf_remappings = [
        ('odom0', ['/', camera_ns, '/odom']),
        ('odom1', ['/', lidar_ns,  '/odom']),
    ]

    return [
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            namespace=localization_ns,
            output='screen',
            parameters=[ekf_config_path, {'use_sim_time': use_sim_time}],
            remappings=ekf_remappings,
            arguments=['--ros-args', '--log-level', 'warn'],
        ),
    ]

def generate_launch_description():
    pkg_share = FindPackageShare('rover_autonomy')
    default_params_file = PathJoinSubstitution([pkg_share, 'config', 'odometry', 'ekf_filter.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('params_file',       default_value=default_params_file, description=''),
        DeclareLaunchArgument('use_sim_time',      default_value='false',             description=''),
        DeclareLaunchArgument('localization_ns',   default_value='localization',      description='Localization node namespace'),
        DeclareLaunchArgument('camera_ns',         default_value='camera_00',         description='Camera sensor namespace'),
        DeclareLaunchArgument('lidar_ns',          default_value='lidar_00',          description='Lidar sensor namespace'),
        OpaqueFunction(function=launch_setup)
    ])
