from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def launch_setup(context, *args, **kwargs):
    sensor_ns = LaunchConfiguration('sensor_ns').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)

    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'),
        value_type=bool
    )

    # Opt. 2: Pass sensor_ns directly as the Node namespace.
    # This automatically scopes 'odom_raw' and 'odom' to /{sensor_ns}/odom_raw
    # and /{sensor_ns}/odom without any explicit topic remappings.
    return [
        Node(
            package='rover_autonomy',
            executable='odometry_health_node',
            name='odometry_health_node',
            namespace=sensor_ns,
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', 'info'],
        ),
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sensor_ns',    default_value='camera_00', description='Namespace of the sensor being watched (e.g. camera_00, camera_01, lidar_00)'),
        DeclareLaunchArgument('params_file',  default_value='',          description='Absolute path to the health node YAML config'),
        DeclareLaunchArgument('use_sim_time', default_value='false',     description='Use simulation clock'),
        OpaqueFunction(function=launch_setup)
    ])
