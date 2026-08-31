from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    camera_ns = LaunchConfiguration('camera_ns').perform(context)
    params_file = LaunchConfiguration('params_file')
    health_enabled = LaunchConfiguration('health_enabled').perform(context).lower() == 'true'

    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'),
        value_type=bool
    )

    # Opt. 1: Relative remapping — the node runs inside its namespace already,
    # so we only need to rename the local 'odom' topic to 'odom_raw'.
    # When health is OFF, no remapping is applied and the topic stays as 'odom'.
    odom_remappings = [("odom", "odom_raw")] if health_enabled else []

    return [
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rtabmap_odom',
            namespace=camera_ns,

            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=odom_remappings,
            arguments=['--ros-args', '--log-level', 'warn'],
        ),
    ]

def generate_launch_description():
    pkg_share = FindPackageShare('rover_autonomy')
    default_params_file = PathJoinSubstitution([pkg_share, 'config', 'odometry', 'rgbd_odom.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('camera_ns',         default_value='camera_00',         description='Camera sensor namespace'),
        DeclareLaunchArgument('params_file',       default_value=default_params_file, description='Path to odometry YAML config'),
        DeclareLaunchArgument('use_sim_time',      default_value='false',             description='Use simulation clock'),
        DeclareLaunchArgument('health_enabled',    default_value='false',             description='If true, remap odom -> odom_raw for health node pipeline'),
        OpaqueFunction(function=launch_setup)
    ])
