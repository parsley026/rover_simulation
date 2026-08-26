from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    camera_ns = LaunchConfiguration('camera_ns').perform(context)
    params_file = LaunchConfiguration('params_file')
    
    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'),
        value_type=bool
    )

    return [
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rtabmap_odom',
            namespace=camera_ns,
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=[],
            arguments=['--ros-args', '--log-level', 'fatal'],
        ),
    ]

def generate_launch_description():
    pkg_share = FindPackageShare('rover_autonomy')
    default_params_file = PathJoinSubstitution([pkg_share, 'config', 'odometry', 'rgbd_odom.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('camera_ns',   default_value='camera_00',         description=''),
        DeclareLaunchArgument('params_file', default_value=default_params_file, description=''),
        
        DeclareLaunchArgument('use_sim_time', default_value='false', description=''),
        OpaqueFunction(function=launch_setup)
    ])
