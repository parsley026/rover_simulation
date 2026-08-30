from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration('params_file')
    
    return [
        Node(
            package='rover_autonomy',
            executable='generic_temporal_sync_node',
            name='generic_temporal_sync',
            output='screen',
            parameters=[params_file]
        )
    ]

def generate_launch_description():
    pkg_share = FindPackageShare('rover_autonomy')
    default_params_file = PathJoinSubstitution([pkg_share, 'config', 'sync', 'generic_sync.yaml'])
    
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params_file),
        OpaqueFunction(function=launch_setup)
    ])
