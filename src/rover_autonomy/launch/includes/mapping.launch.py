from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration('params_file')

    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'),
        value_type=bool
    )
    
    slam_arguments = []
    delete_db_on_start = LaunchConfiguration('delete_db_on_start')
    if delete_db_on_start.perform(context).lower() == 'true':
        slam_arguments.append('--delete_db_on_start')

    slam_remappings = [
        ("rgbd_image", "/camera_00/rgbd_image"),
        ("scan_cloud", "/lidar_00/points"),
        ("odom", "/localization/odometry/filtered")
    ]

    return [
        Node(
            package='rtabmap_slam', 
            executable='rtabmap', 
            name='rtabmap_slam',
            namespace='mapping',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=slam_remappings,
            arguments=slam_arguments
        ),
        
    ]

def generate_launch_description():
    pkg_share = FindPackageShare('rover_autonomy')
    default_params_file = PathJoinSubstitution([pkg_share, 'config', 'mapping', 'mapping.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params_file, description=''),

        DeclareLaunchArgument('use_sim_time', default_value='false', description=''),

        # -- arguments
        DeclareLaunchArgument('delete_db_on_start', default_value='false'),
        OpaqueFunction(function=launch_setup)
    ])
