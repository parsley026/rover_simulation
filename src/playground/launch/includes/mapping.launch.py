from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    delete_db_on_start = LaunchConfiguration('delete_db_on_start')
    enable_rtabmap_viz = LaunchConfiguration('enable_rtabmap_viz')
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    params_file = LaunchConfiguration('params_file')
    
    slam_arguments = []
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

        # Node(
        #     package='rtabmap_viz',
        #     executable='rtabmap_viz',
        #     name='rtabmap_viz',
        #     condition=IfCondition(enable_rtabmap_viz),
        #     ...
        # )
        
    ]

def generate_launch_description():
    pkg_share = FindPackageShare('playground')
    default_params_file = PathJoinSubstitution([pkg_share, 'config', 'mapping', 'rtabmap_slam.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation clock if true'),
        DeclareLaunchArgument('delete_db_on_start', default_value='false'),
        DeclareLaunchArgument('enable_rtabmap_viz', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=default_params_file, description='Path to RTAB-Map parameters file'),
        OpaqueFunction(function=launch_setup)
    ])
