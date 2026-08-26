import os
from datetime import datetime
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration('params_file')
    namespace = LaunchConfiguration('namespace')

    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'),
        value_type=bool
    )
    
    db_folder = LaunchConfiguration('mapping_db_folder').perform(context)
    db_folder_expanded = os.path.expanduser(db_folder)
    
    if not os.path.exists(db_folder_expanded):
        os.makedirs(db_folder_expanded)

    load_existing = LaunchConfiguration('mapping_load_existing_db').perform(context).lower() == 'true'
    
    if load_existing:
        db_file_name = LaunchConfiguration('mapping_db_file_name').perform(context)
        database_path = os.path.join(db_folder_expanded, db_file_name)
    else:
        now = datetime.now().strftime("%Y%m%d_%H%M%S")
        db_file_name = f"rtabmap_{now}.db"
        database_path = os.path.join(db_folder_expanded, db_file_name)

    # We enforce SLAM mode (incremental memory) just to be safe, 
    # though it is usually defined in mapping.yaml.
    rtabmap_parameters = [
        params_file, 
        {
            'use_sim_time': use_sim_time,
            'database_path': database_path,
            'Mem/IncrementalMemory': 'true', 
            'Mem/InitWMWithAllNodes': 'true' 
        }
    ]

    slam_arguments = []

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
            namespace=namespace,
            output='screen',
            parameters=rtabmap_parameters,
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
        DeclareLaunchArgument('namespace', default_value='mapping', description='Namespace for mapping'),

        # -- arguments
        DeclareLaunchArgument('mapping_db_folder', default_value='~/.ros/rtabmap'),
        DeclareLaunchArgument('mapping_load_existing_db', default_value='false'),
        DeclareLaunchArgument('mapping_db_file_name', default_value='rtabmap.db'),
        OpaqueFunction(function=launch_setup)
    ])
