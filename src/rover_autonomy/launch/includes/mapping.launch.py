import os
from datetime import datetime
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
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

    mapping_mode = str(LaunchConfiguration('mapping_mode').perform(context))
    camera_00_ns = LaunchConfiguration('camera_00_ns').perform(context)
    camera_01_ns = LaunchConfiguration('camera_01_ns').perform(context)
    lidar_ns = LaunchConfiguration('lidar_ns').perform(context)
    localization_ns = LaunchConfiguration('localization_ns').perform(context)

    mode_params = {}
    slam_remappings = [
        ("odom", f"/{localization_ns}/odometry/filtered"),
    ]

    sync_nodes = []

    if mapping_mode == '111':
        # 111: Full Fusion Mode (LiDAR + Cam0 + Cam1)
        mode_params = {
            'subscribe_rgbd': True, 'rgbd_cameras': 0,
            'subscribe_scan_cloud': True, 'subscribe_depth': False,
            'Reg/Strategy': '2' # Visual + ICP
        }
        slam_remappings.extend([
            ("rgbd_images", "rgbd_images"),
            ("scan_cloud", f"/{lidar_ns}/points")
        ])
        sync_nodes.append(
            Node(
                package='rtabmap_sync',
                executable='rgbdx_sync',
                name='rgbdx_sync',
                namespace=namespace,
                output='screen',
                parameters=[{'rgbd_cameras': 2, 'approx_sync': True, 'use_sim_time': use_sim_time}],
                remappings=[
                    ('rgbd_image0', f"/{camera_00_ns}/rgbd_image"),
                    ('rgbd_image1', f"/{camera_01_ns}/rgbd_image"),
                    ('rgbd_images', 'rgbd_images')
                ]
            )
        )
    elif mapping_mode == '110':
        # 110: LiDAR + Cam0 Mode
        mode_params = {
            'subscribe_rgbd': True, 'rgbd_cameras': 1,
            'subscribe_scan_cloud': True, 'subscribe_depth': False,
            'Reg/Strategy': '2' # Visual + ICP
        }
        slam_remappings.extend([
            ("rgbd_image", f"/{camera_00_ns}/rgbd_image"),
            ("scan_cloud", f"/{lidar_ns}/points")
        ])
    elif mapping_mode == '101':
        # 101: LiDAR + Cam1 Mode
        mode_params = {
            'subscribe_rgbd': True, 'rgbd_cameras': 1,
            'subscribe_scan_cloud': True, 'subscribe_depth': False,
            'Reg/Strategy': '2' # Visual + ICP
        }
        slam_remappings.extend([
            ("rgbd_image", f"/{camera_01_ns}/rgbd_image"),
            ("scan_cloud", f"/{lidar_ns}/points")
        ])
    elif mapping_mode == '100':
        # 100: LiDAR-Only Mode
        mode_params = {
            'subscribe_rgbd': False, 'subscribe_depth': False, 'subscribe_stereo': False,
            'subscribe_scan_cloud': True,
            'Reg/Strategy': '1', 'Icp/PointToPoint': 'true',
            'Grid/Sensor': '0'
        }
        slam_remappings.extend([
            ("scan_cloud", f"/{lidar_ns}/points")
        ])
    elif mapping_mode in ['011', '11', '9']:
        # 011: Cam0 + Cam1 Mode (9 is included in case yaml evaluates 011 as octal)
        mode_params = {
            'subscribe_rgbd': True, 'rgbd_cameras': 0,
            'subscribe_scan_cloud': False, 'subscribe_depth': False,
            'Grid/FromDepth': 'true',
            'Reg/Strategy': '0', # Visual
            'Grid/Sensor': '1'
        }
        slam_remappings.extend([
            ("rgbd_images", "rgbd_images")
        ])
        sync_nodes.append(
            Node(
                package='rtabmap_sync',
                executable='rgbdx_sync',
                name='rgbdx_sync',
                namespace=namespace,
                output='screen',
                parameters=[{'rgbd_cameras': 2, 'approx_sync': True, 'use_sim_time': use_sim_time}],
                remappings=[
                    ('rgbd_image0', f"/{camera_00_ns}/rgbd_image"),
                    ('rgbd_image1', f"/{camera_01_ns}/rgbd_image"),
                    ('rgbd_images', 'rgbd_images')
                ]
            )
        )
    elif mapping_mode in ['010', '10', '8']:
        # 010: Cam0-Only Mode
        mode_params = {
            'subscribe_rgbd': True, 'rgbd_cameras': 1,
            'subscribe_scan_cloud': False, 'subscribe_depth': False,
            'Grid/FromDepth': 'true',
            'Reg/Strategy': '0', # Visual
            'Grid/Sensor': '1'
        }
        slam_remappings.extend([
            ("rgbd_image", f"/{camera_00_ns}/rgbd_image")
        ])
    elif mapping_mode in ['001', '1']:
        # 001: Cam1-Only Mode
        mode_params = {
            'subscribe_rgbd': True, 'rgbd_cameras': 1,
            'subscribe_scan_cloud': False, 'subscribe_depth': False,
            'Grid/FromDepth': 'true',
            'Reg/Strategy': '0', # Visual
            'Grid/Sensor': '1'
        }
        slam_remappings.extend([
            ("rgbd_image", f"/{camera_01_ns}/rgbd_image")
        ])

    rtabmap_parameters.append(mode_params)

    slam_arguments = []

    return sync_nodes + [
        Node(
            package='rtabmap_slam', 
            executable='rtabmap', 
            name='rtabmap_slam',
            namespace=namespace,
            output='screen',
            parameters=rtabmap_parameters,
            remappings=slam_remappings,
            arguments=slam_arguments,
            additional_env={
        'OMP_NUM_THREADS': '8',
        'OPENBLAS_NUM_THREADS': '8',
        'MKL_NUM_THREADS': '8',
        'OpenCV_NUM_THREADS': '8'
    }
        ),
    ]

def generate_launch_description():
    pkg_share = FindPackageShare('rover_autonomy')
    default_params_file = PathJoinSubstitution([pkg_share, 'config', 'mapping', 'mapping.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params_file, description=''),

        DeclareLaunchArgument('use_sim_time',      default_value='false',        description=''),
        DeclareLaunchArgument('namespace',          default_value='mapping',      description='Namespace for mapping'),
        DeclareLaunchArgument('mapping_mode',       default_value='110',          description='SLAM Mode (Binary: LiDAR-Cam0-Cam1, e.g. 110)'),
        DeclareLaunchArgument('camera_00_ns',  default_value='camera_00',    description='Primary camera sensor namespace'),
        DeclareLaunchArgument('camera_01_ns',default_value='camera_01',    description='Secondary camera sensor namespace'),
        DeclareLaunchArgument('lidar_ns',           default_value='lidar_00',     description='Lidar sensor namespace'),
        DeclareLaunchArgument('localization_ns',    default_value='localization', description='Localization namespace'),

        # -- arguments
        DeclareLaunchArgument('mapping_db_folder', default_value='~/.ros/rtabmap'),
        DeclareLaunchArgument('mapping_load_existing_db', default_value='false'),
        DeclareLaunchArgument('mapping_db_file_name', default_value='rtabmap.db'),
        OpaqueFunction(function=launch_setup)
    ])
