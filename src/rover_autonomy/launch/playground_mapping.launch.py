from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


camera_00_namespace = 'camera_00'
camera_00_name      = 'camera_00'

lidar_00_namespace  = 'lidar_00'
lidar_00_name       = 'lidar_00'

use_sim_time = True

#

camera_00_sync_parameters = {
    'qos': 1,
    'qos_image': 2,
    'qos_camera_info': 2,
    'use_sim_time': use_sim_time,

    'approx_sync': use_sim_time,
    'approx_sync_max_interval': 0.1,

    'topic_queue_size': 50,          
    'sync_queue_size': 50, 
}

camera_00_sync_remappings = [
    ("rgb/camera_info", f"/{camera_00_namespace}/rgb/camera_info"),
    ("rgb/image",       f"/{camera_00_namespace}/rgb/image_raw"),

    ("depth/image",     f"/{camera_00_namespace}/stereo/image_raw"),
]

# 

camera_00_odom_parameters = {
    'qos': 1,
    'qos_image': 2,        
    'qos_camera_info': 2,
    'use_sim_time': use_sim_time,
    
    #--
    'subscribe_rgb'  : False,
    'subscribe_rgbd' : True, 

    'subscribe_depth': False,   

    'subscribe_scan_cloud': False,    

    #--
    'frame_id': 'base_link',
    'odom_frame_id': 'odom',
    'publish_tf': False,

    #--
    'approx_sync': True,
    'approx_sync_max_interval': 0.01,

    'publish_null_when_lost': False,

    'wait_imu_to_init': False,
}

camera_00_odom_remappings = [
]

# 

lidar_00_params_file = ''

lidar_00_odom_parameters = {
    'qos': 1,
    'qos_image': 2,        
    'qos_camera_info': 2,
    'use_sim_time': use_sim_time,

    #--
    'subscribe_rgb'  : False,
    'subscribe_rgbd' : False, 

    'subscribe_depth': False,   

    'subscribe_scan_cloud': True,    

    #--
    'frame_id': 'base_link',
    'odom_frame_id': 'odom',
    'publish_tf': False,

    #--
    'approx_sync': True,
    'approx_sync_max_interval': 0.01,

    'publish_null_when_lost': False,
    
    'wait_imu_to_init': False,
}

lidar_00_odom_remappings = [
    ("scan_cloud", "/lidar_00/points"),
    ("odom", "/lidar_00/odom"),
]

#

robot_localization_params_file_efk = '/home/raptors/Desktop/rover_simulation/src/rover_autonomy/config/efk_filter.yaml'

#

slam_parameters = {
    'qos': 1,
    'qos_image': 2,        
    'qos_camera_info': 2,
    'use_sim_time': use_sim_time,

    #--
    'subscribe_rgb'  : False,
    'subscribe_rgbd' : True, 

    'subscribe_depth': False,   

    'subscribe_scan_cloud': True,    
    
    #--
    'frame_id': 'base_link',
    'map_frame_id': 'map',
    'odom_frame_id': 'odom',         
    'publish_tf': True,       

    'map_always_update': True,

    'Rtabmap/DetectionRate': '10',
    'Rtabmap/TimeThr': '0',

    # 'Mem/IncrementalMemory': 'true', 
    # 'Mem/InitWMWithAllNodes': 'false', 
    # 'RGBD/OptimizeFromGraphEnd': 'false', 

    # 'RGBD/AngularUpdate': '0.05',    
    # 'RGBD/LinearUpdate': '0.05',     

    # 'RGBD/ProximityBySpace': 'true',     
    # 'Reg/Strategy': '2',                 
    # 'Reg/Force3DoF': 'false',            
    # 'RGBD/NeighborLinkRefining': 'true', 

    # 'Icp/PointToPlane': 'true',          
    # 'Icp/VoxelSize': '0.2',              
    # 'Icp/MaxCorrespondenceDistance': '1.0', 

    # 'RGBD/CreateOccupancyGrid': 'true',  
    # 'Grid/FromDepth': 'false',           
    # 'Grid/Sensor': '2',                  
    # 'Grid/3D': 'false',                  
    # 'Grid/RangeMax': '20.0',             
    # 'Grid/CellSize': '0.05',             
    # 'Grid/RayTracing': 'true',           

    #rtabmap
        'Rtabmap/DetectionRate': '10',
        'Rtabmap/TimeThr': '0',

        #mem
        'Mem/STMSize': '20',
        'Mem/IncrementalMemory': 'True',
        'Mem/InitWMWithAllNodes': 'False',

        #kp
        'Kp/MaxFeatures': '750',

        #rgbd
        'RGBD/NeighborLinkRefining': 'True',

        #reg
        'Reg/Strategy': '2',
        'Reg/Force3DoF': 'false',

        #vis
        
        #icp

        #stereo
        'Sterero/WinWidth': '15',
        'Sterero/WinHeight': '3',

        'Sterero/Iterations': '30',

        'Sterero/DenseStrategy': '0',

        #grid
        'Grid/Sensor': '2',

        'Grid/3D': 'True',

        'Grid/DepthDecimation': '4',

        'Grid/RangeMin': '0.5',
        'Grid/RangeMax': '5',

        'Grid/FootprintLength': '1.5',
        'Grid/FootprintWidth': '1.5',
        'Grid/FootprintHeight': '1.5',
        
        'Grid/MaxObstacleHeight': '0.2',

        'Grid/MinGroundHeight': '0.0',
        'Grid/MaxGroundHeight': '0.1',
        
        'Grid/MaxGroundAngle': '35',

        'Grid/FlatObstacleDetected': 'True',

        'Grid/FilteringRadious': '0.10',
        'Grid/NoiseFilteringMinNeighbor': '4',
    
        #global grid

        'GridGlobal/FootprintRadius': '0.75',

}

slam_remappings = [
    ("rgbd_image", f"/{camera_00_namespace}/rgbd_image"),
    ("scan_cloud", f"/{lidar_00_namespace}/points"),
    ("odom", "perfect/odometry")
]

def launch_setup(context, *args, **kwargs):
    enable_camera_00 = LaunchConfiguration('enable_camera_00')
    enable_lidar_00  = LaunchConfiguration('enable_lidar_00')

    camera_00_nodes = [
        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='rtabmap_sync',
            namespace=camera_00_namespace,
            parameters=[camera_00_sync_parameters],
            remappings=camera_00_sync_remappings,
            condition=IfCondition(enable_camera_00)
        ),

        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rtabmap_odom',
            namespace=camera_00_namespace,
            parameters=[camera_00_odom_parameters],
            remappings=camera_00_odom_remappings,
            arguments=['--ros-args', '--log-level', 'fatal'],
            condition=IfCondition(enable_camera_00)
        ),
    ]

    lidar_00_nodes = [
        Node(
            package='rtabmap_odom', 
            executable='icp_odometry',
            name='rtabmap_odom',
            namespace=lidar_00_namespace,
            parameters=[lidar_00_odom_parameters],
            remappings=lidar_00_odom_remappings,
            arguments=['--ros-args', '--log-level', 'fatal'],
            condition=IfCondition(enable_lidar_00)
        ),
    ]

    nodes = [
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            namespace='localization',
            output='screen',
            parameters=[robot_localization_params_file_efk]
        ),
    ]

    mapping = [
        Node(
            package='rtabmap_slam', 
            executable='rtabmap', 
            name='rtabmap_slam',
            namespace='mapping',
            output='screen',
            parameters=[slam_parameters],
            remappings=slam_remappings,
            arguments=['--delete_db_on_start']
        ),
    ]

    return camera_00_nodes + lidar_00_nodes + nodes + mapping 


def generate_launch_description():
    """
    Create the launch description for the mapping system.
    
    Returns:
        LaunchDescription: A launch description containing camera and LiDAR enablement arguments and the launch-time setup function.
    """
    declared_arguments = [
        DeclareLaunchArgument('enable_camera_00', default_value='true'),
        DeclareLaunchArgument('enable_lidar_00',  default_value='true'),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
    