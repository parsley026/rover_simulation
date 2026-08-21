from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    camera_ns = LaunchConfiguration('camera_ns').perform(context)
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'

    rgb_image_topic = f"/{camera_ns}/rgb/image_raw" if use_sim_time else f"/{camera_ns}/rgb/image_raw/uncompressed"

    sync_parameters = {
        'approx_sync': True,
        'approx_sync_max_interval': 0.1,
        'qos': 1,
        'qos_image': 2,
        'qos_camera_info': 2,
        'use_sim_time': use_sim_time,             
        'topic_queue_size': 50,          
        'sync_queue_size': 50, 
    }

    sync_remappings = [
        ("rgb/camera_info", f"/{camera_ns}/rgb/camera_info"),
        ("rgb/image",       rgb_image_topic),
        ("depth/image",     f"/{camera_ns}/stereo/image_raw"),
    ]

    return [
        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='rtabmap_sync',
            namespace=camera_ns,
            parameters=[sync_parameters],
            remappings=sync_remappings,
        ),
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
    pkg_share = FindPackageShare('playground')
    default_params_file = PathJoinSubstitution([pkg_share, 'config', 'odometry', 'rgbd_odom.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation clock if true'),
        DeclareLaunchArgument('camera_ns', default_value='camera_00', description='Namespace of the RGBD camera'),
        DeclareLaunchArgument('params_file', default_value=default_params_file, description='Path to odometry parameter file'),
        OpaqueFunction(function=launch_setup)
    ])
