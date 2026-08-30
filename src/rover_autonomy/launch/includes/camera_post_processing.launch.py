from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def launch_setup(context, *args, **kwargs):
    camera_ns = LaunchConfiguration('camera_ns').perform(context)
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)

    rgb_image_topic = f"/{camera_ns}/rgb/image_raw" if use_sim_time else f"/{camera_ns}/rgb/image_raw/uncompressed"

    sync_parameters = {
        'approx_sync': True,
        'approx_sync_max_interval': 1.0,
        'qos': 1,
        'qos_image': 2,
        'qos_camera_info': 2,
        'use_sim_time': use_sim_time,             
        'topic_queue_size': 100,          
        'sync_queue_size': 100, 
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
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_ns', default_value='camera_00', description=''),
        DeclareLaunchArgument('use_sim_time', default_value='false', description=''),
        OpaqueFunction(function=launch_setup)
    ])
