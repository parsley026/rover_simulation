from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import UnlessCondition, IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    camera_ns = LaunchConfiguration('camera_ns').perform(context)

    return [
        Node(
            package="image_transport",
            executable="republish",
            name="rgb_republish",
            namespace="",
            arguments=['compressed', 'raw'],
            remappings=[
                ("in/compressed", f"/{camera_ns}/rgb/image_raw/compressed"),
                ("out", f"/{camera_ns}/rgb/image_raw/uncompressed")
            ],
            condition=IfCondition(LaunchConfiguration('decompress_rgb'))
        ),
        Node(
            package="image_transport",
            executable="republish",
            name="stereo_republish",
            namespace="",
            arguments=['compressed', 'raw'],
            remappings=[
                ("in/compressed", f"/{camera_ns}/stereo/image_raw/compressed"),
                ("out", f"/{camera_ns}/stereo/image_raw/uncompressed")
            ],
            condition=IfCondition(LaunchConfiguration('decompress_depth'))
        ),
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_ns', default_value='camera_00', description=''),
        DeclareLaunchArgument('decompress_rgb', default_value='true', description=''),
        DeclareLaunchArgument('decompress_depth', default_value='true', description=''),
        DeclareLaunchArgument('use_sim_time', default_value='false', description=''),
        OpaqueFunction(function=launch_setup)
    ])
