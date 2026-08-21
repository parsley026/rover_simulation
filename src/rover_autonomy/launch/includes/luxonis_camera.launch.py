from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    urdf_package = LaunchConfiguration('urdf_package').perform(context)
    urdf_file    = LaunchConfiguration('urdf_file')   .perform(context)

    camera_ns   = LaunchConfiguration('camera_ns').perform(context)
    camera_name = LaunchConfiguration('camera_name').perform(context)
    
    camera_parent_frame = LaunchConfiguration('camera_parent_frame').perform(context)

    params_file = LaunchConfiguration('params_file').perform(context)
    
    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'),
        value_type=bool
    )

    camera_description = {
        'robot_description': ParameterValue(
            Command([
                FindExecutable(name='xacro'),
                ' "',
                PathJoinSubstitution([
                    FindPackageShare(urdf_package),
                    'urdf',
                    urdf_file
                ]),
                '"',
                f" camera_name:={camera_name}",
                f" parent_frame:={camera_parent_frame}",
            ]),
            value_type=str
        )
    }

    camera_nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name=f"{camera_name}_state_publisher",
            namespace=camera_ns,
            parameters=[camera_description],
            remappings=[
                ('tf', '/tf'),
                ('tf_static', '/tf_static')
            ],
        ),
        Node(
            package="depthai_ros_driver",
            executable="camera_node",
            name=camera_name,
            namespace="",
            parameters=[params_file],
            condition=UnlessCondition(LaunchConfiguration('use_sim_time'))
        ),
    ]

    republish_nodes = [
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
            condition=UnlessCondition(LaunchConfiguration('use_sim_time'))
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
            condition=UnlessCondition(LaunchConfiguration('use_sim_time'))
        ),
    ]

    return camera_nodes + republish_nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('urdf_package', default_value='rover_autonomy',     description=''),
        DeclareLaunchArgument('urdf_file',    default_value='camera.urdf.xacro', description=''),
        
        DeclareLaunchArgument('camera_ns',   default_value='camera_00', description=''),
        DeclareLaunchArgument('camera_name', default_value='camera_00', description=''),

        DeclareLaunchArgument('camera_parent_frame', default_value='base_link', description=''),

        DeclareLaunchArgument('params_file',  default_value='',      description=''),
        DeclareLaunchArgument('use_sim_time', default_value='false', description=''),

        OpaqueFunction(function=launch_setup)
    ])
