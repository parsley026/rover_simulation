from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    urdf_package = LaunchConfiguration('urdf_package').perform(context)
    urdf_file    = LaunchConfiguration('urdf_file').perform(context)

    camera_ns   = LaunchConfiguration('camera_ns').perform(context)
    camera_name = LaunchConfiguration('camera_name').perform(context)
    
    camera_parent_frame = LaunchConfiguration('camera_parent_frame').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)

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

    return [
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
            condition=UnlessCondition(LaunchConfiguration('use_sim_time'))
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
