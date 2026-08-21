from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    urdf_package = LaunchConfiguration('urdf_package').perform(context)
    urdf_file = LaunchConfiguration('urdf_file').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'

    mount_description = {
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
            ]),
            value_type=str
        )
    }

    return [
        Node(
            name='mount_state_publisher',
            namespace='',
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[mount_description, {'use_sim_time': use_sim_time}],
        ),
        Node(
            name='mount_joint_state_publisher',
            namespace='',
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[mount_description, {'use_sim_time': use_sim_time}]
        ),
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation clock if true'),
        DeclareLaunchArgument('urdf_package', default_value='playground'),
        DeclareLaunchArgument('urdf_file', default_value='sensor_mounts.urdf.xacro'),
        OpaqueFunction(function=launch_setup)
    ])
