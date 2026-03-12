from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

def generate_launch_description() -> LaunchDescription:
    pkg_project_gazebo = get_package_share_directory('gazebo_simulation')

    config_file = LaunchConfiguration('config-file')

    config_file_launch_arg = DeclareLaunchArgument(
        'config-file',
        default_value=PathJoinSubstitution([pkg_project_gazebo, 'config', 'rex_ros_gz_bridge.yaml']),
        description='Path to YAML config file for ros_gz_bridge'
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': config_file,
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    return LaunchDescription([
        config_file_launch_arg,
        gz_bridge
    ])



