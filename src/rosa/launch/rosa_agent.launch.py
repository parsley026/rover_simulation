import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate a LaunchDescription that starts the ROSA agent node.

    Launch arguments
    ----------------
    params_file : str
        Path to a YAML file of ros__parameters for the rosa_agent node.
        Defaults to config/rosa_agent.yaml in the installed package share.
    """
    pkg_share = get_package_share_directory('rosa')
    default_config = os.path.join(pkg_share, 'config', 'rosa_agent.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_config,
        description='Full path to the YAML parameter file for the rosa_agent node.',
    )

    rosa_agent_node = Node(
        package='rosa',
        executable='rosa_agent',
        name='rosa_agent',
        parameters=[LaunchConfiguration('params_file')],
        output='screen',
        emulate_tty=True,   # preserves colour / ANSI codes in LLM output
    )

    return LaunchDescription([
        params_file_arg,
        rosa_agent_node,
    ])
