import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('rover_robot_arm_kinematics_bridge')
    config_file = os.path.join(pkg_share, 'config', 'bridge_config.yaml')

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='simulation',
        description="Bridge mode: 'simulation' | 'digital_shadow'"
    )

    bridge_node = Node(
        package='rover_robot_arm_kinematics_bridge',
        executable='rover_robot_arm_kinematics_bridge_node',
        name='rover_robot_arm_kinematics_bridge_node',
        output='screen',
        parameters=[
            config_file,
            {'mode': LaunchConfiguration('mode')}
        ]
    )

    return LaunchDescription([
        mode_arg,
        bridge_node
    ])
