from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("rover_robot_arm_kinematics")

    arm_config_file_arg = DeclareLaunchArgument(
        "arm_config_file",
        default_value=PathJoinSubstitution(
            [pkg_share, "config", "arm_config.yaml"]
        ),
        description="Path to arm configuration YAML file",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock",
    )

    arm_controller_node = Node(
        package="rover_robot_arm_kinematics",
        executable="rover_robot_arm_kinematics_node",
        name="rover_robot_arm_kinematics_node",
        parameters=[
            LaunchConfiguration("arm_config_file"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        arm_config_file_arg,
        use_sim_time_arg,
        arm_controller_node,
    ])
