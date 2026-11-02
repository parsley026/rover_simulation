from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_context import LaunchContext
from typing import Any


def launch_setup(context: LaunchContext, *_, **__) -> list[Any]:
    pkg_gazebo_worlds = get_package_share_directory('gazebo_worlds')
    pkg_gazebo        = get_package_share_directory('gazebo_simulation')

    submodule                 = LaunchConfiguration('submodule')
    use_sim_time              = LaunchConfiguration('use_sim_time')
    headless                  = LaunchConfiguration('headless')
    world_name                = LaunchConfiguration('world-name')
    mount_height              = LaunchConfiguration('mount_height')
    include_sensors           = LaunchConfiguration('include_sensors')
    run_arm_kinematics        = LaunchConfiguration('run_arm_kinematics')
    run_arm_kinematics_bridge = LaunchConfiguration('run_arm_kinematics_bridge')

    # ------------------------------------------------------------------ Gazebo world
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo, 'launch', 'components', '_gazebo_server.py'])
        ),
        launch_arguments={
            'world-file': PathJoinSubstitution([pkg_gazebo_worlds, 'worlds', world_name]),
            'headless':   headless,
        }.items()
    )

    # ------------------------------------------------------------------ Clock + TF bridge
    bridge_core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo, 'launch', 'components', '_bridge_core.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    # ------------------------------------------------------------------ Submodule spawn + bringup
    spawn_submodule = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo, 'launch', 'components', '_spawn_submodule.py'])
        ),
        launch_arguments={
            'submodule':                 submodule,
            'use_sim_time':              use_sim_time,
            'mount_height':              mount_height,
            'include_sensors':           include_sensors,
            'run_arm_kinematics':        run_arm_kinematics,
            'run_arm_kinematics_bridge': run_arm_kinematics_bridge,
        }.items()
    )

    return [gazebo_server, bridge_core, spawn_submodule]


def generate_launch_description() -> LaunchDescription:
    """
    """
    return LaunchDescription([
        DeclareLaunchArgument('submodule',                 default_value='robotic_arm', description='Submodule name: robotic_arm | autonomy | crossbar | sampler'),
        DeclareLaunchArgument('use_sim_time',              default_value='true',        description='Use simulation clock'),
        DeclareLaunchArgument('headless',                  default_value='false',       description='Run Gazebo without GUI'),
        DeclareLaunchArgument('world-name',                default_value='rover.sdf',   description='SDF world filename in gazebo_worlds/worlds/'),
        DeclareLaunchArgument('mount_height',              default_value='0.8',         description='Z height of mount above world origin (m)'),
        DeclareLaunchArgument('include_sensors',           default_value='true',        description='Attach sensors to submodule mounts if available'),
        DeclareLaunchArgument('run_arm_kinematics',        default_value='false',       description='Launch rover_robot_arm_kinematics (robotic_arm only)'),
        DeclareLaunchArgument('run_arm_kinematics_bridge', default_value='false',       description='Launch rover_robot_arm_kinematics_bridge (robotic_arm only)'),
        OpaqueFunction(function=launch_setup),
    ])
