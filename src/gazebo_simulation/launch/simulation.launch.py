from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import LifecycleNode


def generate_launch_description() -> LaunchDescription:
    """
    """
    pkg_gazebo_worlds = get_package_share_directory("gazebo_worlds")
    pkg_gazebo        = get_package_share_directory("gazebo_simulation")
    pkg_description   = get_package_share_directory("rover_description")
    pkg_rover_kinematics_bridge = get_package_share_directory('rover_kinematics_bridge')

    # ------------------------------------------------------------------ args
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use simulation clock")
        
    headless_arg = DeclareLaunchArgument(
        "headless", default_value="false",
        description="Run Gazebo headless")

    world_name_arg = DeclareLaunchArgument(
        "world-name", default_value="rover.sdf",
        description="Name of the SDF world file in gazebo_worlds")

    pose_arg = DeclareLaunchArgument(
        "pose", default_value="0.0 0.0 0.5",
        description='Spawn coordinates as "x y z"')

    entity_name_arg = DeclareLaunchArgument(
        "robot_name", default_value="rover",
        description="Gazebo entity name; drives sensor topic path tokens")

    gz_world_name_arg = DeclareLaunchArgument(
        "world_name", default_value="mars_yard",
        description="Gazebo world name; drives sensor topic path tokens")

    mode_arg = DeclareLaunchArgument(
        "mode", default_value="simulation",
        description="Bridge mode: 'simulation' | 'digital_shadow'")
        
    run_kinematics_arg = DeclareLaunchArgument(
        "run_kinematics", default_value="false",
        description="Launch chassis kinematics node (rover_kinematics)")

    run_arm_kinematics_arg = DeclareLaunchArgument(
        "run_arm_kinematics", default_value="false",
        description="Launch robot arm kinematics controller (rover_robot_arm_kinematics)")

    run_kinematics_bridge_arg = DeclareLaunchArgument(
        "run_kinematics_bridge", default_value="true",
        description="Launch rover kinematics bridge")

    run_arm_kinematics_bridge_arg = DeclareLaunchArgument(
        "run_arm_kinematics_bridge", default_value="true",
        description="Launch rover robot arm kinematics bridge")

    spawn_arm_controllers_arg = DeclareLaunchArgument(
        "spawn_arm_controllers", default_value="true",
        description="Spawn arm and gripper controllers in Gazebo")

    run_bridge_arg = DeclareLaunchArgument(
        "run_bridge", default_value="true",
        description="Launch ROS-Gz bridges")

    local_bringup_arg = DeclareLaunchArgument(
        "local_bringup", default_value="true",
        description="Launch local rover_bringup stack on this machine")

    spawn_robot_arg = DeclareLaunchArgument(
        "spawn_robot", default_value="true",
        description="Spawn the robot in Gazebo")

    payload_config_arg = DeclareLaunchArgument(
        "payload_config",
        default_value=PathJoinSubstitution([pkg_description, "config", "payload.yaml"]),
        description="Path to payload.yaml configuration file")

    # ------------------------------------------------------------------ configs
    use_sim_time             = LaunchConfiguration("use_sim_time")
    payload_config           = LaunchConfiguration("payload_config")
    headless                 = LaunchConfiguration("headless")
    world_name               = LaunchConfiguration("world-name")
    pose                     = LaunchConfiguration("pose")
    entity_name              = LaunchConfiguration("robot_name")
    gz_world                 = LaunchConfiguration("world_name")
    mode                     = LaunchConfiguration("mode")
    run_kinematics           = LaunchConfiguration("run_kinematics")
    run_arm_kinematics       = LaunchConfiguration("run_arm_kinematics")
    run_kinematics_bridge    = LaunchConfiguration("run_kinematics_bridge")
    run_arm_kinematics_bridge = LaunchConfiguration("run_arm_kinematics_bridge")
    local_bringup            = LaunchConfiguration("local_bringup")
    spawn_arm_controllers    = LaunchConfiguration("spawn_arm_controllers")
    run_bridge               = LaunchConfiguration("run_bridge")
    spawn_robot              = LaunchConfiguration("spawn_robot")

    # ------------------------------------------------------------------ includes

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo, "launch", "components", "_gazebo_server.py"])),
        launch_arguments={
            "world-file": PathJoinSubstitution([pkg_gazebo_worlds, "worlds", world_name]),
            "headless": headless,
        }.items())

    bridge_core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo, "launch", "components", "_bridge_core.py"])),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items())

    spawn_robot_inc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo, "launch", "components", "_spawn_robot.py"])),
        launch_arguments={
            "payload_config":            payload_config,
            "use_sim_time":              use_sim_time,
            "pose":                      pose,
            "entity_name":               entity_name,
            "local_bringup":             local_bringup,
            "mode":                      mode,
            "run_kinematics":            run_kinematics,
            "run_arm_kinematics":        run_arm_kinematics,
            "run_kinematics_bridge":     run_kinematics_bridge,
            "run_arm_kinematics_bridge": run_arm_kinematics_bridge,
        }.items(),
        condition=IfCondition(spawn_robot))

    return LaunchDescription([
        use_sim_time_arg,
        headless_arg,
        world_name_arg,
        pose_arg,
        entity_name_arg,
        gz_world_name_arg,
        mode_arg,
        run_kinematics_arg,
        run_arm_kinematics_arg,
        run_kinematics_bridge_arg,
        run_arm_kinematics_bridge_arg,
        local_bringup_arg,
        spawn_arm_controllers_arg,
        run_bridge_arg,
        spawn_robot_arg,
        payload_config_arg,
        gazebo_server,
        bridge_core,
        spawn_robot_inc,
    ])