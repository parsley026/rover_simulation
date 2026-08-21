from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

def generate_launch_description() -> LaunchDescription:
    
    # --- Package Directories ---
    """
    Builds a ROS 2 launch description for a Gazebo rover simulation.
    
    The launch description starts Gazebo, bridges simulation data, spawns the
    selected rover model at the configured pose, and optionally launches the rover
    kinematics components.
    
    Returns:
        LaunchDescription: The configured simulation launch actions.
    """
    pkg_project_gazebo_worlds = get_package_share_directory('gazebo_worlds')
    pkg_project_gazebo = get_package_share_directory('gazebo_simulation')
    pkg_project_description = get_package_share_directory('rover_description')
    
    # New packages
    pkg_rover_kinematics = get_package_share_directory('rover_kinematics')
    pkg_rover_kinematics_bridge = get_package_share_directory('rover_kinematics_bridge')

    # --- Declare Launch Arguments ---
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='true', 
        description='Use simulation clock'
    )
    
    world_name_arg = DeclareLaunchArgument(
        'world-name', 
        default_value='rex.sdf',
        description='Name of the SDF world file in gazebo_worlds'
    )

    robot_name_arg = DeclareLaunchArgument(
        'robot-name',
        default_value='rex.urdf.xacro',
        description='Name of the URDF/XACRO robot file in rover_description'
    )

    pose_arg = DeclareLaunchArgument(
        'pose',
        default_value='0.0 0.0 0.5',
        description='Spawn coordinates of the robot as "x y z"'
    )

    # New argument to toggle kinematics
    run_kinematics_arg = DeclareLaunchArgument(
        'run_kinematics',
        default_value='true',
        description='Option to launch the rover kinematics and kinematics bridge'
    )

    # --- Launch Configurations ---
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_name = LaunchConfiguration('world-name')
    robot_name = LaunchConfiguration('robot-name')
    pose = LaunchConfiguration('pose')
    run_kinematics = LaunchConfiguration('run_kinematics')

    # --- Include Launch Descriptions ---
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_project_gazebo, 'launch', 'components', '_gazebo_server.py'])
        ),
        launch_arguments={
            'world-file': PathJoinSubstitution([pkg_project_gazebo_worlds, 'worlds', world_name]),
        }.items()
    )

    bridge_core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_project_gazebo, 'launch', 'components', '_bridge_core.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    # bridge_sensors = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([pkg_project_gazebo, 'launch', 'components', '_bridge_sensors.py'])
    #     ),
    #     launch_arguments={
    #         'use_sim_time': use_sim_time,
    #     }.items()
    # )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_project_gazebo, 'launch', 'components', '_spawn_robot.py'])
        ),
        launch_arguments={
            'model-file': PathJoinSubstitution([pkg_project_description, 'urdf', robot_name]), 
            'use_sim_time': use_sim_time,
            'pose': pose, 
        }.items()
    )

    rover_kinematics_bridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([pkg_rover_kinematics_bridge, 'launch', 'rover_kinematics_bridge.launch.xml'])
        ),
        condition=IfCondition(run_kinematics)
    )

    return LaunchDescription([
        use_sim_time_arg,
        world_name_arg,
        robot_name_arg,
        pose_arg,
        run_kinematics_arg,
        gazebo_server,
        bridge_core,
        spawn_robot,
        rover_kinematics_bridge
    ])