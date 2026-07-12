from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description() -> LaunchDescription:
    
    """
    Compose the Gazebo simulation launch description for the selected world and robot.
    
    The launch description declares arguments for simulation time, world filename,
    robot model filename, and spawn pose, then includes the Gazebo server, bridge,
    and robot-spawning launch components.
    
    Returns:
        LaunchDescription: The configured launch description.
    """
    pkg_project_gazebo_worlds = get_package_share_directory('gazebo_worlds')
    pkg_project_gazebo = get_package_share_directory('gazebo_simulation')
    pkg_project_description = get_package_share_directory('rover_description')

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

    use_sim_time = LaunchConfiguration('use_sim_time')
    world_name = LaunchConfiguration('world-name')
    robot_name = LaunchConfiguration('robot-name')
    pose = LaunchConfiguration('pose')

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
            'model-file': PathJoinSubstitution([pkg_project_description, 'urdf', robot_name]), # Fixed argument name
            'use_sim_time': use_sim_time,
            'pose': pose, # Passing the new pose down
        }.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        world_name_arg,
        robot_name_arg,
        pose_arg,
        gazebo_server,
        bridge_core,
        spawn_robot,
    ])