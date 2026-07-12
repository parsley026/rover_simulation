from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description() -> LaunchDescription:

    pkg_project_gazebo_worlds = get_package_share_directory('gazebo_worlds')
    pkg_project_gazebo = get_package_share_directory('gazebo_simulation')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='true', 
        description='Use simulation clock'
    )

    world_name_arg = DeclareLaunchArgument(
        'world-name', 
        default_value='rex.sdf',
        description='Name of the SDF world file'
    )
    
    world_name = LaunchConfiguration('world-name')

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_project_gazebo, 'launch', 'components', '_gazebo_server.py'])
        ),
        launch_arguments={
            'world-file': PathJoinSubstitution([pkg_project_gazebo_worlds, 'worlds', world_name]),
        }.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        world_name_arg,
        gazebo_server
    ])