from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

def generate_launch_description() -> LaunchDescription:
    pkg_project_gazebo_worlds = get_package_share_directory('gazebo_worlds')
    
    pkg_project_gazebo = get_package_share_directory('gazebo_simulation')

    world_name = LaunchConfiguration('world-name')
    world_name_launch_arg = DeclareLaunchArgument(
        'world-name', 
        default_value='rex.sdf',
        description='Name of the SDF world file'
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_project_gazebo, 'launch', '_gazebo_server.py'])
        ),
        launch_arguments={
            'world-file': PathJoinSubstitution([pkg_project_gazebo_worlds, 'worlds', world_name]),
        }.items()
    )

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_project_gazebo, 'launch', '_ros_gz_bridge.py'])
        ),
    )

    return LaunchDescription([
        world_name_launch_arg,
        gz_sim,
        bridge
    ])