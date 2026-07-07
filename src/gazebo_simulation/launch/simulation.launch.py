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

    pkg_project_description = get_package_share_directory('rover_description')

    world_name = LaunchConfiguration('world-name')
    world_name_launch_arg = DeclareLaunchArgument(
        'world-name', 
        default_value='rex.sdf',
        description='Name of the SDF world name'
    )

    model_name = LaunchConfiguration('model-name')
    model_name_launch_arg = DeclareLaunchArgument(
        'model-name',
        default_value='rex.urdf.xacro',
        description='Name of the URDF model file'
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

    spawn_model = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_project_gazebo, 'launch', '_spawn_model.py'])
        ),
        launch_arguments={
            'model-file': PathJoinSubstitution([pkg_project_description, 'urdf', model_name]),
        }.items()
    )

    return LaunchDescription([
        world_name_launch_arg,
        model_name_launch_arg,
        gz_sim,
        bridge,
        spawn_model,
    ])