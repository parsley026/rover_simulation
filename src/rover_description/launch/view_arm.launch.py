from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Standalone robotic arm visualization (alias wrapping view_submodule.launch.py).
    """
    pkg_description = FindPackageShare('rover_description')

    show_gui     = LaunchConfiguration('gui')
    mount_height = LaunchConfiguration('mount_height')

    view_submodule = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_description, 'launch', 'view_submodule.launch.py'])
        ),
        launch_arguments={
            'submodule':    'robotic_arm',
            'gui':          show_gui,
            'mount_height': mount_height,
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui',          default_value='true', description='Launch joint_state_publisher_gui'),
        DeclareLaunchArgument('mount_height', default_value='0.8',  description='Z height of mount above world origin (m)'),
        view_submodule,
    ])
