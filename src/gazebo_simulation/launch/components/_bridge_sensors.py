from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_gazebo = get_package_share_directory('gazebo_simulation')
    
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    use_sim_time = LaunchConfiguration('use_sim_time')

    config_file = PathJoinSubstitution([pkg_gazebo, 'config', '_bridge_sensors.yaml'])

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': config_file,
            'use_sim_time': use_sim_time,
        }],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg, 
        gz_bridge
    ])