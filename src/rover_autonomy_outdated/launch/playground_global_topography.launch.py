from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rover_autonomy_outdated',
            executable='topography_global_node', # The name of your executable
            name='python_elevation_mapper',
            output='screen'
        )
    ])