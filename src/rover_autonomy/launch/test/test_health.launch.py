import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('rover_autonomy')
    
    health_config = os.path.join(pkg_dir, 'config', 'test', 'test_health.yaml')
    
    return LaunchDescription([
        # 1. The Mock Publisher
        Node(
            package='rover_autonomy',
            executable='mock_health_publisher.py',
            name='mock_health_publisher',
            output='screen'
        ),
        
        # 2. Odometry Health Node
        Node(
            package='rover_autonomy',
            executable='odometry_health_node',
            name='odometry_health_node',
            parameters=[health_config],
            remappings=[
                ('odom_raw', '/camera_00/odom_raw'),
                ('odom', '/camera_00/odom')
            ],
            output='screen'
        )
    ])
