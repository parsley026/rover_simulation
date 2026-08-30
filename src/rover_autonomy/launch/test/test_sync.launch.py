import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('rover_autonomy')
    
    sync_config = os.path.join(pkg_dir, 'config', 'test', 'test_sync.yaml')
    
    return LaunchDescription([
        # 1. The Mock Publisher
        Node(
            package='rover_autonomy',
            executable='mock_sync_publisher.py',
            name='mock_sync_publisher',
            output='screen'
        ),
        
        # 2. Generic Temporal Sync Node
        Node(
            package='rover_autonomy',
            executable='generic_temporal_sync_node',
            name='test_sync_node',
            parameters=[sync_config],
            output='screen'
        )
    ])
