from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # Declare arguments so you can override them from the terminal
    max_vel_arg = DeclareLaunchArgument('max_vel', default_value='1.0', description='Maximum speed limit')
    step_size_arg = DeclareLaunchArgument('step_size', default_value='0.05', description='Acceleration step size')
    odom_topic_arg = DeclareLaunchArgument('odom_topic', default_value='/kinematic/odometry', description='Odometry topic to track')

    # Launch the keyboard teleop node
    teleop_node = Node(
        package='rover_teleop',
        executable='teleop_keyboard',  # This name is defined in your setup.py!
        name='teleop_keyboard',
        output='screen',
        emulate_tty=True, # CRITICAL: Required for reading keyboard inputs inside a launch file!
        parameters=[{
            'max_vel': LaunchConfiguration('max_vel'),
            'step_size': LaunchConfiguration('step_size'),
            'odom_topic': LaunchConfiguration('odom_topic')
        }]
    )

    return LaunchDescription([
        max_vel_arg,
        step_size_arg,
        odom_topic_arg,
        teleop_node
    ])