from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch.actions import EmitEvent, RegisterEventHandler, DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.events import matches_action
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch.event_handlers import OnProcessStart
import lifecycle_msgs.msg

def generate_launch_description():
    pkg_share = FindPackageShare('rover_autonomy')
    default_params_file = PathJoinSubstitution([pkg_share, 'config', 'main_compute.yaml'])

    params_file = LaunchConfiguration('params_file')

    main_compute_node = LifecycleNode(
        package='rover_autonomy',
        executable='main_compute_node',
        name='main_compute',
        namespace='',
        output='screen',
        parameters=[params_file]
    )

    # Automatically transition to configure when node starts
    configure_event = RegisterEventHandler(
        OnProcessStart(
            target_action=main_compute_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(main_compute_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE
                    )
                )
            ]
        )
    )

    # Automatically transition to active only when transitioning from unconfigured to inactive
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=main_compute_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(main_compute_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE
                    )
                )
            ]
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params_file, description='Path to config file for MainComputeNode'),
        main_compute_node,
        configure_event,
        activate_event
    ])
