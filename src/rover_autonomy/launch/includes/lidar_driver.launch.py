from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    EmitEvent,
    LogInfo,
    OpaqueFunction,
)
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import launch

import lifecycle_msgs.msg

def launch_setup(context, *args, **kwargs):
    urdf_package = LaunchConfiguration('urdf_package').perform(context)
    urdf_file    = LaunchConfiguration('urdf_file')   .perform(context)

    lidar_ns   = LaunchConfiguration('lidar_ns').perform(context)
    lidar_name = LaunchConfiguration('lidar_name').perform(context)
    
    lidar_parent_frame = LaunchConfiguration('lidar_parent_frame').perform(context)

    params_file = LaunchConfiguration('params_file').perform(context)
    
    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'),
        value_type=bool
    )

    lidar_description = {
        'robot_description': ParameterValue(
            Command([
                FindExecutable(name='xacro'),
                ' "',
                PathJoinSubstitution([
                    FindPackageShare(urdf_package),
                    'urdf',
                    urdf_file
                ]),
                '"',
                f" lidar_name:={lidar_name}",
                f" parent_frame:={lidar_parent_frame}",
            ]),
            value_type=str
        )
    }

    description_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name=f"{lidar_name}_state_publisher",
        namespace=lidar_ns,
        parameters=[lidar_description],
        remappings=[
            ('tf', '/tf'),
            ('tf_static', '/tf_static')
        ],
        condition=UnlessCondition(LaunchConfiguration('use_sim_time'))
    )

    ouster_node = LifecycleNode(
        package="ouster_ros",
        executable="os_driver",
        name=lidar_name,
        namespace=lidar_ns,
        parameters=[params_file],
        output="screen",
        condition=UnlessCondition(LaunchConfiguration('use_sim_time'))
    )

    sensor_configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(ouster_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        ),
        condition=UnlessCondition(LaunchConfiguration('use_sim_time'))
    )

    sensor_activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=ouster_node,
            goal_state="inactive",
            entities=[
                LogInfo(msg="os_driver activating..."),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(ouster_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
            handle_once=True,
        ),
        condition=UnlessCondition(LaunchConfiguration('use_sim_time'))
    )

    sensor_finalized_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=ouster_node,
            goal_state="finalized",
            entities=[
                LogInfo(
                    msg="Failed to communicate with the sensor in a timely manner."
                ),
                EmitEvent(
                    event=launch.events.Shutdown(
                        reason="Couldn't communicate with sensor"
                    )
                ),
            ],
        ),
        condition=UnlessCondition(LaunchConfiguration('use_sim_time'))
    )
    
    return [description_node, ouster_node, sensor_configure_event, sensor_activate_event, sensor_finalized_event]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('urdf_package', default_value='rover_autonomy',   description=''),
        DeclareLaunchArgument('urdf_file',    default_value='lidar.urdf.xacro', description=''),
        
        DeclareLaunchArgument('lidar_ns',   default_value='lidar_00', description=''),
        DeclareLaunchArgument('lidar_name', default_value='lidar_00', description=''),

        DeclareLaunchArgument('lidar_parent_frame', default_value='base_link', description=''),

        DeclareLaunchArgument('params_file',  default_value='',      description=''),
        DeclareLaunchArgument('use_sim_time', default_value='false', description=''),

        OpaqueFunction(function=launch_setup)
    ])