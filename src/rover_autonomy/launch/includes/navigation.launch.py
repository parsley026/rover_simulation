import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, SetParameter
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml

def launch_setup(context, *args, **kwargs):
    my_package_name = 'rover_autonomy'
    my_package_dir = get_package_share_directory(my_package_name)

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    navigation_mode = str(LaunchConfiguration('navigation_mode').perform(context))
    navigation_odom_only = LaunchConfiguration('navigation_odom_only').perform(context).lower() == 'true'
    launch_mapping = LaunchConfiguration('launch_mapping').perform(context).lower() == 'true'
    launch_local_topography = LaunchConfiguration('launch_local_topography').perform(context).lower() == 'true'
    launch_global_topography = LaunchConfiguration('launch_global_topography').perform(context).lower() == 'true'

    yaml_name = f"costmaps_{navigation_mode}"
    
    if launch_local_topography or launch_global_topography:
        yaml_name = "costmaps_topography" 
    
    if launch_mapping:
        yaml_name += "_mapping"
        
    yaml_name += ".yaml"
    
    core_params_path = os.path.join(my_package_dir, 'config', 'navigation', 'nav2_core.yaml')
    
    user_params_file = LaunchConfiguration('params_file').perform(context)
    if user_params_file and "navigation.yaml" not in user_params_file:
        costmap_params_path = user_params_file
    else:
        costmap_params_path = os.path.join(my_package_dir, 'config', 'navigation', yaml_name)
        
    if not os.path.exists(costmap_params_path):
        print(f"[WARN] Expected config {costmap_params_path} does not exist. Falling back to navigation.yaml")
        costmap_params_path = os.path.join(my_package_dir, 'config', 'navigation', 'navigation.yaml')
    else:
        print(f"[INFO] Using core config: {core_params_path}")
        print(f"[INFO] Using costmap config: {costmap_params_path}")

    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'velocity_smoother',
        'collision_monitor',
        'bt_navigator',
        'waypoint_follower',
    ]

    remappings = [
        ('tf', '/tf'),
        ('tf_static', '/tf_static'),
        ('/tf', '/tf'),
        ('/tf_static', '/tf_static'),
        ('/map', '/mapping/map'),
        ('goal_pose', '/goal_pose'),
        ('initialpose', '/initialpose')
    ]

    param_substitutions = {'autostart': autostart}
    
    if navigation_odom_only:
        param_substitutions['global_frame'] = 'odom'
        param_substitutions['rolling_window'] = 'True'

    configured_core_params = ParameterFile(
        RewrittenYaml(
            source_file=core_params_path,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    configured_costmap_params = ParameterFile(
        RewrittenYaml(
            source_file=costmap_params_path,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            Node(
                namespace=namespace,
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_core_params, configured_costmap_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                namespace=namespace,
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_core_params, configured_costmap_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                namespace=namespace,
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_core_params, configured_costmap_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                namespace=namespace,
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_core_params, configured_costmap_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                namespace=namespace,
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_core_params, configured_costmap_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                namespace=namespace,
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_core_params, configured_costmap_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                namespace=namespace,
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_core_params, configured_costmap_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                namespace=namespace,
                package='nav2_collision_monitor',
                executable='collision_monitor',
                name='collision_monitor',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_core_params, configured_costmap_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                namespace=namespace,
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'autostart': autostart}, {'node_names': lifecycle_nodes}],
            ),
        ],
    )

    load_composable_nodes = GroupAction(
        condition=IfCondition(use_composition),
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            Node(
                namespace=namespace,
                package='rclcpp_components',
                executable='component_container_isolated',
                name=container_name,
                output='screen',
                parameters=[configured_core_params, configured_costmap_params, {'autostart': autostart}],
                arguments=['--ros-args', '--log-level', log_level],
            ),
            LoadComposableNodes(
                target_container=container_name_full,
                composable_node_descriptions=[
                    ComposableNode(
                        namespace=namespace,
                        package='nav2_controller',
                        plugin='nav2_controller::ControllerServer',
                        name='controller_server',
                        parameters=[configured_core_params, configured_costmap_params],
                        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
                    ),
                    ComposableNode(
                        namespace=namespace,
                        package='nav2_smoother',
                        plugin='nav2_smoother::SmootherServer',
                        name='smoother_server',
                        parameters=[configured_core_params, configured_costmap_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        namespace=namespace,
                        package='nav2_planner',
                        plugin='nav2_planner::PlannerServer',
                        name='planner_server',
                        parameters=[configured_core_params, configured_costmap_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        namespace=namespace,
                        package='nav2_behaviors',
                        plugin='behavior_server::BehaviorServer',
                        name='behavior_server',
                        parameters=[configured_core_params, configured_costmap_params],
                        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
                    ),
                    ComposableNode(
                        namespace=namespace,
                        package='nav2_bt_navigator',
                        plugin='nav2_bt_navigator::BtNavigator',
                        name='bt_navigator',
                        parameters=[configured_core_params, configured_costmap_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        namespace=namespace,
                        package='nav2_waypoint_follower',
                        plugin='nav2_waypoint_follower::WaypointFollower',
                        name='waypoint_follower',
                        parameters=[configured_core_params, configured_costmap_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        namespace=namespace,
                        package='nav2_velocity_smoother',
                        plugin='nav2_velocity_smoother::VelocitySmoother',
                        name='velocity_smoother',
                        parameters=[configured_core_params, configured_costmap_params],
                        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
                    ),
                    ComposableNode(
                        namespace=namespace,
                        package='nav2_collision_monitor',
                        plugin='nav2_collision_monitor::CollisionMonitor',
                        name='collision_monitor',
                        parameters=[configured_core_params, configured_costmap_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        namespace=namespace,
                        package='nav2_lifecycle_manager',
                        plugin='nav2_lifecycle_manager::LifecycleManager',
                        name='lifecycle_manager_navigation',
                        parameters=[
                            {'autostart': autostart, 'node_names': lifecycle_nodes}
                        ],
                    ),
                ],
            ),
        ],
    )

    return [load_nodes, load_composable_nodes]


def generate_launch_description():
    my_package_name = 'rover_autonomy'
    my_package_dir = get_package_share_directory(my_package_name)

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(my_package_dir, 'config', 'navigation', 'navigation.yaml'), description='Full path to the ROS2 parameters file to use for all launched nodes'),
        DeclareLaunchArgument('autostart', default_value='true', description='Automatically startup the nav2 stack'),
        DeclareLaunchArgument('use_composition', default_value='False', description='Use composed bringup if True'),
        DeclareLaunchArgument('container_name', default_value='nav2_container', description='the name of conatiner that nodes will load in if use composition'),
        DeclareLaunchArgument('use_respawn', default_value='False', description='Whether to respawn if a node crashes. Applied when composition is disabled.'),
        DeclareLaunchArgument('log_level', default_value='info', description='log level'),
        
        # New arguments for dynamic mode switching
        DeclareLaunchArgument('navigation_mode', default_value='110', description='Navigation Mode (e.g. 110)'),
        DeclareLaunchArgument('navigation_odom_only', default_value='false', description='Navigate using only odom'),
        DeclareLaunchArgument('launch_mapping', default_value='false', description='Is mapping active?'),
        DeclareLaunchArgument('launch_local_topography', default_value='false', description='Is local topography active?'),
        DeclareLaunchArgument('launch_global_topography', default_value='false', description='Is global topography active?'),

        OpaqueFunction(function=launch_setup)
    ])