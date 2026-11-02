import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    use_sim_time   = LaunchConfiguration('use_sim_time')
    mode           = LaunchConfiguration('mode')
    payload_config = LaunchConfiguration('payload_config')

    run_kinematics_bridge     = LaunchConfiguration('run_kinematics_bridge').perform(context).lower() == 'true'
    run_arm_kinematics_bridge = LaunchConfiguration('run_arm_kinematics_bridge').perform(context).lower() == 'true'
    run_kinematics            = LaunchConfiguration('run_kinematics').perform(context).lower() == 'true'
    run_arm_kinematics        = LaunchConfiguration('run_arm_kinematics').perform(context).lower() == 'true'
    run_bridge                = LaunchConfiguration('run_bridge').perform(context).lower() == 'true'
    entity_name               = LaunchConfiguration('entity_name').perform(context)

    base_robot_description_content = Command([
        'xacro "',
        PathJoinSubstitution([FindPackageShare('rover_description'), 'urdf', 'rover.urdf.xacro']),
        '" use_sim_time:=', use_sim_time,
        ' payload_config:="', payload_config, '"',
    ])
    base_robot_description_param = ParameterValue(base_robot_description_content, value_type=str)

    base_rsp_node = LifecycleNode(
        package   ='robot_state_publisher',
        executable='robot_state_publisher',
        name      ='robot_state_publisher',
        namespace ='',
        output    ='screen',
        parameters=[{
            'robot_description': base_robot_description_param,
            'use_sim_time':      use_sim_time,
        }],
    )

    nodes = [base_rsp_node]

    from launch_ros.actions import Node as RosNode

    # Dynamic ROS-Gz parameter bridge for rover chassis, submodules, and sensors.
    # Launched when in simulation mode (use_sim_time=true) and run_bridge=true.
    if (use_sim_time.perform(context).lower() == 'true') and run_bridge:
        from rover_description.bridge_generator import generate_active_bridge_config

        payload_file = payload_config.perform(context)
        bridge_config_path = generate_active_bridge_config(
            payload_file=payload_file,
            entity_name=entity_name
        )

        rover_bridge = RosNode(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='rover_bridge',
            output='screen',
            parameters=[{
                'config_file': bridge_config_path,
                'use_sim_time': use_sim_time,
            }],
        )
        nodes.append(rover_bridge)

    # Joint state merger: aggregates /chassis/joint_states and /arm/joint_states
    # into a single /joint_states topic for robot_state_publisher and MoveIt 2 (hardware / digital_shadow only).
    # In simulation (use_sim_time=true), gz_ros2_control's joint_state_broadcaster publishes directly
    # to /joint_states for all chassis (and optional arm) joints, so joint_state_merger must NOT run.
    if not (use_sim_time.perform(context).lower() == 'true'):
        joint_state_merger = RosNode(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_merger',
            output='screen',
            parameters=[{
                'source_list': ['/chassis/joint_states', '/arm/joint_states'],
                'rate': 100,
                'publish_default_velocities': True,
                'use_sim_time': use_sim_time,
            }],
        )
        nodes.append(joint_state_merger)

    # - conditional - rover kinematics bridge
    if run_kinematics_bridge:
        pkg_rover_kinematics_bridge = FindPackageShare('rover_kinematics_bridge')
        nodes.append(IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                PathJoinSubstitution([
                    pkg_rover_kinematics_bridge, 'launch',
                    'rover_kinematics_bridge.launch.xml',
                ])),
            launch_arguments={'mode': mode}.items()
        ))

    # - conditional - rover arm kinematics bridge
    if run_arm_kinematics_bridge:
        pkg_rover_robot_arm_kinematics_bridge = FindPackageShare('rover_robot_arm_kinematics_bridge')
        nodes.append(IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                PathJoinSubstitution([
                    pkg_rover_robot_arm_kinematics_bridge, 'launch',
                    'rover_robot_arm_kinematics_bridge.launch.xml',
                ])),
            launch_arguments={'mode': mode}.items()
        ))

    # - conditional - rover chassis kinematics
    if run_kinematics:
        pkg_rover_kinematics = FindPackageShare('rover_kinematics')
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    pkg_rover_kinematics, 'launch',
                    'rover_kinematics.launch.py',
                ])),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ))

    # - conditional - rover robot arm kinematics controller
    if run_arm_kinematics:
        pkg_rover_robot_arm_kinematics = FindPackageShare('rover_robot_arm_kinematics')
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    pkg_rover_robot_arm_kinematics, 'launch',
                    'rover_robot_arm_kinematics.launch.py',
                ])),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',              default_value='false', description='Use simulation time'),
        DeclareLaunchArgument('mode',                      default_value='simulation', description='Bridge mode: simulation | digital_shadow'),
        DeclareLaunchArgument('run_kinematics_bridge',     default_value='true',  description='Launch rover kinematics bridge'),
        DeclareLaunchArgument('run_arm_kinematics_bridge', default_value='true',  description='Launch rover arm kinematics bridge'),
        DeclareLaunchArgument('run_kinematics',            default_value='false', description='Launch rover chassis kinematics node'),
        DeclareLaunchArgument('run_arm_kinematics',        default_value='false', description='Launch rover robot arm kinematics controller node'),
        DeclareLaunchArgument('run_bridge',                default_value='true',  description='Launch dynamic ros_gz_bridge for sensors when use_sim_time:=true'),
        DeclareLaunchArgument('entity_name',               default_value='rover', description='Name of the robot model in simulation'),
        DeclareLaunchArgument('payload_config',            default_value=PathJoinSubstitution([FindPackageShare('rover_bringup'), 'config', 'payload.yaml']), description='Path to payload.yaml config file'),
        OpaqueFunction(function=launch_setup),
    ])

