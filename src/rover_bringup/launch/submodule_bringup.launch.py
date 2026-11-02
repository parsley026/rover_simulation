import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    submodule                 = LaunchConfiguration('submodule').perform(context).strip()
    use_sim_time              = LaunchConfiguration('use_sim_time')
    mount_height              = LaunchConfiguration('mount_height')
    include_sensors           = LaunchConfiguration('include_sensors')
    run_arm_kinematics        = LaunchConfiguration('run_arm_kinematics').perform(context).lower() == 'true'
    run_arm_kinematics_bridge = LaunchConfiguration('run_arm_kinematics_bridge').perform(context).lower() == 'true'

    # Build submodule /robot_description from submodule_standalone.urdf.xacro
    submodule_urdf_content = Command([
        'xacro "',
        PathJoinSubstitution([FindPackageShare('rover_description'), 'urdf', 'submodule_standalone.urdf.xacro']),
        '" submodule:=', submodule,
        ' use_sim_time:=', use_sim_time,
        ' mount_height:=', mount_height,
        ' include_sensors:=', include_sensors,
    ])
    submodule_urdf_param = ParameterValue(submodule_urdf_content, value_type=str)

    # Robot State Publisher for the submodule
    rsp_node = Node(
        package   ='robot_state_publisher',
        executable='robot_state_publisher',
        name      ='robot_state_publisher',
        namespace ='',
        output    ='screen',
        parameters=[{
            'robot_description': submodule_urdf_param,
            'use_sim_time':      use_sim_time,
        }],
    )

    nodes = [rsp_node]

    # Submodule-specific state publishing & controllers
    if submodule == 'robotic_arm':
        # Joint state merger for arm controllers: aggregates /arm/joint_states -> /joint_states
        joint_state_merger = Node(
            package   ='joint_state_publisher',
            executable='joint_state_publisher',
            name      ='joint_state_merger',
            output    ='screen',
            parameters=[{
                'source_list':               ['/arm/joint_states'],
                'rate':                      100,
                'publish_default_velocities': True,
                'use_sim_time':              use_sim_time,
            }],
        )
        nodes.append(joint_state_merger)

        # Optional: arm kinematics bridge
        if run_arm_kinematics_bridge:
            pkg_arm_bridge = FindPackageShare('rover_robot_arm_kinematics_bridge')
            nodes.append(IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    PathJoinSubstitution([
                        pkg_arm_bridge, 'launch',
                        'rover_robot_arm_kinematics_bridge.launch.xml',
                    ])),
                launch_arguments={'mode': 'simulation'}.items()
            ))

        # Optional: arm kinematics controller
        if run_arm_kinematics:
            pkg_arm_kinematics = FindPackageShare('rover_robot_arm_kinematics')
            nodes.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        pkg_arm_kinematics, 'launch',
                        'rover_robot_arm_kinematics.launch.py',
                    ])),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            ))

    return nodes


def generate_launch_description():
    """
    Bringup for any standalone rover submodule (robotic_arm, autonomy, crossbar, sampler).

    Starts robot_state_publisher for the chosen submodule mounted on a standalone pedestal,
    joint state merging (if applicable), and optional kinematics nodes.
    """
    return LaunchDescription([
        DeclareLaunchArgument('submodule',                 default_value='robotic_arm', description='Submodule to bring up: robotic_arm | autonomy | crossbar | sampler'),
        DeclareLaunchArgument('use_sim_time',              default_value='true',        description='Use simulation clock'),
        DeclareLaunchArgument('mount_height',              default_value='0.8',         description='Z height of mount above world frame (m)'),
        DeclareLaunchArgument('include_sensors',           default_value='true',        description='Attach submodule sensors if available'),
        DeclareLaunchArgument('run_arm_kinematics',        default_value='false',       description='Launch rover_robot_arm_kinematics (robotic_arm only)'),
        DeclareLaunchArgument('run_arm_kinematics_bridge', default_value='false',       description='Launch rover_robot_arm_kinematics_bridge (robotic_arm only)'),
        OpaqueFunction(function=launch_setup),
    ])
