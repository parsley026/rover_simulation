import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    submodule       = LaunchConfiguration('submodule').perform(context).strip()
    mount_height    = LaunchConfiguration('mount_height')
    include_sensors = LaunchConfiguration('include_sensors')
    show_gui        = LaunchConfiguration('gui')

    pkg_rover_bringup     = FindPackageShare('rover_bringup')
    pkg_rover_description = get_package_share_directory('rover_description')

    # Bringup submodule robot_state_publisher
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_rover_bringup, 'launch', 'submodule_bringup.launch.py'])
        ),
        launch_arguments={
            'submodule':       submodule,
            'use_sim_time':    'false',
            'mount_height':    mount_height,
            'include_sensors': include_sensors,
        }.items()
    )

    # For robotic_arm, joint_state_merger reads /arm/joint_states;
    # for static submodules without a merger, GUI publishes directly to /joint_states.
    if submodule == 'robotic_arm':
        jsp_gui_node = Node(
            condition=IfCondition(show_gui),
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            remappings=[('joint_states', '/arm/joint_states')],
            output='screen',
        )
    else:
        jsp_gui_node = Node(
            condition=IfCondition(show_gui),
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        )

    rviz_config = os.path.join(pkg_rover_description, 'rviz', 'urdf.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return [bringup_cmd, jsp_gui_node, rviz_node]


def generate_launch_description():
    """
    Visualize any standalone rover submodule (robotic_arm, autonomy, crossbar, sampler)
    in RViz2 with joint sliders.

    Usage examples
    --------------
    ros2 launch rover_description view_submodule.launch.py submodule:=robotic_arm
    ros2 launch rover_description view_submodule.launch.py submodule:=autonomy
    ros2 launch rover_description view_submodule.launch.py submodule:=crossbar
    ros2 launch rover_description view_submodule.launch.py submodule:=sampler
    """
    return LaunchDescription([
        DeclareLaunchArgument('submodule',       default_value='robotic_arm', description='Submodule: robotic_arm | autonomy | crossbar | sampler'),
        DeclareLaunchArgument('mount_height',    default_value='0.8',         description='Z height of mount above world origin (m)'),
        DeclareLaunchArgument('include_sensors', default_value='true',        description='Attach submodule sensors if available'),
        DeclareLaunchArgument('gui',             default_value='true',        description='Launch joint_state_publisher_gui'),
        OpaqueFunction(function=launch_setup),
    ])
