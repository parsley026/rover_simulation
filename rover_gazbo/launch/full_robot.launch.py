import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument,Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo=get_package_share_directory('rover_gazbo')
    pkg_description=get_package_share_directory('my_robot_description')



    robot_desription_launch= IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_description,'launch','robot_description.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()

    )


    gazebo_world=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo,'launch','krontonbot_apartment.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()

    )

    robot_spawner=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo,'launch','spawn_robot.launch.py')
        ),
    )

    rviz_node=Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': True}],
        output='screen',
        on_exit=Shutdown()
    )

    return LaunchDescription([
        robot_desription_launch,
        gazebo_world,
        robot_spawner,
        #rviz_node
    ])
