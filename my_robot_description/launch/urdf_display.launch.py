import os 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument 
from launch.substitutions import LaunchConfiguration,Command
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    #the most important thing is to make it has the ability to reach the path inside the share directory
    

    pkg_share=get_package_share_directory('krontonbot_description')
     
    #the second is to get the urdf path

    urdf_path=os.path.join(pkg_share,'urdf','krontonbot.urdf')

    #i need to path the urdf as a parameter to robot_state_publisher

    robot_description=ParameterValue(
        Command(['xacro ',urdf_path]),
        value_type=str
    )

    robot_state_publisher_node=Node(

        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description':robot_description}]
    )

    joint_state_publisher_node= Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'

    )


   


    
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node

       
        
         
         
       
    ])