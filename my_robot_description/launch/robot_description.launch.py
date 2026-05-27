import os 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument 
from launch.substitutions import LaunchConfiguration,Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_prefix


def generate_launch_description():

    #the most important thing is to make it has the ability to reach the path inside the share directory
    
    package_description='my_robot_description'
    pkg_share=get_package_share_directory('my_robot_description')
     




    #the second is to get the urdf path

    #urdf_path=os.path.join(pkg_share,'urdf','krontonbot.urdf')



    # Set the Path to Robot Mesh Models for Loading in Gazebo Sim
    
    install_description_dir_path = get_package_prefix(package_description) + "/share"

    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        if install_description_dir_path not in os.environ["GZ_SIM_RESOURCE_PATH"]:
            os.environ["GZ_SIM_RESOURCE_PATH"] += (':' + install_description_dir_path)
    else:
        os.environ["GZ_SIM_RESOURCE_PATH"] = (':'.join(install_description_dir_path))

    urdf_path = os.path.join(pkg_share,'urdf','rover.xacro')






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

   

    
    return LaunchDescription([
        robot_state_publisher_node
        
       
    ])