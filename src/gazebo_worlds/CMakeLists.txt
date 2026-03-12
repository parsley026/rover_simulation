cmake_minimum_required(VERSION 3.8)
project(gazebo_worlds)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

# Install Gazebo resources
install(
  DIRECTORY models worlds media
  DESTINATION share/${PROJECT_NAME}
)

 file(GLOB xacro_files
   "${CMAKE_CURRENT_SOURCE_DIR}/worlds/*.world.xacro"
   "${CMAKE_CURRENT_SOURCE_DIR}/worlds/sdf/*.sdf.xacro"
 )
 xacro_add_files(${xacro_files} TARGET generate_world_files)

# Environment hooks (so gz sim can find models/worlds/media after `source install/setup.bash`)
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/gz_sim_resource_path.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/gz_sim_resource_path.sh")

ament_package()
