cmake_minimum_required(VERSION 3.8)
project(rover_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz2 REQUIRED)
find_package(urdf REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(gz-cmake3 REQUIRED)
#find_package(gz-sim8 REQUIRED)
#find_package(gz-sim8 REQUIRED COMPONENTS sim)
find_package(ros_gz REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(gazebo_dev REQUIRED)

# Jazzy
find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
message(STATUS "Compiling with gz-sim8")


# Find Gazebo libraries
gz_find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
gz_find_package(gz-common5 REQUIRED)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})


add_library(differential_plugin SHARED src/differential_plugin.cpp)
set_property(TARGET differential_plugin PROPERTY CXX_STANDARD 17)
# target_link_libraries(BasicSystem PRIVATE
#   gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})

target_link_libraries(differential_plugin
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  ${gazebo_ros_LIBRARIES}
  rclcpp::rclcpp
)

ament_target_dependencies(differential_plugin
  #gazebo_dev
  ros_gz
  rclcpp
)
ament_export_libraries(differential_plugin)

install(
  TARGETS differential_plugin
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch config urdf src mesh
  DESTINATION share/${PROJECT_NAME}/
)


ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/gz_sim_resource_path.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/gz_sim_resource_path.sh")

ament_package()
