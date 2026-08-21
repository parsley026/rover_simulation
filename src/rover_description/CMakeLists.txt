cmake_minimum_required(VERSION 3.8)
project(rover_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz2 REQUIRED)
find_package(urdf REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(gz-cmake3 REQUIRED)
find_package(ros_gz REQUIRED)

find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
message(STATUS "Compiling with gz-sim8")

gz_find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
gz_find_package(gz-common5 REQUIRED)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

include_directories(include)

add_library(differential_plugin SHARED src/differential_plugin.cpp)
set_property(TARGET differential_plugin PROPERTY CXX_STANDARD 17)

target_link_libraries(differential_plugin
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  ${gazebo_ros_LIBRARIES}
  rclcpp::rclcpp
)

ament_target_dependencies(differential_plugin
  ros_gz
  rclcpp
)
ament_export_libraries(differential_plugin)

# ---- VESC Drive Plugin ----
add_library(vesc_drive_plugin SHARED src/VescDrivePlugin.cpp)
set_property(TARGET vesc_drive_plugin PROPERTY CXX_STANDARD 17)

target_link_libraries(vesc_drive_plugin
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  ${gazebo_ros_LIBRARIES}
  rclcpp::rclcpp
)

ament_target_dependencies(vesc_drive_plugin
  ros_gz
  rclcpp
)
ament_export_libraries(vesc_drive_plugin)

# ---- VESC Steer Plugin ----
add_library(vesc_steer_plugin SHARED src/VescSteerPlugin.cpp)
set_property(TARGET vesc_steer_plugin PROPERTY CXX_STANDARD 17)

target_link_libraries(vesc_steer_plugin
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  ${gazebo_ros_LIBRARIES}
  rclcpp::rclcpp
)

ament_target_dependencies(vesc_steer_plugin
  ros_gz
  rclcpp
)
ament_export_libraries(vesc_steer_plugin)

add_library(custom_camera_noise_plugin SHARED src/CustomCameraNoisePlugin.cpp)
set_property(TARGET custom_camera_noise_plugin PROPERTY CXX_STANDARD 17)

target_link_libraries(custom_camera_noise_plugin
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  ${gazebo_ros_LIBRARIES}
  rclcpp::rclcpp
)

ament_target_dependencies(custom_camera_noise_plugin
  ros_gz
  rclcpp
)
ament_export_libraries(custom_camera_noise_plugin)

install(
  TARGETS
    differential_plugin
    custom_camera_noise_plugin
    vesc_drive_plugin
    vesc_steer_plugin
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch config urdf src meshes
  DESTINATION share/${PROJECT_NAME}/
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/gz_sim_resource_path.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/gz_sim_resource_path.sh")

ament_package()