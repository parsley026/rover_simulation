cmake_minimum_required(VERSION 3.8)
project(rover_robot_arm_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rex_interfaces REQUIRED)
find_package(libVescCan REQUIRED)

# Component node library
add_library(manipulator_simple_component SHARED
  src/ManipulatorSimple.cpp
)

target_include_directories(manipulator_simple_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(manipulator_simple_component
  rclcpp
  rclcpp_components
  rex_interfaces
)

target_link_libraries(manipulator_simple_component
  libVescCan::libVescCan
)

rclcpp_components_register_node(manipulator_simple_component
  PLUGIN "ManipulatorSimple"
  EXECUTABLE rover_robot_arm_kinematics_node
)

# Install
install(TARGETS
  manipulator_simple_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
