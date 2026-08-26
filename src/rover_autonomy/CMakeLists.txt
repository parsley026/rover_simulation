cmake_minimum_required(VERSION 3.8)
project(rover_autonomy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2_msgs REQUIRED)

# ── Shared component library ──────────────────────────────────────────────────
add_library(main_compute_component SHARED
  src/main_compute_node.cpp
  src/parameter_manager.cpp
  src/subsystems/process_subsystem_manager.cpp
  src/subsystems/composite_subsystem_manager.cpp
  src/subsystems/camera_subsystem_manager.cpp
  src/subsystems/lidar_subsystem_manager.cpp
  src/subsystems/description_subsystem_manager.cpp
  src/subsystems/odometry_subsystem_manager.cpp
  src/subsystems/localization_and_mapping_subsystem_manager.cpp
  src/subsystems/navigation_subsystem_manager.cpp
)

target_include_directories(main_compute_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(main_compute_component
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  ament_index_cpp
  tf2_msgs
)

# Register node for component loading and create standalone executable
rclcpp_components_register_node(
  main_compute_component
  PLUGIN "rover_autonomy::MainComputeNode"
  EXECUTABLE main_compute_node
)

# Install directories
install(DIRECTORY
  config  
  launch
  urdf
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  main_compute_component
  main_compute_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
