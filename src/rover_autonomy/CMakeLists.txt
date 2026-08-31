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
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rtabmap_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

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
  std_srvs
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
  mesh
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

# ── Utilities ─────────────────────────────────────────────────────────────────
add_executable(generic_temporal_sync_node src/utils/generic_temporal_sync_node.cpp)
ament_target_dependencies(generic_temporal_sync_node rclcpp)

install(TARGETS generic_temporal_sync_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(odometry_health_node src/utils/odometry_health_node.cpp)
target_include_directories(odometry_health_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(odometry_health_node rclcpp nav_msgs diagnostic_msgs rtabmap_msgs geometry_msgs std_srvs tf2)

install(TARGETS odometry_health_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS 
  scripts/mock_sync_publisher.py 
  scripts/mock_health_publisher.py 
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
