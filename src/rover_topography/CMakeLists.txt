cmake_minimum_required(VERSION 3.16)
project(rover_topography)

# ── C++ standard ─────────────────────────────────────────────────────────────
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_filters REQUIRED)
find_package(filters REQUIRED)
find_package(message_filters REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# ── Shared component library ──────────────────────────────────────────────────
add_library(${PROJECT_NAME}_component SHARED
  src/core/map_bounds_strategy.cpp
  src/core/elevation_fuser.cpp
  src/ros/topography_node.cpp
)

target_include_directories(${PROJECT_NAME}_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(${PROJECT_NAME}_component
  rclcpp
  rclcpp_components
  ament_index_cpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
  grid_map_core
  grid_map_ros
  grid_map_msgs
  grid_map_filters
  filters
  message_filters
  Eigen3
  yaml-cpp
)

target_link_libraries(${PROJECT_NAME}_component
  yaml-cpp
)

# Register composable node plugin
rclcpp_components_register_node(${PROJECT_NAME}_component PLUGIN "rover_topography::ros::TopographyNode" EXECUTABLE topography_component_node)

# ── Standalone executable ─────────────────────────────────────────────────────
add_executable(topography_node src/ros/main.cpp)
target_include_directories(topography_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(topography_node ${PROJECT_NAME}_component)
ament_target_dependencies(topography_node rclcpp ament_index_cpp)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS
  ${PROJECT_NAME}_component
  topography_component_node
  topography_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

# ── Testing ───────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_add_gtest(test_topography_core tests/test_topography_core.cpp)
  target_include_directories(test_topography_core PUBLIC include)
  target_link_libraries(test_topography_core ${PROJECT_NAME}_component)
  ament_target_dependencies(test_topography_core grid_map_core sensor_msgs)
endif()

ament_package()
