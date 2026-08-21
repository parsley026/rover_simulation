cmake_minimum_required(VERSION 3.8)
project(rover_kinematics_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rex_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
#find_package(rclpy REQUIRED)

add_executable(${PROJECT_NAME}_node
  src/main.cpp
  src/rover_kinematics_bridge.cpp
  src/core/conversions.cpp
  src/gazebo/gazebo_command_publisher.cpp
  src/bridge_modes/simulation_mode.cpp
  src/bridge_modes/digital_shadow_mode.cpp
  src/bridge_modes/digital_twin_mode.cpp
)

target_include_directories(${PROJECT_NAME}_node PUBLIC include
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  ament_index_cpp
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
  rex_interfaces
)

# Link dependencies to the executable
target_link_libraries(${PROJECT_NAME}_node
  "${rclcpp_LIBRARIES}"
)

# Install the executable
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

# install(
#   DIRECTORY scripts/
#   DESTINATION lib/${PROJECT_NAME}
#   FILES_MATCHING PATTERN "*.py"
# )

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

ament_package()
