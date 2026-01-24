cmake_minimum_required(VERSION 3.8)
project(gazebo_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ros_gz REQUIRED)
find_package(rover_description REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

find_package(rover_description REQUIRED)

find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
find_package(gz-common5 REQUIRED COMPONENTS profiler)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

# Harmonic
if("$ENV{GZ_VERSION}" STREQUAL "harmonic")
  find_package(gz-sim8 REQUIRED)
  set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
  message(STATUS "Compiling against Gazebo Harmonic")
# Default to Garden
# else()
#   find_package(gz-sim7 REQUIRED)
#   set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})
#   message(STATUS "Compiling against Gazebo Garden")
endif()


# add_library(BasicSystem
#   SHARED
#   src/BasicSystem.cc
# )

# target_include_directories(
#   BasicSystem PRIVATE include
# )

# target_link_libraries(BasicSystem PRIVATE
#   gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})


# # Following directives similarly specify another target: 'FullSystem'.
# add_library(FullSystem
#   SHARED
#   src/FullSystem.cc
# )

# target_include_directories(
#   FullSystem PRIVATE include
# )

# target_link_libraries(FullSystem PRIVATE
#   gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})

# Following 'install' directive ensures that the compiled libraries
# of the 'BasicSystem' and 'FullSystem' targets will be copied
# to the subfolder 'lib/ros_gz_example_gazebo' of the install directory.
# install(
#   TARGETS BasicSystem FullSystem
#   DESTINATION lib/${PROJECT_NAME}
# )

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

# Following hooks are used to ensure that the correct environment variables
# will be set by exectuting 'sourece install/setup.bash' after compilation.
# When using this template for your project, change the filenames of the
# files in the 'hooks' folder, to correspond to your project name.

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/gz_sim_resource_path.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/gz_sim_resource_path.sh")


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
