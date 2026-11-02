cmake_minimum_required(VERSION 3.8)
project(rover_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
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

add_library(differential_plugin SHARED src/DifferentialPlugin.cpp)
set_property(TARGET differential_plugin PROPERTY CXX_STANDARD 17)

target_link_libraries(differential_plugin
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  rclcpp::rclcpp
)

ament_target_dependencies(differential_plugin
  ros_gz
  rclcpp
)
ament_export_libraries(differential_plugin)

# Install compiled plugin to lib/ so ament_export_libraries can locate it
install(TARGETS differential_plugin
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

endif()

# Install C++ headers (for downstream packages)
install(DIRECTORY include/
  DESTINATION include
)

# Install all declarative and launch assets in one sweep
install(DIRECTORY launch config urdf meshes rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/gz_sim_resource_path.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/gz_sim_resource_path.sh")

ament_python_install_package(${PROJECT_NAME})

ament_package()