# Quad Rover Kinematics

This package provides kinematic calculations and control for a quad rover robot. It includes functionalities for different driving modes such as advance, crab, and spin.

## Description

This branch contains customizations in order to work with [raptor_ws](https://github.com/wisniax/raptor_ws)

## Dependencies

To build and run this package, you need to install the following dependencies:

- ROS 2 (Foxy, Galactic, Humble, or Rolling)
- ament_cmake
- rclcpp
- geometry_msgs
- nav_msgs
- realtime_tools
- sensor_msgs
- std_msgs
- tf2
- tf2_ros
- Eigen3
- yaml-cpp
- rex_interfaces
- tf2_geometry_msgs
- std_srvs
- ceres-solver

### Installation Commands

```bash
# Install ROS 2 (example for ROS 2 Foxy)
sudo apt update && sudo apt install -y ros-jazzy-desktop

# Install dependencies
sudo apt install -y \
  ros-jazzy-ament-cmake \
  ros-jazzy-rclcpp \
  ros-jazzy-geometry-msgs \
  ros-jazzy-nav-msgs \
  ros-jazzy-realtime-tools \
  ros-jazzy-sensor-msgs \
  ros-jazzy-std-msgs \
  ros-jazzy-tf2 \
  ros-jazzy-tf2-ros \
  libeigen3-dev \
  libyaml-cpp-dev \
  ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-std-srvs \
```

## Ceres lover installation

Please follow the guide for downloading the ceres-slover: [Ceres Solver Installation Guide](http://ceres-solver.org/installation.html)

Place the package into "/src" folder of your workspace

Build with:

```bash
colcon build --packages-select ceres-solver
```

## Building the Package

Navigate to the workspace directory and build the package using colcon:

```bash
cd ~/kinematics_ws
colcon build --packages-select rover_kinematics
```

## Running the Package

Source the workspace and run the node:

```bash
source ~/kinematics_ws/install/setup.bash
ros2 run rover_kinematics rover_kinematics_node
```

## Configuration

The package uses a YAML configuration file located at `config/rover_config.yaml`. You can modify this file to set parameters such as wheel dimensions, steering radius, and odometry covariances.

Example configuration:

```yaml
wheels_distance_length: 0.9  # [m]
wheels_distance_width: 0.654 # [m] 
wheel_diameter: 0.256        # [m]
poles_pair_number: 15        # Number of poles in the motor
velocity_limit: 1.0          # [m/s]
publish_rate: 20.0           # [Hz] (Recommended: 20.0, Maximum: 50.0)
use_sim_time: true           # Use simulation time

max_steering_radius: 5.0     # [m] Maximum steering radius of the robot
min_steering_radius: 1.0     # [m] Minimum steering radius of the robot

pose_covariance_diagonal:  [0.001, 0.001, 0.001, 0.001, 0.001, 0.03]  
twist_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.03]

base_frame_id: '/base_footprint'
odom_frame_id: '/odom'
enable_odom_tf: false
```

## Phase 3 architecture notes

The odometry estimator has been extracted into a dedicated forward-kinematics component.

- Estimator API: `OdometryEstimate update(const rex_interfaces::msg::Wheels& feedback, const rclcpp::Time& timestamp)`
- Internal state: wheel feedback history, body pose, body twist, timestamp, covariance setup, and the Ceres solver guesses
- Solver ownership: the estimator owns its own Ceres `Problem` and solver configuration per update call, so the ROS node does not manage solver state
- Dependency graph: `Rover -> Kinematics -> KinematicsEstimator` and `Kinematics -> KinematicsSolver`
- Files modified: `include/rover_kinematics/KinematicsEstimator.hpp`, `src/KinematicsEstimator.cpp`, `include/rover_kinematics/kinematics.h`, `src/kinematics.cpp`, and `CMakeLists.txt`
- Moved responsibilities: steering-angle conversion, wheel velocity projection, Ceres optimization, pose integration, timestamp handling, covariance initialization, and odometry/TF message population are now owned by the estimator

## Nodes

### rover_kinematics_node

This node handles the kinematic calculations and control for the quad rover.

#### Subscribed Topics

- `/MQTT/RoverControl` (rex_interfaces/msg/RoverControl)
- `/ackermann_drive_controller/cmd_vel` (geometry_msgs/msg/Twist)
- `/MQTT/RoverStatus` (rex_interfaces/msg/RoverStatus)
- `/CAN/RX/vesc_status` (rex_interfaces/msg/VescStatus)
- `/clock` (rosgraph_msgs/msg/Clock)

#### Published Topics

- `/CAN/TX/set_motor_vel` (rex_interfaces/msg/Wheels)
- `/kinematic/odometry` (nav_msgs/msg/Odometry)
- `/tf` (tf2_msgs/msg/TFMessage)

#### Services

- `reset_odometry` (std_srvs/srv/Empty)

## License

This package is licensed under the BSD License.
