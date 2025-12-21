# Rex Interfaces

## Overview
Rex Interfaces is a ROS2 package containing custom message definitions for the Rex robotic system. These interfaces define the communication protocol for controlling and monitoring various components of the Rex robot, including:

- Rover mobility control and status
- Manipulator arm control
- Probe/soil analysis functionality
- VESC motor commands and status

## Message Types

### Rover Control
- **RoverControl**: Commands for controlling the rover's movement
  - Velocity and directional control (x-axis, y-axis)
  - Kinematic mode (4WS Ackermman, Omnidirectional, Spinning)
- **RoverStatus**: Rover status information
  - Communication state
  - Controller connection status
  - Current control mode

### Wheel Control
- **Wheels**: Combined information for all four wheels
  - Front left, front right, rear left, and rear right wheel data
- **Wheel**: Individual wheel control
  - Drive and turn motor commands

### Manipulator Control
- **ManipulatorControl**: Commands for controlling the robotic arm
  - 6-DOF axis control
  - Gripper control

### Probe System
- **ProbeControl**: Control commands for the soil analysis probe
  - Drill movement and action control
  - Platform positioning
  - Container rotation commands
- **ProbeStatus**: Soil analysis sensor data
  - Weight measurements
  - Distance, humidity, and vibration detection
  - Soil composition data (potassium, nitrogen, phosphorus, pH)

### Motor Control
- **VescMotorCommand**: Low-level motor control commands
  - Command ID and set value
  - for best backward compatibility and including recent changes in libVescCan:
    - set_origin_data - used only with SetOrigin command ID
    - set_pos_speed_loop... - used only with SetPosSpeedLoop command ID

- **VescStatus**: VESC motor controller status feedback
  - Motor parameters (ERPM, current, duty cycle)
  - Temperature readings
  - Battery metrics (voltage, amp-hours)
  - Position and sensor data

## Usage
To use these interfaces in your ROS2 package:

1. Add a dependency in your `package.xml`:
   ```xml
   <depend>rex_interfaces</depend>
   ```

2. Include in your CMakeLists.txt:
   ```cmake
   find_package(rex_interfaces REQUIRED)
   ```

3. Use in your C++ code:
   ```cpp
   #include "rex_interfaces/msg/rover_control.hpp"
   ```

   Or in Python:
   ```python
   from rex_interfaces.msg import RoverControl
   ```

## License
This package is available under the BSD license.

## Maintainer
- Bitnut01 (wmkuchar@gmail.com)