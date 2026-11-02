#pragma once

#include <string>
#include <vector>
#include <array>

namespace rover_robot_arm_kinematics_bridge {
namespace core {

// ---------------------------------------------------------------------------
// VESC CAN bus IDs — robotic arm joints & gripper
// ---------------------------------------------------------------------------
constexpr int VESC_ID_LINK1    = 112; // 0x70
constexpr int VESC_ID_LINK2    = 113; // 0x71
constexpr int VESC_ID_LINK3    = 114; // 0x72
constexpr int VESC_ID_LINK4    = 115; // 0x73
constexpr int VESC_ID_LINK5    = 116; // 0x74
constexpr int VESC_ID_LINK6    = 117; // 0x75
constexpr int VESC_ID_GRIPPER1 = 118; // 0x76

constexpr size_t NUM_ARM_JOINTS = 6;

// URDF joint names
inline const std::vector<std::string> ARM_JOINT_NAMES = {
    "link1", "link2", "link3", "link4", "link5", "link6"
};

inline const std::string GRIPPER1_JOINT_NAME = "gripper1";
inline const std::string GRIPPER2_JOINT_NAME = "gripper2";

// Default / rest joint positions (radians / meters) matching URDF / ros2_control defaults
constexpr double DEFAULT_LINK1_POS = 0.0;
constexpr double DEFAULT_LINK2_POS = -0.17453292519943295; // -10 deg
constexpr double DEFAULT_LINK3_POS = -0.45378560551852565; // -26 deg
constexpr double DEFAULT_LINK4_POS = 0.0;
constexpr double DEFAULT_LINK5_POS = 0.6108652381980153;  //  35 deg
constexpr double DEFAULT_LINK6_POS = 0.0;
constexpr double DEFAULT_GRIPPER1_POS = 0.001;
constexpr double DEFAULT_GRIPPER2_POS = -0.001;

} // namespace core
} // namespace rover_robot_arm_kinematics_bridge
