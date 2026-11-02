#pragma once

#include "rover_robot_arm_kinematics_bridge/core/arm_constants.hpp"
#include <array>

namespace rover_robot_arm_kinematics_bridge {
namespace bridge_modes {

struct ArmBridgeConfig {
    std::array<bool, core::NUM_ARM_JOINTS> invert_joints{false, false, false, false, false, false};
    bool invert_gripper{false};

    std::array<double, core::NUM_ARM_JOINTS> gear_ratios{1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0};
    double gripper_gear_ratio{1000.0};
    double gripper_linear_scale{0.0001}; // meters per motor degree

    int pole_numbers{7};
    double publish_rate{20.0};
    double hardware_timeout_sec{0.5};
};

class IBridgeMode {
public:
    virtual ~IBridgeMode() = default;

    /// Called periodically by the node's main timer.
    virtual void update() = 0;
};

} // namespace bridge_modes
} // namespace rover_robot_arm_kinematics_bridge
