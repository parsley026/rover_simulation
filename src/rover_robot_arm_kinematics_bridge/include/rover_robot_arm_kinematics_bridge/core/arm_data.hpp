#pragma once

#include "rover_robot_arm_kinematics_bridge/core/arm_constants.hpp"
#include <array>

namespace rover_robot_arm_kinematics_bridge {
namespace core {

struct JointData {
    double position{0.0};
    double velocity{0.0};
    double effort{0.0};
};

struct ArmData {
    std::array<JointData, NUM_ARM_JOINTS> joints{
        JointData{DEFAULT_LINK1_POS, 0.0, 0.0},
        JointData{DEFAULT_LINK2_POS, 0.0, 0.0},
        JointData{DEFAULT_LINK3_POS, 0.0, 0.0},
        JointData{DEFAULT_LINK4_POS, 0.0, 0.0},
        JointData{DEFAULT_LINK5_POS, 0.0, 0.0},
        JointData{DEFAULT_LINK6_POS, 0.0, 0.0}
    };

    JointData gripper1{DEFAULT_GRIPPER1_POS, 0.0, 0.0};
    JointData gripper2{DEFAULT_GRIPPER2_POS, 0.0, 0.0};
};

} // namespace core
} // namespace rover_robot_arm_kinematics_bridge
