#pragma once

#include "std_msgs/msg/float64.hpp"

namespace rover_kinematics_bridge {
namespace core {

// -----------------------------------------------------------------------
// Data buffers
// -----------------------------------------------------------------------
struct WheelData {
    std_msgs::msg::Float64 front_left_drive{};
    std_msgs::msg::Float64 front_right_drive{};
    std_msgs::msg::Float64 rear_right_drive{};
    std_msgs::msg::Float64 rear_left_drive{};

    std_msgs::msg::Float64 front_left_steer{};
    std_msgs::msg::Float64 front_right_steer{};
    std_msgs::msg::Float64 rear_right_steer{};
    std_msgs::msg::Float64 rear_left_steer{};
};

} // namespace core
} // namespace rover_kinematics_bridge
