#pragma once

#include "rover_robot_arm_kinematics_bridge/bridge_modes/bridge_mode_base.hpp"
#include "rover_robot_arm_kinematics_bridge/core/conversions.hpp"
#include "rover_robot_arm_kinematics_bridge/core/arm_data.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rex_interfaces/msg/vesc_status.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <memory>
#include <mutex>

namespace rover_robot_arm_kinematics_bridge {
namespace bridge_modes {

class DigitalShadowMode : public IBridgeMode {
public:
    DigitalShadowMode(
        rclcpp::Node* node,
        std::shared_ptr<core::Conversions> conversions,
        const ArmBridgeConfig& config);

    void update() override;

private:
    void hardwareFeedbackCallback(const rex_interfaces::msg::VescStatus::SharedPtr msg);

    rclcpp::Node* node_;
    std::shared_ptr<core::Conversions> conversions_;
    ArmBridgeConfig config_;

    rclcpp::Subscription<rex_interfaces::msg::VescStatus>::SharedPtr hardware_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

    std::mutex data_mutex_;
    core::ArmData measured_state_current_;
    core::ArmData measured_state_buffor_;
};

} // namespace bridge_modes
} // namespace rover_robot_arm_kinematics_bridge
