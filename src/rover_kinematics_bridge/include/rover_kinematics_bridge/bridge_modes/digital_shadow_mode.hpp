#pragma once

#include "rover_kinematics_bridge/bridge_modes/bridge_mode_base.hpp"
#include "rover_kinematics_bridge/gazebo/gazebo_command_publisher.hpp"
#include "rover_kinematics_bridge/core/conversions.hpp"
#include "rover_kinematics_bridge/core/wheel_data.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rex_interfaces/msg/vesc_status.hpp>
#include <memory>
#include <mutex>

namespace rover_kinematics_bridge {
namespace bridge_modes {

class DigitalShadowMode : public IBridgeMode {
public:
    DigitalShadowMode(
        rclcpp::Node* node,
        std::shared_ptr<gazebo::GazeboCommandPublisher> gazebo_pub,
        std::shared_ptr<core::Conversions> conversions,
        const BridgeConfig& config);

    void update() override;

private:
    void hardwareFeedbackCallback(const rex_interfaces::msg::VescStatus::SharedPtr msg);

    rclcpp::Node* node_;
    std::shared_ptr<gazebo::GazeboCommandPublisher> gazebo_pub_;
    std::shared_ptr<core::Conversions> conversions_;
    BridgeConfig config_;

    rclcpp::Subscription<rex_interfaces::msg::VescStatus>::SharedPtr hardware_sub_;

    std::mutex data_mutex_;
    core::WheelData measured_state_current_;
    core::WheelData measured_state_buffor_;
};

} // namespace bridge_modes
} // namespace rover_kinematics_bridge
