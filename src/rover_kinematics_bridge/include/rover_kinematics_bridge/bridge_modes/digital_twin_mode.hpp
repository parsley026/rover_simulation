#pragma once

#include "rover_kinematics_bridge/bridge_modes/bridge_mode_base.hpp"
#include "rover_kinematics_bridge/gazebo/gazebo_command_publisher.hpp"
#include "rover_kinematics_bridge/core/conversions.hpp"
#include "rover_kinematics_bridge/core/wheel_data.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rex_interfaces/msg/wheels.hpp>
#include <rex_interfaces/msg/vesc_status.hpp>
#include <memory>
#include <mutex>
#include <atomic>

namespace rover_kinematics_bridge {
namespace bridge_modes {

class DigitalTwinMode : public IBridgeMode {
public:
    DigitalTwinMode(
        rclcpp::Node* node,
        std::shared_ptr<gazebo::GazeboCommandPublisher> gazebo_pub,
        std::shared_ptr<core::Conversions> conversions,
        double hardware_timeout_sec,
        const BridgeConfig& config);

    void update() override;

private:
    void kinematicsCallback(const rex_interfaces::msg::Wheels::SharedPtr msg);
    void hardwareFeedbackCallback(const rex_interfaces::msg::VescStatus::SharedPtr msg);

    rclcpp::Node* node_;
    std::shared_ptr<gazebo::GazeboCommandPublisher> gazebo_pub_;
    std::shared_ptr<core::Conversions> conversions_;
    double hardware_timeout_sec_;
    BridgeConfig config_;

    rclcpp::Subscription<rex_interfaces::msg::Wheels>::SharedPtr kinematics_sub_;
    rclcpp::Subscription<rex_interfaces::msg::VescStatus>::SharedPtr hardware_sub_;

    std::atomic<int64_t> last_hardware_time_ns_{0};

    std::mutex data_mutex_;
    core::WheelData predicted_state_current_;
    core::WheelData predicted_state_buffor_;
    core::WheelData measured_state_current_;
    core::WheelData measured_state_buffor_;
};

} // namespace bridge_modes
} // namespace rover_kinematics_bridge
