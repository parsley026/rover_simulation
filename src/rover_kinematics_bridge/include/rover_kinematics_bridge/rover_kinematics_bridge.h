#pragma once

#include "rover_kinematics_bridge/bridge_modes/bridge_mode_base.hpp"
#include "rover_kinematics_bridge/gazebo/gazebo_command_publisher.hpp"
#include "rover_kinematics_bridge/core/conversions.hpp"

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>

namespace rover_kinematics_bridge {

class rover_kinematics_bridge : public rclcpp::Node {
public:
    explicit rover_kinematics_bridge(const rclcpp::NodeOptions& options);
    ~rover_kinematics_bridge() = default;

private:
    void loadParameters();
    void onUpdate();

    std::string mode_str_;
    int    pole_numbers_{7};
    double motor_gear_ratio_{31.0};
    double publish_rate_hz_{20.0};
    double hardware_timeout_sec_{0.5};
    bridge_modes::BridgeConfig config_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Duration publish_period_{rclcpp::Duration::from_seconds(0.05)};

    std::shared_ptr<gazebo::GazeboCommandPublisher> gazebo_pub_;
    std::shared_ptr<core::Conversions> conversions_;

    std::unique_ptr<bridge_modes::IBridgeMode> active_mode_;
};

} // namespace rover_kinematics_bridge
