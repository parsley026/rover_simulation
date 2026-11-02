#pragma once

#include "rover_robot_arm_kinematics_bridge/bridge_modes/bridge_mode_base.hpp"
#include "rover_robot_arm_kinematics_bridge/gazebo/gazebo_arm_publisher.hpp"
#include "rover_robot_arm_kinematics_bridge/core/conversions.hpp"

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>

namespace rover_robot_arm_kinematics_bridge {

class rover_robot_arm_kinematics_bridge : public rclcpp::Node {
public:
    explicit rover_robot_arm_kinematics_bridge(const rclcpp::NodeOptions& options);
    ~rover_robot_arm_kinematics_bridge() = default;

private:
    void loadParameters();
    void onUpdate();

    std::string mode_str_{"simulation"};
    int    pole_numbers_{7};
    double default_gear_ratio_{1000.0};
    double publish_rate_hz_{20.0};
    double hardware_timeout_sec_{0.5};
    bridge_modes::ArmBridgeConfig config_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Duration publish_period_{rclcpp::Duration::from_seconds(0.05)};

    std::shared_ptr<gazebo::GazeboArmPublisher> gazebo_pub_;
    std::shared_ptr<core::Conversions> conversions_;

    std::unique_ptr<bridge_modes::IBridgeMode> active_mode_;
};

} // namespace rover_robot_arm_kinematics_bridge
