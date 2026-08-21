#pragma once

#include "rover_kinematics_bridge/bridge_modes/bridge_mode_base.hpp"
#include "rover_kinematics_bridge/gazebo/gazebo_command_publisher.hpp"
#include "rover_kinematics_bridge/core/conversions.hpp"
#include "rover_kinematics_bridge/core/wheel_data.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rex_interfaces/msg/wheels.hpp>
#include <rex_interfaces/msg/vesc_status.hpp>
#include <memory>
#include <mutex>

namespace rover_kinematics_bridge {
namespace bridge_modes {

class SimulationMode : public IBridgeMode {
public:
    SimulationMode(
        rclcpp::Node* node,
        std::shared_ptr<gazebo::GazeboCommandPublisher> gazebo_pub,
        std::shared_ptr<core::Conversions> conversions,
        const BridgeConfig& config);

    void update() override;

private:
    void kinematicsCallback(const rex_interfaces::msg::Wheels::SharedPtr msg);
    void feedbackCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void publishSimFeedback();

    rclcpp::Node* node_;
    std::shared_ptr<gazebo::GazeboCommandPublisher> gazebo_pub_;
    std::shared_ptr<core::Conversions> conversions_;
    BridgeConfig config_;

    rclcpp::Subscription<rex_interfaces::msg::Wheels>::SharedPtr kinematics_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr feedback_sub_;
    rclcpp::Publisher<rex_interfaces::msg::VescStatus>::SharedPtr vesc_feedback_pub_;

    std::mutex data_mutex_;
    core::WheelData predicted_state_current_;
    core::WheelData predicted_state_buffor_;
    core::WheelData simulated_feedback_current_;
    core::WheelData simulated_feedback_buffor_;
};

} // namespace bridge_modes
} // namespace rover_kinematics_bridge
