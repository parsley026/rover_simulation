#pragma once

#include "rover_robot_arm_kinematics_bridge/bridge_modes/bridge_mode_base.hpp"
#include "rover_robot_arm_kinematics_bridge/gazebo/gazebo_arm_publisher.hpp"
#include "rover_robot_arm_kinematics_bridge/core/conversions.hpp"
#include "rover_robot_arm_kinematics_bridge/core/arm_data.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <rex_interfaces/msg/manipulator_control.hpp>
#include <rex_interfaces/msg/vesc_status.hpp>
#include <memory>
#include <mutex>

namespace rover_robot_arm_kinematics_bridge {
namespace bridge_modes {

class SimulationMode : public IBridgeMode {
public:
    SimulationMode(
        rclcpp::Node* node,
        std::shared_ptr<gazebo::GazeboArmPublisher> gazebo_pub,
        std::shared_ptr<core::Conversions> conversions,
        const ArmBridgeConfig& config);

    void update() override;

private:
    void manipulatorControlCallback(const rex_interfaces::msg::ManipulatorControl::SharedPtr msg);
    void armTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
    void feedbackCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void publishSimFeedback();

    rclcpp::Node* node_;
    std::shared_ptr<gazebo::GazeboArmPublisher> gazebo_pub_;
    std::shared_ptr<core::Conversions> conversions_;
    ArmBridgeConfig config_;

    rclcpp::Subscription<rex_interfaces::msg::ManipulatorControl>::SharedPtr manipulator_control_sub_;
    rclcpp::Subscription<rex_interfaces::msg::ManipulatorControl>::SharedPtr manipulator_ctl_sub_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_trajectory_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr feedback_sub_;
    rclcpp::Publisher<rex_interfaces::msg::VescStatus>::SharedPtr vesc_feedback_pub_;

    std::mutex data_mutex_;
    core::ArmData predicted_state_current_;
    core::ArmData predicted_state_buffor_;
    core::ArmData simulated_feedback_current_;
    core::ArmData simulated_feedback_buffor_;
};

} // namespace bridge_modes
} // namespace rover_robot_arm_kinematics_bridge
