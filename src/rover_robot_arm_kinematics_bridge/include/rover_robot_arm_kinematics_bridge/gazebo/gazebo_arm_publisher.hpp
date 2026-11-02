#pragma once

#include "rover_robot_arm_kinematics_bridge/core/arm_data.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <memory>

namespace rover_robot_arm_kinematics_bridge {
namespace gazebo {

class GazeboArmPublisher {
public:
    using GripperCommand = control_msgs::action::GripperCommand;
    using GoalHandleGripper = rclcpp_action::ClientGoalHandle<GripperCommand>;

    explicit GazeboArmPublisher(rclcpp::Node* node);

    /// Publishes arm joint trajectory and gripper commands to Gazebo ros2_control controllers
    void publish(const core::ArmData& state);

    /// Explicitly send a gripper goal (position in meters, max_effort in N)
    void sendGripperCommand(double position, double max_effort = 100.0);

private:
    rclcpp::Node* node_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_traj_pub_;
    rclcpp_action::Client<GripperCommand>::SharedPtr gripper_action_client_;

    double last_sent_gripper_pos_{-999.0};
};

} // namespace gazebo
} // namespace rover_robot_arm_kinematics_bridge
