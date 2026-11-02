#include "rover_robot_arm_kinematics_bridge/gazebo/gazebo_arm_publisher.hpp"
#include <cmath>

namespace rover_robot_arm_kinematics_bridge {
namespace gazebo {

GazeboArmPublisher::GazeboArmPublisher(rclcpp::Node* node)
    : node_(node)
{
    arm_traj_pub_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/arm_controller/joint_trajectory", 10);

    gripper_action_client_ = rclcpp_action::create_client<GripperCommand>(
        node_, "/gripper_controller/gripper_cmd");
}

void GazeboArmPublisher::publish(const core::ArmData& state)
{
    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.stamp = node_->now();
    traj.joint_names = core::ARM_JOINT_NAMES;

    trajectory_msgs::msg::JointTrajectoryPoint pt;
    for (size_t i = 0; i < core::NUM_ARM_JOINTS; ++i) {
        pt.positions.push_back(state.joints[i].position);
        pt.velocities.push_back(state.joints[i].velocity);
    }
    // Set duration to a small step in the future (matching bridge publish period)
    pt.time_from_start = rclcpp::Duration::from_seconds(0.05);
    traj.points.push_back(pt);

    arm_traj_pub_->publish(traj);

    sendGripperCommand(state.gripper1.position);
}

void GazeboArmPublisher::sendGripperCommand(double position, double max_effort)
{
    if (std::abs(position - last_sent_gripper_pos_) > 0.0005) {
        if (gripper_action_client_->action_server_is_ready()) {
            auto goal = GripperCommand::Goal();
            goal.command.position = position;
            goal.command.max_effort = max_effort;
            gripper_action_client_->async_send_goal(goal);
            last_sent_gripper_pos_ = position;
        }
    }
}

} // namespace gazebo
} // namespace rover_robot_arm_kinematics_bridge
