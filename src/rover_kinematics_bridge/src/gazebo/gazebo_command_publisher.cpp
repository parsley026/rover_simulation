#include "rover_kinematics_bridge/gazebo/gazebo_command_publisher.hpp"

namespace rover_kinematics_bridge {
namespace gazebo {

GazeboCommandPublisher::GazeboCommandPublisher(rclcpp::Node* node)
{
    front_left_drive_pub_  = node->create_publisher<std_msgs::msg::Float64>("/wheel/front_left/cmd_vel",  10);
    front_right_drive_pub_ = node->create_publisher<std_msgs::msg::Float64>("/wheel/front_right/cmd_vel", 10);
    rear_right_drive_pub_  = node->create_publisher<std_msgs::msg::Float64>("/wheel/rear_right/cmd_vel",  10);
    rear_left_drive_pub_   = node->create_publisher<std_msgs::msg::Float64>("/wheel/rear_left/cmd_vel",   10);

    front_left_steer_pub_  = node->create_publisher<std_msgs::msg::Float64>("/steer/front_left/cmd_pos",  10);
    front_right_steer_pub_ = node->create_publisher<std_msgs::msg::Float64>("/steer/front_right/cmd_pos", 10);
    rear_right_steer_pub_  = node->create_publisher<std_msgs::msg::Float64>("/steer/rear_right/cmd_pos",  10);
    rear_left_steer_pub_   = node->create_publisher<std_msgs::msg::Float64>("/steer/rear_left/cmd_pos",   10);
}

void GazeboCommandPublisher::publish(const core::WheelData& state)
{
    front_left_drive_pub_->publish(state.front_left_drive);
    front_right_drive_pub_->publish(state.front_right_drive);
    rear_right_drive_pub_->publish(state.rear_right_drive);
    rear_left_drive_pub_->publish(state.rear_left_drive);

    front_left_steer_pub_->publish(state.front_left_steer);
    front_right_steer_pub_->publish(state.front_right_steer);
    rear_right_steer_pub_->publish(state.rear_right_steer);
    rear_left_steer_pub_->publish(state.rear_left_steer);
}

} // namespace gazebo
} // namespace rover_kinematics_bridge
