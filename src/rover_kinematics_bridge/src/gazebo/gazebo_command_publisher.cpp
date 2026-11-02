#include "rover_kinematics_bridge/gazebo/gazebo_command_publisher.hpp"

namespace rover_kinematics_bridge {
namespace gazebo {

GazeboCommandPublisher::GazeboCommandPublisher(rclcpp::Node* node)
{
    front_left_drive_pub_  = node->create_publisher<std_msgs::msg::Float64MultiArray>("/drive_front_left_controller/commands",  10);
    front_right_drive_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("/drive_front_right_controller/commands", 10);
    rear_right_drive_pub_  = node->create_publisher<std_msgs::msg::Float64MultiArray>("/drive_rear_right_controller/commands",  10);
    rear_left_drive_pub_   = node->create_publisher<std_msgs::msg::Float64MultiArray>("/drive_rear_left_controller/commands",   10);

    front_left_steer_pub_  = node->create_publisher<std_msgs::msg::Float64MultiArray>("/steer_front_left_controller/commands",  10);
    front_right_steer_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("/steer_front_right_controller/commands", 10);
    rear_right_steer_pub_  = node->create_publisher<std_msgs::msg::Float64MultiArray>("/steer_rear_right_controller/commands",  10);
    rear_left_steer_pub_   = node->create_publisher<std_msgs::msg::Float64MultiArray>("/steer_rear_left_controller/commands",   10);
}

void GazeboCommandPublisher::publish(const core::WheelData& state)
{
    auto make_msg = [](double val) {
        std_msgs::msg::Float64MultiArray msg;
        msg.data.push_back(val);
        return msg;
    };

    front_left_drive_pub_->publish(make_msg(state.front_left_drive.data));
    front_right_drive_pub_->publish(make_msg(state.front_right_drive.data));
    rear_right_drive_pub_->publish(make_msg(state.rear_right_drive.data));
    rear_left_drive_pub_->publish(make_msg(state.rear_left_drive.data));

    front_left_steer_pub_->publish(make_msg(state.front_left_steer.data));
    front_right_steer_pub_->publish(make_msg(state.front_right_steer.data));
    rear_right_steer_pub_->publish(make_msg(state.rear_right_steer.data));
    rear_left_steer_pub_->publish(make_msg(state.rear_left_steer.data));
}

} // namespace gazebo
} // namespace rover_kinematics_bridge
