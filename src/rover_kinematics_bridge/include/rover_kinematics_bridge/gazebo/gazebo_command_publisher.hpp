#pragma once

#include "rover_kinematics_bridge/core/wheel_data.hpp"
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

namespace rover_kinematics_bridge {
namespace gazebo {

class GazeboCommandPublisher {
public:
  explicit GazeboCommandPublisher(rclcpp::Node *node);

  /// Publishes a WheelData snapshot to all 8 Gazebo joint controllers.
  void publish(const core::WheelData &state);

private:
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr front_left_drive_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr front_right_drive_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rear_right_drive_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rear_left_drive_pub_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr front_left_steer_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr front_right_steer_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rear_right_steer_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rear_left_steer_pub_;
};

} // namespace gazebo
} // namespace rover_kinematics_bridge
