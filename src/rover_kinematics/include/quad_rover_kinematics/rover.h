#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <yaml-cpp/yaml.h>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <std_srvs/srv/empty.hpp> 
#include "rex_interfaces/msg/wheels.hpp"
#include "rex_interfaces/msg/rover_control.hpp"
#include "rex_interfaces/msg/vesc_status.hpp"
#include "rex_interfaces/msg/rover_status.hpp"
#include "quad_rover_kinematics/kinematics.h"
#include <rosgraph_msgs/msg/clock.hpp>

// Connection and control modes
constexpr int CONNECTION_CONNECTED =    2;
constexpr int CONNECTION_DISCONNECTED = 0;

constexpr int ESTOP_CONTROL = 0;
constexpr int USER_CONTROL = 1;
constexpr int MANIPULATOR_CONTROL = 2;
constexpr int PROBE_CONTROL = 3;
constexpr int AUTONOMY_CONTROL = 4;

// Rover driving modes
constexpr int ADVANCE_MODE = 1;
constexpr int CRAB_MODE =    2;
constexpr int SPIN_MODE =    3;

class Rover : public rclcpp::Node {
public:
    Rover(const rclcpp::NodeOptions& options, const std::string& config_file_path);

    void onUpdate();

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_odom_srv_;

private:
    void readConfig(const std::string& config_file_path); // Read YAML file with rover configuration
    void setCmdVelocity(double x_vel, double y_vel, double speed);

    // Callback functions
    void cmdVelCallback(const rex_interfaces::msg::RoverControl::SharedPtr msg);
    void feedbackCallback(const rex_interfaces::msg::VescStatus::SharedPtr msg);
    void statusCallback(const rex_interfaces::msg::RoverStatus::SharedPtr msg);
    void cmdVelAutoCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void clockCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg);

    // Service callback
    void resetOdometryCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                               std::shared_ptr<std_srvs::srv::Empty::Response> response);

    

    // ROS2 Communication Members
    rclcpp::Subscription<rex_interfaces::msg::RoverControl>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<rex_interfaces::msg::VescStatus>::SharedPtr wheels_vel_feedback_sub_;
    rclcpp::Subscription<rex_interfaces::msg::RoverStatus>::SharedPtr status_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_auto_;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscription_;

    rclcpp::Publisher<rex_interfaces::msg::Wheels>::SharedPtr wheels_vel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_odom_pub_;

    // Time management
    rclcpp::Time sim_time_;
    rclcpp::Duration publish_period_;

    // Buffers and state management
    rex_interfaces::msg::RoverControl rover_cmd_velocity_;         // Workspace for callback
    rex_interfaces::msg::Wheels rover_wheels_velocity_;            // Workspace for update
    rex_interfaces::msg::Wheels rover_wheels_velocity_feedback_;   // Workspace for callback

    
    // Realtime buffers
    realtime_tools::RealtimeBuffer<rex_interfaces::msg::RoverControl> rover_cmd_velocity_buffer_;
    realtime_tools::RealtimeBuffer<rex_interfaces::msg::Wheels> rover_wheels_velocity_feedback_buffer_;

    // Kinematics object
    Kinematics kinematic_;

    // Configuration parameters
    double L_; // Length between front and back wheels
    double W_; // Width between left and right wheels
    double wheel_diameter_;
    int poles_pairs_number_;
    double motor_gear_ratio_;
    double min_erpm_; // Minimum ERPM for the motors
    double max_steering_radius_;
    double min_steering_radius_;
    std::vector<double> pose_covariance_diagonal_;
    std::vector<double> twist_covariance_diagonal_;
    bool use_sim_time_;
    double publish_rate_;
    std::string base_frame_id_;
    std::string odom_frame_id_;
    bool enable_odom_tf_;
    int control_mode_;
    int communication_state_;

    bool initialized_ = false; // Flag to check if the rover is initialized

};
