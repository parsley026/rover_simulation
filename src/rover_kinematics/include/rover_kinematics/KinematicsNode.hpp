#pragma once

#include <atomic>
#include <array>
#include <cstddef>
#include <cstdint>
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
#include "rover_kinematics/KinematicsConfig.hpp"
#include "rover_kinematics/KinematicsSolver.hpp"
#include "rover_kinematics/KinematicsEstimator.hpp"
#include "rover_kinematics/HardwareInterface.hpp"
#include <rosgraph_msgs/msg/clock.hpp>

constexpr int COMMUNICATION_STATE_CREATED = 0;
// constexpr int COMMUNICATION_STATE_OPENING = 1;
constexpr int COMMUNICATION_STATE_OPENED  = 2;
// constexpr int COMMUNICATION_STATE_CLOSING = 3;
// constexpr int COMMUNICATION_STATE_CLOSED  = 4;
// constexpr int COMMUNICATION_STATE_FAULTED = 5;

// constexpr int CONTROL_MODE_NONE   = 0;
// constexpr int CONTROL_MODE_ESTOP  = 1;
// constexpr int CONTROL_MODE_STOP   = 2;
// constexpr int CONTROL_MODE_CONFIG = 4;

constexpr int CONTROL_MODE_DRIVE           = 1;
constexpr int CONTROL_MODE_ROBOTIC_ARM     = 2;
constexpr int CONTROL_MODE_DEEP_SAMPLER    = 3;
constexpr int CONTROL_MODE_SURFACE_SAMPLER = 3;
constexpr int CONTROL_MODE_DRIVE_AUTONOMY  = 4;

// constexpr int CONTROL_MODE_ROBOTIC_ARM_AUTONOMY     = 256;
// constexpr int CONTROL_MODE_DEEP_SAMPLER_AUTONOMY    = 512;
// constexpr int CONTROL_MODE_SURFACE_SAMPLER_AUTONOMY = 1024;

constexpr int SYMMETRIC_ACKERMANN_MODE = 1;
constexpr int CRAB_MODE =    2;
constexpr int SYMMETRIC_SPIN_MODE = 3;

#define DUTY_CONTROL    0
#define CURRENT_CONTROL 1
#define RPM_CONTROL     3
#define SET_ORIGIN      5


#define STEP_MOTOR_VELOCITY_CONTROL 3
#define STEP_MOTOR_CONTROL 4


class KinematicsNode : public rclcpp::Node {
public:

    /**
     * @brief
     * @param options
     * @param config_file_path
     * @return
     */
    KinematicsNode(const rclcpp::NodeOptions& options, const std::string& config_file_path);

    
    /**
     * @brief
     * @return
     */
    void onUpdate();

    rclcpp::TimerBase::SharedPtr timer_;

    std::string config_file_path_;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_kinematics_srv_;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_odometry_srv_;

private:

    /**
     * @brief
     * @param msg
     * @return
     */
    void updateWatchdog(const rclcpp::Time& time_curr);

    /**
     * @brief
     * @param msg
     * @return
     */
    void clockCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg);

    /**
     * @brief
     * @param msg
     * @return
     */
    void statusCallback(const rex_interfaces::msg::RoverStatus::SharedPtr msg);

    /**
     * @brief
     * @param msg
     * @return
     */
    void cmdVelManualCallback(const rex_interfaces::msg::RoverControl::SharedPtr msg);

    /**
     * @brief
     * @param msg
     * @return
     */
    void cmdVelAutoCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    /**
     * @brief
     * @param msg
     * @return
     */
    void feedbackCallback(const rex_interfaces::msg::VescStatus::SharedPtr msg);

    double computeWheelQualityScore(const rex_interfaces::msg::VescStatus& status) const;
    void updateWheelQuality(std::size_t wheel_index, const rex_interfaces::msg::VescStatus& status);

    /**
     * @brief
     * @return
     */
    void initCmdVelBuffer();

    /**
     * @brief
     * @return
     */
    void initFeedbackBuffer();
    
    /**
     * @brief
     * @param request
     * @param response
     * @return
     */
    void resetOdometryCallback(
        const std::shared_ptr<std_srvs::srv::Empty::Request>  request,
              std::shared_ptr<std_srvs::srv::Empty::Response> response);
    
    /**
     * @brief
     * @param request
     * @param response
     * @return
     */
    void resetKinematicsCallback(
        const std::shared_ptr<std_srvs::srv::Empty::Request>  request,
              std::shared_ptr<std_srvs::srv::Empty::Response> response);
              
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;

    rclcpp::Subscription<rex_interfaces::msg::RoverStatus>::SharedPtr status_sub_;

    rclcpp::Subscription<rex_interfaces::msg::RoverControl>::SharedPtr cmd_vel_sub_manual_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_auto_;

    rclcpp::Subscription<rex_interfaces::msg::VescStatus>::SharedPtr wheels_vel_feedback_sub_;

    rclcpp::Publisher<rex_interfaces::msg::Wheels>::SharedPtr wheels_vel_pub_;

    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    // Time
    std::atomic<int64_t> sim_time_ns_{0};
    rclcpp::Duration publish_period_;

    rex_interfaces::msg::Wheels rover_wheels_velocity_;

    realtime_tools::RealtimeBuffer<rex_interfaces::msg::RoverControl> rover_cmd_velocity_buffer_;
    realtime_tools::RealtimeBuffer<rex_interfaces::msg::Wheels> rover_wheels_velocity_feedback_buffer_;

    KinematicsConfig config_;
    KinematicsSolver kinematics_solver_;
    KinematicsEstimator kinematics_estimator;
    HardwareInterface hardware_interface_;

    std::mutex feedback_mutex_;

    std::array<double, 4> wheel_quality_weights_{{1.0, 1.0, 1.0, 1.0}};
    std::array<double, 4> drive_quality_weights_{{1.0, 1.0, 1.0, 1.0}};
    std::array<double, 4> steer_quality_weights_{{1.0, 1.0, 1.0, 1.0}};

    std::atomic<int> control_mode_{CONTROL_MODE_DRIVE};
    std::atomic<int> communication_state_{COMMUNICATION_STATE_CREATED};

    std::atomic<int64_t> last_feedback_time_ns_{0};
    std::atomic<bool> prev_feedback_stale_{false};
    std::atomic<bool>      feedback_stale_{false};

    bool initialized_ = false;

    /**
     * @brief
     * @param command
     * @param time
     * @return
     */
    rex_interfaces::msg::Wheels assembleWheelsFromCommand(const WheelCommand& command, const rclcpp::Time& time);
};
