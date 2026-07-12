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
#include "quad_rover_kinematics/RoverConfig.hpp"
#include "quad_rover_kinematics/InverseKinematics.hpp"
#include "quad_rover_kinematics/ForwardKinematics.hpp"
#include "quad_rover_kinematics/HardwareInterface.hpp"
#include <rosgraph_msgs/msg/clock.hpp>

/**
 * @defgroup ConnectionStates Connection and Control Mode constants
 * @brief Integer constants used to describe connection and control modes.
 */

/** Connected to hardware/feedback (non-zero positive). */
constexpr int CONNECTION_CONNECTED =    2;
/** Disconnected from hardware/feedback. */
constexpr int CONNECTION_DISCONNECTED = 0;

/** Emergency stop control mode. */
constexpr int ESTOP_CONTROL = 0;
/** User/operator direct control mode. */
constexpr int USER_CONTROL = 1;
/** Manipulator control mode (unused in core kinematics). */
constexpr int MANIPULATOR_CONTROL = 2;
/** Probe mode (reserved). */
constexpr int PROBE_CONTROL = 3;
/** Autonomy control mode (commands from planner/autonomy stack). */
constexpr int AUTONOMY_CONTROL = 4;

/** Advance driving mode (forward/backward with steering). */
constexpr int ADVANCE_MODE = 1;
/** Crab driving mode (lateral translation). */
constexpr int CRAB_MODE =    2;
/** Spin driving mode (on-the-spot rotation). */
constexpr int SPIN_MODE =    3;

// Wheel control mode constants mirror hardware protocol values
/** Duty control (hardware). */
#define DUTY_CONTROL    0
/** Current control (hardware). */
#define CURRENT_CONTROL 1
/** RPM control (hardware). */
#define RPM_CONTROL     3
/** Set origin command id (hardware). */
#define SET_ORIGIN      5

// Steering control mode constants mirror hardware protocol values
/** Stepper motor velocity control. */
#define STEP_MOTOR_VELOCITY_CONTROL 3
/** Stepper motor position control. */
#define STEP_MOTOR_CONTROL 4

/**
 * @class RoverNode
 * @brief ROS 2 node that orchestrates kinematics, hardware interface and state.
 *
 * @details
 * RoverNode is the entry point for sensor feedback, operator/autonomy
 * commands and the kinematics stack. It owns the `RoverConfig` (single
 * source of configuration), the `InverseKinematics` and `ForwardKinematics`
 * components, and the `HardwareInterface` used for unit conversions.
 *
 * Responsibilities:
 * - Subscribe to command and feedback topics and translate them into
 *   wheel-level commands via `InverseKinematics` and `HardwareInterface`.
 * - Run forward kinematics to provide `nav_msgs::msg::Odometry` and optional
 *   TF transforms using `ForwardKinematics`.
 * - Detect stale hardware feedback and publish a safe brake command.
 *
 * Thread-safety: `RoverNode` uses realtime buffers for cross-thread data
 * passing between non-RT callbacks and the RT timer update loop. The node
 * schedules `onUpdate()` on a ROS timer and uses `RealtimeBuffer` to avoid
 * locking in the fast path. Initialization (config load) must complete before
 * the timer starts to ensure consistent component state.
 *
 * Ownership: the node owns its components as member objects (value semantics)
 * and passes `RoverConfig` by const reference into them during initialization.
 */
class RoverNode : public rclcpp::Node {
public:
    /**
     * @brief Construct the RoverNode.
     *
     * Loads configuration from `config_file_path`, initializes subcomponents
     * and starts a periodic timer to publish commands and odometry.
     *
     * @param options Node options forwarded to rclcpp::Node
     * @param config_file_path Path to YAML configuration file in package share.
     */
    RoverNode(const rclcpp::NodeOptions& options, const std::string& config_file_path);

    /**
     * @brief Periodic update called by the ROS timer.
     *
     * This function runs at the configured `publish_rate` and performs:
     * - forward kinematics update from latest wheel feedback
     * - assembly and publication of wheel setpoint messages
     * - stale-feedback detection and safe braking
     */
    void onUpdate();

    /// Timer used to schedule `onUpdate()` at `publish_rate`.
    rclcpp::TimerBase::SharedPtr timer_;
    /// Service to reset odometry counters in the forward estimator.
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_odom_srv_;

private:
    /**
     * @brief Legacy YAML reader used as a fallback when `loadFromFile` fails.
     *
     * This reads the YAML and applies values to `config_` without locking
     * `initialized_`. Prefer `RoverConfig::loadFromFile()` for a validated
     * immutable configuration.
     *
     * @param config_file_path Path to YAML config file.
     */
    void readConfig(const std::string& config_file_path);

    /**
     * @brief Convenience to construct an internal `RoverControl` message.
     *
     * @param x_vel Lateral axis command (unit depends on mode)
     * @param y_vel Secondary axis command (unit depends on mode)
     * @param speed Primary speed / magnitude (m/s or rad/s depending on mode)
     */
    void setCmdVelocity(double x_vel, double y_vel, double speed);

    // Callbacks
    /**
     * @brief MQTT/operator command callback.
     *
     * Accepts `rex_interfaces::msg::RoverControl` messages and stores them in
     * the realtime buffer for use in `onUpdate()`.
     */
    void cmdVelCallback(const rex_interfaces::msg::RoverControl::SharedPtr msg);

    /**
     * @brief Wheel/VESC feedback callback.
     *
     * Translates `VescStatus` messages into internal wheel velocity and
     * steering feedback values via `HardwareInterface` conversions and
     * updates the last feedback timestamp used for stale detection.
     */
    void feedbackCallback(const rex_interfaces::msg::VescStatus::SharedPtr msg);

    /**
     * @brief Rover status callback (control/communication state updates).
     */
    void statusCallback(const rex_interfaces::msg::RoverStatus::SharedPtr msg);

    /**
     * @brief Twist callback for autonomy commands (`/cmd_vel`).
     *
     * Converts a geometry_msgs::msg::Twist into the internal `RoverControl`
     * shape when the node is in `AUTONOMY_CONTROL` mode.
     */
    void cmdVelAutoCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    /**
     * @brief Simulated clock callback. Updates `sim_time_` when `use_sim_time`.
     */
    void clockCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg);

    /**
     * @brief Service callback to reset odometry in the forward estimator.
     */
    void resetOdometryCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                               std::shared_ptr<std_srvs::srv::Empty::Response> response);

    // ROS communication
    rclcpp::Subscription<rex_interfaces::msg::RoverControl>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<rex_interfaces::msg::VescStatus>::SharedPtr wheels_vel_feedback_sub_;
    rclcpp::Subscription<rex_interfaces::msg::RoverStatus>::SharedPtr status_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_auto_;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscription_;

    rclcpp::Publisher<rex_interfaces::msg::Wheels>::SharedPtr wheels_vel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_odom_pub_;

    // Time
    rclcpp::Time sim_time_;
    rclcpp::Duration publish_period_;

    // Buffers and state
    rex_interfaces::msg::RoverControl rover_cmd_velocity_;
    rex_interfaces::msg::Wheels rover_wheels_velocity_;
    rex_interfaces::msg::Wheels rover_wheels_velocity_feedback_;

    realtime_tools::RealtimeBuffer<rex_interfaces::msg::RoverControl> rover_cmd_velocity_buffer_;
    realtime_tools::RealtimeBuffer<rex_interfaces::msg::Wheels> rover_wheels_velocity_feedback_buffer_;

    // Owned components (new layered architecture)
    RoverConfig config_;
    InverseKinematics inverse_kinematics_;
    ForwardKinematics forward_kinematics_;
    HardwareInterface hardware_interface_;

    // Configuration is owned in `config_`; do not duplicate members here.
    int control_mode_;
    int communication_state_;

    // Feedback timeout handling
    rclcpp::Time last_feedback_time_;
    bool feedback_stale_{false};
    bool prev_feedback_stale_{false};

    bool initialized_ = false;

    // Helpers
    /**
     * @brief Convert a `WheelCommand` (meters/sec and radians) into the
     * `rex_interfaces::msg::Wheels` message with hardware units applied.
     *
     * This uses `HardwareInterface` to map physical units to hardware set
     * values (ERPM for drive, degrees or precise_pos for steering) and
     * populates command_id fields expected by the CAN bridge.
     *
     * @param command WheelCommand in physical units (m/s and rad)
     * @param time Timestamp to attach to the resulting message
     * @return rex_interfaces::msg::Wheels populated with hardware set values
     */
    rex_interfaces::msg::Wheels assembleWheelsFromCommand(const WheelCommand& command, const rclcpp::Time& time);
};
