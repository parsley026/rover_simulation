#include "quad_rover_kinematics/RoverNode.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

/**
 * @brief Construct the RoverNode and initialize subscriptions/publishers.
 *
 * Loads configuration from `config_file_path`, initializes subcomponents
 * (`InverseKinematics`, `ForwardKinematics`, `HardwareInterface`) and
 * starts a periodic timer that calls `onUpdate()` at the configured rate.
 *
 * @param options rclcpp NodeOptions forwarded to the base `rclcpp::Node`.
 * @param config_file_path Absolute path to the YAML configuration file.
 */
RoverNode::RoverNode(const rclcpp::NodeOptions &options, const std::string &config_file_path)
        : Node("quad_rover_kinematics", options),
            sim_time_(0, 0, RCL_SYSTEM_TIME),
            publish_period_(rclcpp::Duration::from_seconds(0.05)),
        inverse_kinematics_(),
        forward_kinematics_(),
        config_(),
        hardware_interface_(),
                        control_mode_(USER_CONTROL), communication_state_(CONNECTION_DISCONNECTED)
{
    // Load configuration via RoverConfig::loadFromFile to lock immutability
    if (!config_.loadFromFile(config_file_path)) {
        RCLCPP_WARN(this->get_logger(), "Failed to load config via loadFromFile(), falling back to readConfig().");
        readConfig(config_file_path);
    } else {
        // Apply config to components and compute derived values
        publish_period_ = rclcpp::Duration::from_seconds(1.0 / config_.publish_rate_);
    }
    setCmdVelocity(0,0,0);

    inverse_kinematics_.setConfig(config_);
    forward_kinematics_.setConfig(config_);
    forward_kinematics_.setFrames(config_.odom_frame_id_, config_.base_frame_id_);
    forward_kinematics_.setSteeringRadius(config_.min_steering_radius_, config_.max_steering_radius_);
    forward_kinematics_.setWheelRadius(config_.wheel_radius_);
    forward_kinematics_.setPolesPairsNumber(config_.poles_pairs_number_);
    forward_kinematics_.setMotorGearRatio(config_.motor_gear_ratio_);
    hardware_interface_.setConfig(config_);

    timer_ = this->create_wall_timer(
        publish_period_.to_chrono<std::chrono::milliseconds>(), std::bind(&RoverNode::onUpdate, this));

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

    cmd_vel_sub_ = this->create_subscription<rex_interfaces::msg::RoverControl>(
        "/MQTT/RoverControl", qos, std::bind(&RoverNode::cmdVelCallback, this, std::placeholders::_1));

    cmd_vel_sub_auto_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", qos, std::bind(&RoverNode::cmdVelAutoCallback, this, std::placeholders::_1));

    status_sub_ = this->create_subscription<rex_interfaces::msg::RoverStatus>(
        "/MQTT/RoverStatus", qos, std::bind(&RoverNode::statusCallback, this, std::placeholders::_1));

    wheels_vel_feedback_sub_ = this->create_subscription<rex_interfaces::msg::VescStatus>(
        "/CAN/RX/vesc_status", rclcpp::QoS(1000).reliable(), std::bind(&RoverNode::feedbackCallback, this, std::placeholders::_1));

    wheels_vel_pub_ = this->create_publisher<rex_interfaces::msg::Wheels>("/CAN/TX/set_motor_vel", qos);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/kinematic/odometry", qos);
    tf_odom_pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", qos);

    reset_odom_srv_ = this->create_service<std_srvs::srv::Empty>(
        "reset_odometry", std::bind(&RoverNode::resetOdometryCallback, this, std::placeholders::_1, std::placeholders::_2));

    clock_subscription_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
        "/clock", 10, std::bind(&RoverNode::clockCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "RoverNode started.");
}

/**
 * @brief Loads rover configuration from a YAML file and applies it to the node components.
 *
 * @param config_file_path Path to the YAML configuration file.
 */
void RoverNode::readConfig(const std::string &config_file_path) {
    try {
        YAML::Node config = YAML::LoadFile(config_file_path);
        config_.loadFromYaml(config);
        publish_period_ = rclcpp::Duration::from_seconds(1.0 / config_.publish_rate_);

        inverse_kinematics_.setConfig(config_);
        forward_kinematics_.setConfig(config_);
        hardware_interface_.setConfig(config_);

        RCLCPP_INFO(this->get_logger(), "YAML configuration loaded successfully!");
    } catch (const YAML::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Error reading configuration file: %s", e.what());
    }
}

/**
 * @brief Service callback that resets the odometry estimate in `ForwardKinematics`.
 *
 * This is exposed as the `reset_odometry` ROS service. It resets internal
 * pose and estimator state to zero so the next sensor fusion starts fresh.
 */
void RoverNode::resetOdometryCallback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                                      std::shared_ptr<std_srvs::srv::Empty::Response>)
{
    RCLCPP_INFO(this->get_logger(), "Resetting odometry");
    forward_kinematics_.reset();
}

/**
 * @brief Sets the rover's commanded motion and its timestamp.
 *
 * @param x_vel Command along the x axis.
 * @param y_vel Command along the y axis.
 * @param speed Commanded speed.
 */
void RoverNode::setCmdVelocity(double x_vel, double y_vel, double speed) {
    rover_cmd_velocity_.header.stamp = config_.use_sim_time_ ? sim_time_ : this->get_clock()->now();
    rover_cmd_velocity_.x_axis = x_vel;
    rover_cmd_velocity_.y_axis = y_vel;
    rover_cmd_velocity_.vel = speed;
}

/**
 * @brief Convert a `WheelCommand` in physical units to a hardware `Wheels` message.
 *
 * This function uses `HardwareInterface` to convert per-wheel linear speeds
 * (m/s) and steering angles (rad) into hardware set values and populates
 * the appropriate `command_id` fields required by the CAN bridge.
 *
 * @param command WheelCommand containing per-wheel speeds (m/s) and steering (rad).
 * @param time Timestamp to tag the produced `Wheels` message.
 * @return A `rex_interfaces::msg::Wheels` message ready for publication.
 */
rex_interfaces::msg::Wheels RoverNode::assembleWheelsFromCommand(const WheelCommand& command, const rclcpp::Time& time) {
    rex_interfaces::msg::Wheels vel;
    vel.header.stamp = time;

    auto setWheelParams = [&](rex_interfaces::msg::Wheel &wheel, std::size_t index) {
        wheel.drive.set_value = hardware_interface_.driveSetFromMetersPerSecond(command.drive_m_s[index], index);
        wheel.turn.set_value = hardware_interface_.steeringSetFromRadians(command.steer_rad[index], index);
        wheel.turn.command_id = STEP_MOTOR_CONTROL;
        wheel.drive.command_id = RPM_CONTROL;
    };

    setWheelParams(vel.front_left, 0);
    setWheelParams(vel.front_right, 1);
    setWheelParams(vel.rear_left, 2);
    setWheelParams(vel.rear_right, 3);

    return vel;
}

/**
 * @brief Processes wheel feedback and commands, publishing odometry and wheel setpoints.
 *
 * Enters brake mode when wheel feedback is stale or communication is disconnected.
 * Otherwise, updates kinematics, publishes valid odometry and optional TF data, and
 * publishes wheel commands for the active control mode.
 */
void RoverNode::onUpdate() {
    rclcpp::Time time_curr = config_.use_sim_time_ ? sim_time_ : this->get_clock()->now();

    auto feedback_msg = *(rover_wheels_velocity_feedback_buffer_.readFromNonRT());

    // Feedback timeout check based on last_feedback_time_
    prev_feedback_stale_ = feedback_stale_;
    feedback_stale_ = false;
    if (last_feedback_time_.nanoseconds() == 0) {
        feedback_stale_ = true;
    } else if ((time_curr - last_feedback_time_).seconds() > config_.feedback_timeout_sec_) {
        feedback_stale_ = true;
    }

    if (feedback_stale_) {
        if (!prev_feedback_stale_) {
            // first tick entering stale state: log and publish brake
            RCLCPP_ERROR(this->get_logger(), "Wheel feedback stale for > %.3f sec, entering hardware fault state.", config_.feedback_timeout_sec_);
            rex_interfaces::msg::Wheels brake_msg;
            brake_msg.header.stamp = time_curr;
            brake_msg.front_left.drive.command_id = CURRENT_CONTROL;
            brake_msg.front_left.turn.command_id = STEP_MOTOR_CONTROL;
            brake_msg.front_left.turn.set_value = 0.0;
            brake_msg.front_right.drive.command_id = CURRENT_CONTROL;
            brake_msg.front_right.turn.command_id = STEP_MOTOR_CONTROL;
            brake_msg.front_right.turn.set_value = 0.0;
            brake_msg.rear_right.drive.command_id = CURRENT_CONTROL;
            brake_msg.rear_right.turn.command_id = STEP_MOTOR_CONTROL;
            brake_msg.rear_right.turn.set_value = 0.0;
            brake_msg.rear_left.drive.command_id = CURRENT_CONTROL;
            brake_msg.rear_left.turn.command_id = STEP_MOTOR_CONTROL;
            brake_msg.rear_left.turn.set_value = 0.0;
            brake_msg.front_left.drive.set_value = 0.0;
            brake_msg.front_right.drive.set_value = 0.0;
            brake_msg.rear_right.drive.set_value = 0.0;
            brake_msg.rear_left.drive.set_value = 0.0;

            wheels_vel_pub_->publish(brake_msg);
            communication_state_ = CONNECTION_DISCONNECTED;
        }
        // skip forward kinematics update while stale
    } else {
        if (prev_feedback_stale_) {
            RCLCPP_INFO(this->get_logger(), "Wheel feedback recovered; resuming normal operation.");
            communication_state_ = CONNECTION_CONNECTED;
        }
        const auto estimate = forward_kinematics_.update(feedback_msg, time_curr);
        if (estimate.valid) {
            odom_pub_->publish(estimate.odometry);
            if (config_.enable_odom_tf_) tf_odom_pub_->publish(estimate.transform);
        }
    }

    auto cmd_vel = *(rover_cmd_velocity_buffer_.readFromNonRT());

    if (communication_state_ != CONNECTION_CONNECTED) {
        // brake
        rex_interfaces::msg::Wheels brake_msg;
        brake_msg.header.stamp = time_curr;
        brake_msg.front_left.drive.command_id = CURRENT_CONTROL;
        brake_msg.front_left.turn.command_id = STEP_MOTOR_CONTROL;
        brake_msg.front_left.turn.set_value = 0.0;
        brake_msg.front_right.drive.command_id = CURRENT_CONTROL;
        brake_msg.front_right.turn.command_id = STEP_MOTOR_CONTROL;
        brake_msg.front_right.turn.set_value = 0.0;
        brake_msg.rear_right.drive.command_id = CURRENT_CONTROL;
        brake_msg.rear_right.turn.command_id = STEP_MOTOR_CONTROL;
        brake_msg.rear_right.turn.set_value = 0.0;
        brake_msg.rear_left.drive.command_id = CURRENT_CONTROL;
        brake_msg.rear_left.turn.command_id = STEP_MOTOR_CONTROL;
        brake_msg.rear_left.turn.set_value = 0.0;
        brake_msg.front_left.drive.set_value = 0.0;
        brake_msg.front_right.drive.set_value = 0.0;
        brake_msg.rear_right.drive.set_value = 0.0;
        brake_msg.rear_left.drive.set_value = 0.0;

        rover_wheels_velocity_ = brake_msg;
        cmd_vel.vel = 0.0; cmd_vel.x_axis = 0.0; cmd_vel.y_axis = 0.0;
        rover_cmd_velocity_buffer_.writeFromNonRT(cmd_vel);
    } else {
        if (control_mode_ == USER_CONTROL) {
            if (cmd_vel.mode == ADVANCE_MODE) {
                const auto c = inverse_kinematics_.computeAdvance(cmd_vel.x_axis, cmd_vel.vel);
                rover_wheels_velocity_ = assembleWheelsFromCommand(c, time_curr);
            } else if (cmd_vel.mode == CRAB_MODE) {
                const auto c = inverse_kinematics_.computeCrab(cmd_vel.x_axis, cmd_vel.y_axis, cmd_vel.vel);
                rover_wheels_velocity_ = assembleWheelsFromCommand(c, time_curr);
            } else if (cmd_vel.mode == SPIN_MODE) {
                const auto c = inverse_kinematics_.computeSpin(cmd_vel.vel);
                rover_wheels_velocity_ = assembleWheelsFromCommand(c, time_curr);
            } else {
                // brake
                rover_wheels_velocity_ = assembleWheelsFromCommand(WheelCommand{}, time_curr);
            }
        } else if (control_mode_ == AUTONOMY_CONTROL) {
            if (cmd_vel.mode == ADVANCE_MODE) {
                const auto c = inverse_kinematics_.computeAdvance(cmd_vel.x_axis, cmd_vel.vel);
                rover_wheels_velocity_ = assembleWheelsFromCommand(c, time_curr);
            } else if (cmd_vel.mode == SPIN_MODE) {
                const auto c = inverse_kinematics_.computeSpin(cmd_vel.vel);
                rover_wheels_velocity_ = assembleWheelsFromCommand(c, time_curr);
            } else {
                rover_wheels_velocity_ = assembleWheelsFromCommand(WheelCommand{}, time_curr);
            }
        } else {
            rover_wheels_velocity_ = assembleWheelsFromCommand(WheelCommand{}, time_curr);
        }
    }

    wheels_vel_pub_->publish(rover_wheels_velocity_);
}

/**
 * @brief Callback for operator/MQTT `RoverControl` messages.
 *
 * Only accepted when the node is in `USER_CONTROL` mode; stores the message
 * in a realtime buffer for `onUpdate()` to process.
 */
void RoverNode::cmdVelCallback(const rex_interfaces::msg::RoverControl::SharedPtr msg) {
    if (control_mode_ == USER_CONTROL) {
        rover_cmd_velocity_buffer_.writeFromNonRT(*msg);
    }
}

/**
 * @brief Updates per-wheel feedback from a VESC status message.
 *
 * Converts drive ERPM or steering position according to the VESC identifier
 * and records the resulting feedback for subsequent kinematic updates.
 *
 * @param msg VESC status message containing the controller identifier and feedback value.
 */
void RoverNode::feedbackCallback(const rex_interfaces::msg::VescStatus::SharedPtr msg) {
    rover_wheels_velocity_feedback_.header.stamp = config_.use_sim_time_ ? sim_time_ : this->get_clock()->now();

    switch (msg->vesc_id) {
        case 0x50:
            this->rover_wheels_velocity_feedback_.front_left.drive.set_value = hardware_interface_.metersPerSecondFromErpm(msg->erpm, HardwareInterface::vescIdToDriveWheelIndex(msg->vesc_id));
            break;
        case 0x51:
            this->rover_wheels_velocity_feedback_.front_right.drive.set_value = hardware_interface_.metersPerSecondFromErpm(msg->erpm, HardwareInterface::vescIdToDriveWheelIndex(msg->vesc_id));
            break;
        case 0x52:
            this->rover_wheels_velocity_feedback_.rear_left.drive.set_value =  hardware_interface_.metersPerSecondFromErpm(msg->erpm, HardwareInterface::vescIdToDriveWheelIndex(msg->vesc_id));
            break;
        case 0x53:
            this->rover_wheels_velocity_feedback_.rear_right.drive.set_value =  hardware_interface_.metersPerSecondFromErpm(msg->erpm, HardwareInterface::vescIdToDriveWheelIndex(msg->vesc_id));
            break;
        case 0x60:
            this->rover_wheels_velocity_feedback_.front_left.turn.set_value = hardware_interface_.steeringSetFromPrecisePos(msg->precise_pos, HardwareInterface::vescIdToTurnWheelIndex(msg->vesc_id));
            break;
        case 0x61:
            this->rover_wheels_velocity_feedback_.front_right.turn.set_value = hardware_interface_.steeringSetFromPrecisePos(msg->precise_pos, HardwareInterface::vescIdToTurnWheelIndex(msg->vesc_id));
            break;
        case 0x62:
            this->rover_wheels_velocity_feedback_.rear_right.turn.set_value = hardware_interface_.steeringSetFromPrecisePos(msg->precise_pos, HardwareInterface::vescIdToTurnWheelIndex(msg->vesc_id));
            break;
        case 0x63:
            this->rover_wheels_velocity_feedback_.rear_left.turn.set_value = hardware_interface_.steeringSetFromPrecisePos(msg->precise_pos, HardwareInterface::vescIdToTurnWheelIndex(msg->vesc_id));
            break;
        default:
            break;
    }

    rover_wheels_velocity_feedback_buffer_.writeFromNonRT(rover_wheels_velocity_feedback_);

    // Mark last feedback time and communication state
    last_feedback_time_ = rover_wheels_velocity_feedback_.header.stamp;
    feedback_stale_ = false;
    communication_state_ = CONNECTION_CONNECTED;
}

/**
 * @brief Callback for high-level rover status updates (control and comms state).
 * @param msg Incoming `RoverStatus` message produced by monitoring components.
 */
void RoverNode::statusCallback(const rex_interfaces::msg::RoverStatus::SharedPtr msg) {
    communication_state_ = msg->communication_state;
    control_mode_ = msg->control_mode;
}

/**
 * @brief Converts autonomous velocity commands into rover control commands.
 *
 * Maps forward velocity and yaw rate to advance, spin, or turning commands.
 * Turning-axis values are constrained to the configured steering range.
 *
 * @param msg Incoming velocity command.
 */
void RoverNode::cmdVelAutoCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (control_mode_ == AUTONOMY_CONTROL) {
        if (msg->linear.x != 0.0 && msg->angular.z != 0.0) {
            rover_cmd_velocity_.mode = ADVANCE_MODE;
            rover_cmd_velocity_.vel = msg->linear.x;

            // 1. Calculate physical target turning radius (R = v / w)
            double target_radius = msg->linear.x / msg->angular.z;
            double target_radius_abs = std::fabs(target_radius);

            // 2. Reverse map physical radius to the normalized axis expected by computeAdvance()
            // Using your kinematics formula: steering_radius = a * axis + b
            double radius_a_ratio = (config_.min_steering_radius_ - config_.max_steering_radius_) / 0.99;
            double radius_b_ratio = config_.min_steering_radius_ - radius_a_ratio;
            
            double mapped_axis_mag = (target_radius_abs - radius_b_ratio) / radius_a_ratio;

            // 3. Clamp magnitude to [0.0, 1.0] to prevent mechanical boundary violations
            mapped_axis_mag = std::max(0.0, std::min(1.0, mapped_axis_mag));

            // 4. Apply steering direction: positive radius means center of rotation is to the left
            double sign_steer = (target_radius >= 0.0) ? 1.0 : -1.0;
            rover_cmd_velocity_.x_axis = sign_steer * mapped_axis_mag;

            rover_cmd_velocity_buffer_.writeFromNonRT(rover_cmd_velocity_);
        }
        else if (msg->linear.x != 0.0 && msg->angular.z == 0.0) {
            rover_cmd_velocity_.mode = ADVANCE_MODE;
            rover_cmd_velocity_.vel = msg->linear.x;
            rover_cmd_velocity_.x_axis = 0.0;
            rover_cmd_velocity_buffer_.writeFromNonRT(rover_cmd_velocity_);
        }
        else if (msg->linear.x == 0.0 && msg->angular.z != 0.0) {
            rover_cmd_velocity_.mode = SPIN_MODE;
            rover_cmd_velocity_.vel = msg->angular.z;
            rover_cmd_velocity_.x_axis = 0.0;
            rover_cmd_velocity_buffer_.writeFromNonRT(rover_cmd_velocity_);
        }
        else {
            rover_cmd_velocity_.mode = ADVANCE_MODE;
            rover_cmd_velocity_.vel = 0.0;
            rover_cmd_velocity_.x_axis = 0.0;
            rover_cmd_velocity_buffer_.writeFromNonRT(rover_cmd_velocity_);
        }
    }
}

/**
 * @brief Update the internal simulated time used when `use_sim_time` is true.
 */
void RoverNode::clockCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg) {
    sim_time_ = msg->clock;
}
