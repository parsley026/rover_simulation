#include "rover_kinematics/KinematicsNode.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>

/**
 * @brief 
 * @param options 
 * @param config_file_path 
 * @return
 */
KinematicsNode::KinematicsNode(const rclcpp::NodeOptions &options, const std::string &config_file_path) : 
        Node("rover_kinematics", options),
        
        publish_period_(rclcpp::Duration::from_seconds(0.05)),
        
        config_(),
        kinematics_solver_(),
        kinematics_estimator(),
        hardware_interface_(),

        control_mode_(CONTROL_MODE_DRIVE),
        communication_state_(COMMUNICATION_STATE_CREATED)
{
    config_file_path_ = config_file_path;

    if (!config_.loadFromFile(config_file_path)) {
        RCLCPP_WARN(this->get_logger(), "Failed to load config from file: %s", config_file_path.c_str());
    }

    kinematics_solver_.setConfig(config_);
    kinematics_estimator.setConfig(config_);
    hardware_interface_.setConfig(config_);

    publish_period_ = rclcpp::Duration::from_seconds(1.0 / config_.publish_rate_);
    timer_ = this->create_wall_timer(publish_period_.to_chrono<std::chrono::milliseconds>(), std::bind(&KinematicsNode::onUpdate, this));

    initCmdVelBuffer();
    initFeedbackBuffer();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    
    clock_sub_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
        "/clock", 10, std::bind(&KinematicsNode::clockCallback, this, std::placeholders::_1));

    status_sub_ = this->create_subscription<rex_interfaces::msg::RoverStatus>(
        "/MQTT/RoverStatus",  qos, std::bind(&KinematicsNode::statusCallback,       this, std::placeholders::_1));

    cmd_vel_sub_manual_ = this->create_subscription<rex_interfaces::msg::RoverControl>(
        "/MQTT/RoverControl", qos, std::bind(&KinematicsNode::cmdVelManualCallback, this, std::placeholders::_1));

    wheels_vel_feedback_sub_ = this->create_subscription<rex_interfaces::msg::VescStatus>(
        "/CAN/RX/vesc_status", rclcpp::QoS(1000).reliable(), std::bind(&KinematicsNode::feedbackCallback, this, std::placeholders::_1));

    wheels_vel_pub_ = this->create_publisher<rex_interfaces::msg::Wheels>("/CAN/TX/set_motor_vel", qos);
    
    // - - -
    
    cmd_vel_sub_auto_ = this->create_subscription<geometry_msgs::msg::Twist>(
        config_.cmd_vel_autonomy_topic_, qos, std::bind(&KinematicsNode::cmdVelAutoCallback, this, std::placeholders::_1));
    
    // - - -

    tf_odom_pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", qos);

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/kinematics/odom", qos);

    // - - -
    
    reset_kinematics_srv_ = this->create_service<std_srvs::srv::Empty>(
        "reset_kinematics", std::bind(&KinematicsNode::resetKinematicsCallback, this, std::placeholders::_1, std::placeholders::_2));

    reset_odometry_srv_   = this->create_service<std_srvs::srv::Empty>(
        "reset_odometry",   std::bind(&KinematicsNode::resetOdometryCallback,   this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Initialized KinematicsNode: %s with config file: %s", this->get_name(), config_file_path.c_str());
}

/**
 * @brief 
 */
void KinematicsNode::initCmdVelBuffer() {
    rex_interfaces::msg::RoverControl cmd_vel;
    
    rclcpp::Time current_time;
    if (config_.use_sim_time_) {
        current_time = rclcpp::Time(
            sim_time_ns_.load(std::memory_order_relaxed),
            RCL_ROS_TIME
        );
    } else {
        current_time = this->get_clock()->now();
    }
    
    cmd_vel.header.stamp = current_time;
    
    cmd_vel.x_axis = 0.0;
    cmd_vel.y_axis = 0.0;
    cmd_vel.vel    = 0.0;
    // cmd_vel.mode   = CONTROL_MODE_NONE;
    
    
    rover_cmd_velocity_buffer_.writeFromNonRT(cmd_vel);
}

/**
 * @brief
 */
void KinematicsNode::initFeedbackBuffer() {
    rex_interfaces::msg::Wheels feedback;
    
    rclcpp::Time current_time;
    if (config_.use_sim_time_) {
        current_time = rclcpp::Time(
            sim_time_ns_.load(std::memory_order_relaxed),
            RCL_ROS_TIME
        );
    } else {
        current_time = this->get_clock()->now();
    }
    
    feedback.header.stamp = current_time;

    feedback.front_left.drive.set_value  = 0.0;
    feedback.front_left.turn.set_value   = 0.0;
    
    feedback.front_right.drive.set_value = 0.0;
    feedback.front_right.turn.set_value  = 0.0;
    
    feedback.rear_left.drive.set_value   = 0.0;
    feedback.rear_left.turn.set_value    = 0.0;

    feedback.rear_right.drive.set_value  = 0.0;
    feedback.rear_right.turn.set_value   = 0.0;

    rover_wheels_velocity_feedback_buffer_.writeFromNonRT(feedback);

    std::lock_guard<std::mutex> lock(feedback_mutex_);
    wheel_quality_weights_.fill(1.0);
    drive_quality_weights_.fill(1.0);
    steer_quality_weights_.fill(1.0);
    kinematics_estimator.setWheelQuality(wheel_quality_weights_);
}

double KinematicsNode::computeWheelQualityScore(const rex_interfaces::msg::VescStatus& status) const {
    if (!config_.enable_dynamic_wheel_weighting_) {
        return 1.0;
    }

    const double erpm = std::abs(static_cast<double>(status.erpm));
    const double current = std::abs(static_cast<double>(status.current));
    const double duty_cycle = std::abs(static_cast<double>(status.duty_cycle));

    if (erpm >= config_.wheel_quality_low_erpm_threshold_) {
        return 1.0;
    }

    const double current_threshold = std::max(config_.wheel_quality_high_current_threshold_, 1e-6);
    const double duty_threshold = std::clamp(config_.wheel_quality_high_duty_threshold_, 1e-6, 0.999999);

    const double current_penalty = std::max(0.0, (current - current_threshold) / current_threshold);
    const double duty_penalty = std::max(0.0, (duty_cycle - duty_threshold) / (1.0 - duty_threshold));

    const double quality = 1.0 / (1.0 + current_penalty + duty_penalty);
    return std::clamp(quality, config_.wheel_quality_min_weight_, 1.0);
}

void KinematicsNode::updateWheelQuality(std::size_t wheel_index, const rex_interfaces::msg::VescStatus& status) {
    if (wheel_index >= wheel_quality_weights_.size()) {
        return;
    }

    const double quality = computeWheelQualityScore(status);

    if (status.vesc_id >= 0x50 && status.vesc_id <= 0x53) {
        drive_quality_weights_[wheel_index] = quality;
    } else {
        steer_quality_weights_[wheel_index] = quality;
    }

    wheel_quality_weights_[wheel_index] = std::min(drive_quality_weights_[wheel_index], steer_quality_weights_[wheel_index]);
    kinematics_estimator.setWheelQuality(wheel_quality_weights_);
}

/**
 * @brief
 *
 * @param command 
 * @param time 
 * 
 * @return
 */
rex_interfaces::msg::Wheels KinematicsNode::assembleWheelsFromCommand(const WheelCommand& command, const rclcpp::Time& time) {
    rex_interfaces::msg::Wheels vel_msg;
    vel_msg.header.stamp = time;

    rex_interfaces::msg::Wheel* wheels[4] = {
        &vel_msg.front_left,   
        &vel_msg.front_right,  
        &vel_msg.rear_left,    
        &vel_msg.rear_right
    };

    for (std::size_t i = 0; i < 4; ++i) {
        wheels[i]->drive.command_id = RPM_CONTROL;
        wheels[i]->drive.set_value  = hardware_interface_.driveSetFromMetersPerSecond(command.drive_velocity_mps[i], i);
        
        wheels[i]->turn.command_id  = STEP_MOTOR_CONTROL;
        wheels[i]->turn.set_value   = hardware_interface_.steeringSetFromRadians(command.steering_angle_rad[i], i);
    }

    return vel_msg;
}

void KinematicsNode::updateWatchdog(const rclcpp::Time& current_time) {
    const bool was_feedback_stale = feedback_stale_.load(std::memory_order_acquire);
    
    const int64_t last_feedback_time_ns = last_feedback_time_ns_.load(std::memory_order_acquire);

    const bool    feedback_never_received = (last_feedback_time_ns == 0);
    const int64_t elapsed_time_ns = current_time.nanoseconds() - last_feedback_time_ns;
    const int64_t timeout_time_ns = static_cast<int64_t>(config_.feedback_timeout_sec_ * 1e9);

    const bool is_feedback_stale = feedback_never_received || (elapsed_time_ns > timeout_time_ns);
    feedback_stale_.store(is_feedback_stale, std::memory_order_release);

    if (is_feedback_stale && !was_feedback_stale) {
        RCLCPP_ERROR(get_logger(), "Wheel feedback stale for > %.3f sec. Braking.", config_.feedback_timeout_sec_);
        communication_state_.store(COMMUNICATION_STATE_CREATED, std::memory_order_release);
    } 
    else if (!is_feedback_stale && was_feedback_stale) {
        RCLCPP_INFO(get_logger(),  "Wheel feedback restored. Resuming normal operation.");
        communication_state_.store(COMMUNICATION_STATE_OPENED,  std::memory_order_release);
    }
}

/**
 * @brief
 * @return
 */
void KinematicsNode::onUpdate() {
    
    rclcpp::Time current_time;
    if (config_.use_sim_time_) {
        current_time = rclcpp::Time(
            sim_time_ns_.load(std::memory_order_relaxed),
            RCL_ROS_TIME
        );
    } else {
        current_time = this->get_clock()->now();
    }

    updateWatchdog(current_time);

    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        kinematics_estimator.setWheelQuality(wheel_quality_weights_);
    }

    if (feedback_stale_.load(std::memory_order_acquire)) {

    } else {
        auto feedback_msg = *(rover_wheels_velocity_feedback_buffer_.readFromNonRT());
        const auto estimate = kinematics_estimator.update(feedback_msg, current_time);
        
        if (estimate.valid) {
            odom_pub_->publish(estimate.odometry);
            if (config_.enable_odom_tf_) {
                tf_odom_pub_->publish(estimate.transform);
            }
        }
    }

    auto cmd = *(rover_cmd_velocity_buffer_.readFromNonRT());
    rex_interfaces::msg::Wheels target_wheels_msg;

    if (communication_state_.load(std::memory_order_acquire) == COMMUNICATION_STATE_OPENED) {
        WheelCommand target_ik{};
        
        switch (control_mode_.load(std::memory_order_acquire)) {

            // case CONTROL_MODE_NONE:
            //     break;
            // case CONTROL_MODE_ESTOP:
            //     break;
            // case CONTROL_MODE_STOP:
            //     break;
            // case CONTROL_MODE_CONFIG:
            //     break;

            case CONTROL_MODE_DRIVE:
                switch (cmd.mode) {
                    case SYMMETRIC_ACKERMANN_MODE:
                    {
                        double linear_m_s = cmd.vel;
                        double angular_rad_s = 0.0;
                        double axis_abs = std::fabs(cmd.x_axis);

                        // 1. Ghost Velocity: If stationary, use 1.0 m/s to calculate steering geometry
                        double calc_linear_m_s = (std::fabs(linear_m_s) < 1e-9) ? 1.0 : linear_m_s;

                        if (axis_abs > 1e-9) {
                            double radius_a_ratio = (config_.min_steering_radius_ - config_.max_steering_radius_) / 0.99;
                            double radius_b_ratio = (config_.min_steering_radius_ - radius_a_ratio);
                            double physical_radius = (radius_a_ratio * axis_abs) + radius_b_ratio;
                            double sign = (cmd.x_axis >= 0.0) ? 1.0 : -1.0;
                            
                            // 2. Use the ghost velocity to find the correct angular rate
                            angular_rad_s = (calc_linear_m_s / physical_radius) * sign;
                        }
                        
                        // 3. Solve IK using the ghost velocity (generates correct angles)
                        target_ik = kinematics_solver_.computeSymmetricAckermann(calc_linear_m_s, angular_rad_s);
                        
                        // 4. Mute the drive motors if the actual commanded velocity is 0
                        if (std::fabs(linear_m_s) < 1e-9) {
                            target_ik.drive_velocity_mps.fill(0.0);
                        }
                        
                        break;
                    }

                    // case FRONT_ACKERMANN_MODE:
                    //     break;
                    // case REAR_ACKERMANN_MODE:
                    //     break;

                    case CRAB_MODE:
                    {
                        double axis_y = cmd.y_axis < 0.0 ? 0.0 : cmd.y_axis;

                        // 1. Calculate global translation angle from joystick axes
                        double theta = 0.0;
                        if (std::fabs(axis_y) >= 1e-9 || std::fabs(cmd.x_axis) >= 1e-9) {
                            theta = std::atan2(cmd.x_axis, axis_y);
                        }

                        // 2. Ghost Velocity: Pretend we are moving at 1.0 m/s if throttle is zero
                        double calc_vel = (std::fabs(cmd.vel) < 1e-9) ? 1.0 : cmd.vel;

                        // 3. Convert polar joystick command to Cartesian body velocities
                        double linear_x_m_s = calc_vel * std::cos(theta);
                        double linear_y_m_s = calc_vel * std::sin(theta);

                        // 4. Pass the calculated velocities to the solver to get the correct angles
                        target_ik = kinematics_solver_.computeCrab(linear_x_m_s, linear_y_m_s);
                        
                        // 5. Mute the drive motors if the actual commanded velocity is 0
                        if (std::fabs(cmd.vel) < 1e-9) {
                            target_ik.drive_velocity_mps.fill(0.0);
                        }
                        
                        break;
                    }
                    case SYMMETRIC_SPIN_MODE:
                    {
                        double calc_omega = (std::fabs(cmd.vel) < 1e-9) ? 1.0 : cmd.vel;

                        // 2. Solve IK to force the wheels into the spin circle
                        target_ik = kinematics_solver_.computeSymmetricSpin(calc_omega);

                        // 3. Mute the drive motors if the actual commanded velocity is 0
                        if (std::fabs(cmd.vel) < 1e-9) {
                            target_ik.drive_velocity_mps.fill(0.0);
                        }
                        break;
                    }
                    default:
                        break;
                }
                break;

            // case CONTROL_MODE_ROBOTIC_ARM:
            //     break;
            // case CONTROL_MODE_DEEP_SAMPLER:
            //     target_ik = kinematics_solver_.computeXConfiguration();
            //     break;
            // case CONTROL_MODE_SURFACE_SAMPLER:
            //     target_ik = kinematics_solver_.computeXConfiguration();
            //     break;

            case CONTROL_MODE_DRIVE_AUTONOMY:
                switch (cmd.mode) {
                    case SYMMETRIC_ACKERMANN_MODE:
                        target_ik = kinematics_solver_.computeSymmetricAckermann(cmd.vel, cmd.x_axis);
                        break;
                    // case FRONT_ACKERMANN_MODE:
                    //     break;
                    // case REAR_ACKERMANN_MODE:
                    //     break;
                    // case CRAB_MODE:
                    //     break;
                    // case SYMMETRIC_SPIN_MODE:
                    //     break;
                    default:
                        break;
                }
                break;

            // case CONTROL_MODE_ROBOTIC_ARM_AUTONOMY:
            //     break;
            // case CONTROL_MODE_DEEP_SAMPLER_AUTONOMY:
            //     target_ik = kinematics_solver_.computeXConfiguration();
            //     break;
            // case CONTROL_MODE_SURFACE_SAMPLER_AUTONOMY:
            //     target_ik = kinematics_solver_.computeXConfiguration();
            //     break;

            default:
                break;
        }

        target_wheels_msg = assembleWheelsFromCommand(target_ik, current_time);
    } else {
        rex_interfaces::msg::RoverControl zero_cmd;
        
        zero_cmd.header.stamp = current_time;

        zero_cmd.x_axis = 0.0; 
        zero_cmd.y_axis = 0.0;
        zero_cmd.vel = 0.0;
        
        rover_cmd_velocity_buffer_.writeFromNonRT(zero_cmd);

        target_wheels_msg.header.stamp = current_time;

        target_wheels_msg.front_left.drive.command_id = CURRENT_CONTROL;
        target_wheels_msg.front_left.drive.set_value  = 0.0;
        target_wheels_msg.front_left.turn.command_id  = STEP_MOTOR_CONTROL;
        target_wheels_msg.front_left.turn.set_value   = 0.0;
        
        target_wheels_msg.front_right.drive.command_id = CURRENT_CONTROL;
        target_wheels_msg.front_right.drive.set_value  = 0.0;
        target_wheels_msg.front_right.turn.command_id  = STEP_MOTOR_CONTROL;
        target_wheels_msg.front_right.turn.set_value   = 0.0;
        
        target_wheels_msg.rear_left.drive.command_id = CURRENT_CONTROL;
        target_wheels_msg.rear_left.drive.set_value  = 0.0;
        target_wheels_msg.rear_left.turn.command_id  = STEP_MOTOR_CONTROL;
        target_wheels_msg.rear_left.turn.set_value   = 0.0;

        target_wheels_msg.rear_right.drive.command_id = CURRENT_CONTROL;
        target_wheels_msg.rear_right.drive.set_value  = 0.0;
        target_wheels_msg.rear_right.turn.command_id  = STEP_MOTOR_CONTROL;
        target_wheels_msg.rear_right.turn.set_value   = 0.0;
    }
    
    rover_wheels_velocity_ = target_wheels_msg;

    wheels_vel_pub_->publish(rover_wheels_velocity_);
}

/**
 * @brief 
 * @param msg
 * @return
 */
void KinematicsNode::clockCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg) {
    constexpr int64_t kNanosecondsPerSecond = 1'000'000'000LL;

    const int64_t ns =
        static_cast<int64_t>(msg->clock.sec) * kNanosecondsPerSecond +
        static_cast<int64_t>(msg->clock.nanosec);
        
    sim_time_ns_.store(ns, std::memory_order_relaxed);
}

/**
 * @brief
 * @param msg
 * @return
 */
void KinematicsNode::statusCallback(const rex_interfaces::msg::RoverStatus::SharedPtr msg) {
    communication_state_.store(msg->communication_state, std::memory_order_release);
           control_mode_.store(msg->control_mode,        std::memory_order_release);
}

/**
 * @brief 
 * @param msg 
 * @return
 */
void KinematicsNode::cmdVelManualCallback(const rex_interfaces::msg::RoverControl::SharedPtr msg) {
    if (control_mode_.load(std::memory_order_acquire) == CONTROL_MODE_DRIVE) {
        rover_cmd_velocity_buffer_.writeFromNonRT(*msg);
    }
}

/**
 * @brief 
 * @param msg 
 * @return
 */
void KinematicsNode::cmdVelAutoCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (control_mode_.load(std::memory_order_acquire) == CONTROL_MODE_DRIVE_AUTONOMY) {
        rex_interfaces::msg::RoverControl local_cmd;

        local_cmd.mode   = SYMMETRIC_ACKERMANN_MODE;
        local_cmd.vel    = msg->linear.x;
        local_cmd.x_axis = msg->angular.z;

        rover_cmd_velocity_buffer_.writeFromNonRT(local_cmd);
    }
}


/**
 * @brief 
 * @param msg 
 * @return
 */
void KinematicsNode::feedbackCallback(const rex_interfaces::msg::VescStatus::SharedPtr msg) {
    
    std::lock_guard<std::mutex> lock(feedback_mutex_);

    rex_interfaces::msg::Wheels current_feedback = *(rover_wheels_velocity_feedback_buffer_.readFromNonRT());
    
    rclcpp::Time receive_time;
    if (config_.use_sim_time_) {
        receive_time = rclcpp::Time(
            sim_time_ns_.load(std::memory_order_relaxed),
            RCL_ROS_TIME
        );
    } else {
        receive_time = this->get_clock()->now();
    }

    rclcpp::Time measurement_time;
    if (config_.use_measurement_timestamp_ && (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0)) {
        measurement_time = rclcpp::Time(msg->header.stamp);
    } else {
        measurement_time = receive_time;
    }

    current_feedback.header.stamp = measurement_time;

    switch (msg->vesc_id) {
        case 0x50:
            current_feedback.front_left.drive.set_value = hardware_interface_.metersPerSecondFromErpm(msg->erpm, HardwareInterface::vescIdToDriveWheelIndex(msg->vesc_id));
            updateWheelQuality(0, *msg);
            break;
        case 0x51:
            current_feedback.front_right.drive.set_value = hardware_interface_.metersPerSecondFromErpm(msg->erpm, HardwareInterface::vescIdToDriveWheelIndex(msg->vesc_id));
            updateWheelQuality(1, *msg);
            break;
        case 0x52:
            current_feedback.rear_left.drive.set_value = hardware_interface_.metersPerSecondFromErpm(msg->erpm, HardwareInterface::vescIdToDriveWheelIndex(msg->vesc_id));
            updateWheelQuality(2, *msg);
            break;
        case 0x53:
            current_feedback.rear_right.drive.set_value = hardware_interface_.metersPerSecondFromErpm(msg->erpm, HardwareInterface::vescIdToDriveWheelIndex(msg->vesc_id));
            updateWheelQuality(3, *msg);
            break;
        case 0x60:
            current_feedback.front_left.turn.set_value = hardware_interface_.steeringSetFromPrecisePos(msg->precise_pos, HardwareInterface::vescIdToTurnWheelIndex(msg->vesc_id));
            updateWheelQuality(0, *msg);
            break;
        case 0x61:
            current_feedback.front_right.turn.set_value = hardware_interface_.steeringSetFromPrecisePos(msg->precise_pos, HardwareInterface::vescIdToTurnWheelIndex(msg->vesc_id));
            updateWheelQuality(1, *msg);
            break;
        case 0x62:
            current_feedback.rear_right.turn.set_value = hardware_interface_.steeringSetFromPrecisePos(msg->precise_pos, HardwareInterface::vescIdToTurnWheelIndex(msg->vesc_id));
            updateWheelQuality(3, *msg);
            break;
        case 0x63:
            current_feedback.rear_left.turn.set_value = hardware_interface_.steeringSetFromPrecisePos(msg->precise_pos, HardwareInterface::vescIdToTurnWheelIndex(msg->vesc_id));
            updateWheelQuality(2, *msg);
            break;
        default:
            break;
    }

    rover_wheels_velocity_feedback_buffer_.writeFromNonRT(current_feedback);

    last_feedback_time_ns_.store(measurement_time.nanoseconds(), std::memory_order_release);
    feedback_stale_.store(false, std::memory_order_release);
    communication_state_.store(COMMUNICATION_STATE_OPENED, std::memory_order_release);
}

void KinematicsNode::resetOdometryCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
          std::shared_ptr<std_srvs::srv::Empty::Response>
) {
    RCLCPP_INFO(this->get_logger(), "Resetting Odometry Node");
    
    kinematics_estimator.reset();
}


void KinematicsNode::resetKinematicsCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
          std::shared_ptr<std_srvs::srv::Empty::Response>
) {
    RCLCPP_INFO(this->get_logger(), "Restarting Kinematics Node: Reloading config...");

    config_.initialized_ = false;

    if (config_.loadFromFile(config_file_path_)) {

        kinematics_solver_.setConfig(config_);
        kinematics_estimator.setConfig(config_);
        hardware_interface_.setConfig(config_);

        {
            std::lock_guard<std::mutex> lock(feedback_mutex_);
            wheel_quality_weights_.fill(1.0);
            drive_quality_weights_.fill(1.0);
            steer_quality_weights_.fill(1.0);
            kinematics_estimator.setWheelQuality(wheel_quality_weights_);
        }

        kinematics_estimator.reset();

        timer_->cancel();
        publish_period_ = rclcpp::Duration::from_seconds(1.0 / config_.publish_rate_);
        timer_ = this->create_wall_timer(publish_period_.to_chrono<std::chrono::milliseconds>(), std::bind(&KinematicsNode::onUpdate, this));

        RCLCPP_INFO(this->get_logger(),  "succeeded to reload config file: %s", config_file_path_.c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "failed    to reload config file: %s", config_file_path_.c_str());
    }
}