#include "rover_robot_arm_kinematics_bridge/bridge_modes/simulation_mode.hpp"
#include "rover_robot_arm_kinematics_bridge/core/arm_constants.hpp"
#include <unordered_map>
#include <functional>

namespace rover_robot_arm_kinematics_bridge {
namespace bridge_modes {

SimulationMode::SimulationMode(
    rclcpp::Node* node,
    std::shared_ptr<gazebo::GazeboArmPublisher> gazebo_pub,
    std::shared_ptr<core::Conversions> conversions,
    const ArmBridgeConfig& config)
    : node_(node)
    , gazebo_pub_(gazebo_pub)
    , conversions_(conversions)
    , config_(config)
{
    manipulator_control_sub_ = node_->create_subscription<rex_interfaces::msg::ManipulatorControl>(
        "/CAN/TX/manipulator_control", 10,
        std::bind(&SimulationMode::manipulatorControlCallback, this, std::placeholders::_1));

    manipulator_ctl_sub_ = node_->create_subscription<rex_interfaces::msg::ManipulatorControl>(
        "/CAN/TX/manipulator_ctl", 10,
        std::bind(&SimulationMode::manipulatorControlCallback, this, std::placeholders::_1));

    arm_trajectory_sub_ = node_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        "/robot_arm/joint_trajectory", 10,
        std::bind(&SimulationMode::armTrajectoryCallback, this, std::placeholders::_1));

    feedback_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::QoS(10).reliable(),
        std::bind(&SimulationMode::feedbackCallback, this, std::placeholders::_1));

    vesc_feedback_pub_ = node_->create_publisher<rex_interfaces::msg::VescStatus>(
        "/CAN/RX/vesc_status", rclcpp::QoS(10).reliable());
}

void SimulationMode::update()
{
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        predicted_state_current_    = predicted_state_buffor_;
        simulated_feedback_current_ = simulated_feedback_buffor_;
    }

    publishSimFeedback();
    gazebo_pub_->publish(predicted_state_current_);
}

void SimulationMode::manipulatorControlCallback(const rex_interfaces::msg::ManipulatorControl::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    for (size_t i = 0; i < core::NUM_ARM_JOINTS && i < 6; ++i) {
        double cmd = msg->axes[i].set_value;
        if (cmd == 0.0 && msg->axes[i].set_pos_speed_loop_position != 0.0) {
            cmd = msg->axes[i].set_pos_speed_loop_position;
        }
        if (config_.invert_joints[i]) {
            cmd = -cmd;
        }
        predicted_state_buffor_.joints[i].position = cmd;
    }

    double grip_cmd = msg->gripper.set_value;
    if (grip_cmd == 0.0 && msg->gripper.set_pos_speed_loop_position != 0.0) {
        grip_cmd = msg->gripper.set_pos_speed_loop_position;
    }
    if (config_.invert_gripper) {
        grip_cmd = -grip_cmd;
    }
    predicted_state_buffor_.gripper1.position = grip_cmd;
    predicted_state_buffor_.gripper2.position = -grip_cmd;
}

void SimulationMode::armTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
    if (msg->points.empty()) return;

    std::lock_guard<std::mutex> lock(data_mutex_);
    const auto& pt = msg->points.back();

    for (size_t j = 0; j < msg->joint_names.size(); ++j) {
        const std::string& name = msg->joint_names[j];
        double pos = (j < pt.positions.size()) ? pt.positions[j] : 0.0;
        double vel = (j < pt.velocities.size()) ? pt.velocities[j] : 0.0;

        for (size_t i = 0; i < core::NUM_ARM_JOINTS; ++i) {
            if (name == core::ARM_JOINT_NAMES[i]) {
                if (config_.invert_joints[i]) {
                    pos = -pos;
                    vel = -vel;
                }
                predicted_state_buffor_.joints[i].position = pos;
                predicted_state_buffor_.joints[i].velocity = vel;
                break;
            }
        }

        if (name == core::GRIPPER1_JOINT_NAME) {
            if (config_.invert_gripper) {
                pos = -pos;
                vel = -vel;
            }
            predicted_state_buffor_.gripper1.position = pos;
            predicted_state_buffor_.gripper1.velocity = vel;
            predicted_state_buffor_.gripper2.position = -pos;
            predicted_state_buffor_.gripper2.velocity = -vel;
        }
    }
}

void SimulationMode::feedbackCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    for (size_t j = 0; j < msg->name.size(); ++j) {
        const std::string& name = msg->name[j];
        double pos = (j < msg->position.size()) ? msg->position[j] : 0.0;
        double vel = (j < msg->velocity.size()) ? msg->velocity[j] : 0.0;

        for (size_t i = 0; i < core::NUM_ARM_JOINTS; ++i) {
            if (name == core::ARM_JOINT_NAMES[i]) {
                simulated_feedback_buffor_.joints[i].position = pos;
                simulated_feedback_buffor_.joints[i].velocity = vel;
                break;
            }
        }

        if (name == core::GRIPPER1_JOINT_NAME) {
            simulated_feedback_buffor_.gripper1.position = pos;
            simulated_feedback_buffor_.gripper1.velocity = vel;
        } else if (name == core::GRIPPER2_JOINT_NAME) {
            simulated_feedback_buffor_.gripper2.position = pos;
            simulated_feedback_buffor_.gripper2.velocity = vel;
        }
    }
}

void SimulationMode::publishSimFeedback()
{
    rclcpp::Time now = node_->get_clock()->now();

    auto pub = [this, &now](int vesc_id, double erpm, double precise_pos = 0.0) {
        rex_interfaces::msg::VescStatus msg;
        msg.header.stamp = now;
        msg.vesc_id      = vesc_id;
        msg.erpm         = static_cast<int32_t>(erpm);
        msg.precise_pos  = precise_pos;
        vesc_feedback_pub_->publish(msg);
    };

    // Publish fake VescStatus for link1..link6 (VESC IDs 112..117)
    const int vesc_ids[core::NUM_ARM_JOINTS] = {
        core::VESC_ID_LINK1,
        core::VESC_ID_LINK2,
        core::VESC_ID_LINK3,
        core::VESC_ID_LINK4,
        core::VESC_ID_LINK5,
        core::VESC_ID_LINK6
    };

    for (size_t i = 0; i < core::NUM_ARM_JOINTS; ++i) {
        double pos = simulated_feedback_current_.joints[i].position;
        double vel = simulated_feedback_current_.joints[i].velocity;
        double precise_pos = conversions_->radToPrecisePos(pos, config_.gear_ratios[i]);
        double erpm = conversions_->radPerSecToErpm(vel, config_.gear_ratios[i]);
        pub(vesc_ids[i], erpm, precise_pos);
    }

    // Gripper feedback (VESC ID 118)
    double grip_pos = simulated_feedback_current_.gripper1.position;
    double grip_vel = simulated_feedback_current_.gripper1.velocity;
    double grip_precise_pos = conversions_->metersToPrecisePos(grip_pos, config_.gripper_linear_scale);
    double grip_erpm = conversions_->radPerSecToErpm(
        (config_.gripper_linear_scale != 0.0 ? grip_vel / config_.gripper_linear_scale : 0.0),
        config_.gripper_gear_ratio);

    pub(core::VESC_ID_GRIPPER1, grip_erpm, grip_precise_pos);
}

} // namespace bridge_modes
} // namespace rover_robot_arm_kinematics_bridge
