#include "rover_robot_arm_kinematics_bridge/bridge_modes/digital_shadow_mode.hpp"
#include "rover_robot_arm_kinematics_bridge/core/arm_constants.hpp"

namespace rover_robot_arm_kinematics_bridge {
namespace bridge_modes {

DigitalShadowMode::DigitalShadowMode(
    rclcpp::Node* node,
    std::shared_ptr<core::Conversions> conversions,
    const ArmBridgeConfig& config)
    : node_(node)
    , conversions_(conversions)
    , config_(config)
{
    hardware_sub_ = node_->create_subscription<rex_interfaces::msg::VescStatus>(
        "/CAN/RX/vesc_status", rclcpp::QoS(1000).reliable(),
        std::bind(&DigitalShadowMode::hardwareFeedbackCallback, this, std::placeholders::_1));

    joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::QoS(10));
}

void DigitalShadowMode::update()
{
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        measured_state_current_ = measured_state_buffor_;
    }

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = node_->now();

    // 6 Arm joints
    for (size_t i = 0; i < core::NUM_ARM_JOINTS; ++i) {
        msg.name.push_back(core::ARM_JOINT_NAMES[i]);
        msg.position.push_back(measured_state_current_.joints[i].position);
        msg.velocity.push_back(measured_state_current_.joints[i].velocity);
    }

    // Gripper joints
    msg.name.push_back(core::GRIPPER1_JOINT_NAME);
    msg.position.push_back(measured_state_current_.gripper1.position);
    msg.velocity.push_back(measured_state_current_.gripper1.velocity);

    msg.name.push_back(core::GRIPPER2_JOINT_NAME);
    msg.position.push_back(measured_state_current_.gripper2.position);
    msg.velocity.push_back(measured_state_current_.gripper2.velocity);

    joint_state_pub_->publish(msg);
}

void DigitalShadowMode::hardwareFeedbackCallback(const rex_interfaces::msg::VescStatus::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    switch (msg->vesc_id) {
        case core::VESC_ID_LINK1: {
            double pos = conversions_->precisePosToRad(msg->precise_pos, config_.gear_ratios[0]);
            double vel = conversions_->erpmToRadPerSec(msg->erpm, config_.gear_ratios[0]);
            if (config_.invert_joints[0]) { pos = -pos; vel = -vel; }
            measured_state_buffor_.joints[0].position = pos;
            measured_state_buffor_.joints[0].velocity = vel;
            break;
        }
        case core::VESC_ID_LINK2: {
            double pos = conversions_->precisePosToRad(msg->precise_pos, config_.gear_ratios[1]);
            double vel = conversions_->erpmToRadPerSec(msg->erpm, config_.gear_ratios[1]);
            if (config_.invert_joints[1]) { pos = -pos; vel = -vel; }
            measured_state_buffor_.joints[1].position = pos;
            measured_state_buffor_.joints[1].velocity = vel;
            break;
        }
        case core::VESC_ID_LINK3: {
            double pos = conversions_->precisePosToRad(msg->precise_pos, config_.gear_ratios[2]);
            double vel = conversions_->erpmToRadPerSec(msg->erpm, config_.gear_ratios[2]);
            if (config_.invert_joints[2]) { pos = -pos; vel = -vel; }
            measured_state_buffor_.joints[2].position = pos;
            measured_state_buffor_.joints[2].velocity = vel;
            break;
        }
        case core::VESC_ID_LINK4: {
            double pos = conversions_->precisePosToRad(msg->precise_pos, config_.gear_ratios[3]);
            double vel = conversions_->erpmToRadPerSec(msg->erpm, config_.gear_ratios[3]);
            if (config_.invert_joints[3]) { pos = -pos; vel = -vel; }
            measured_state_buffor_.joints[3].position = pos;
            measured_state_buffor_.joints[3].velocity = vel;
            break;
        }
        case core::VESC_ID_LINK5: {
            double pos = conversions_->precisePosToRad(msg->precise_pos, config_.gear_ratios[4]);
            double vel = conversions_->erpmToRadPerSec(msg->erpm, config_.gear_ratios[4]);
            if (config_.invert_joints[4]) { pos = -pos; vel = -vel; }
            measured_state_buffor_.joints[4].position = pos;
            measured_state_buffor_.joints[4].velocity = vel;
            break;
        }
        case core::VESC_ID_LINK6: {
            double pos = conversions_->precisePosToRad(msg->precise_pos, config_.gear_ratios[5]);
            double vel = conversions_->erpmToRadPerSec(msg->erpm, config_.gear_ratios[5]);
            if (config_.invert_joints[5]) { pos = -pos; vel = -vel; }
            measured_state_buffor_.joints[5].position = pos;
            measured_state_buffor_.joints[5].velocity = vel;
            break;
        }
        case core::VESC_ID_GRIPPER1: {
            double pos = conversions_->precisePosToMeters(msg->precise_pos, config_.gripper_linear_scale);
            double vel = conversions_->erpmToRadPerSec(msg->erpm, config_.gripper_gear_ratio) * config_.gripper_linear_scale;
            if (config_.invert_gripper) { pos = -pos; vel = -vel; }
            measured_state_buffor_.gripper1.position = pos;
            measured_state_buffor_.gripper1.velocity = vel;
            measured_state_buffor_.gripper2.position = -pos;
            measured_state_buffor_.gripper2.velocity = -vel;
            break;
        }
        default:
            break;
    }
}

} // namespace bridge_modes
} // namespace rover_robot_arm_kinematics_bridge
