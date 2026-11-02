#include "rover_kinematics_bridge/bridge_modes/digital_shadow_mode.hpp"
#include "rover_kinematics_bridge/core/vesc_constants.hpp"

namespace rover_kinematics_bridge {
namespace bridge_modes {

DigitalShadowMode::DigitalShadowMode(
    rclcpp::Node* node,
    std::shared_ptr<core::Conversions> conversions,
    const BridgeConfig& config)
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
    
    // Active steering joints
    msg.name.push_back("steer_front_left_joint");
    msg.position.push_back(measured_state_current_.front_left_steer.data);
    msg.name.push_back("steer_front_right_joint");
    msg.position.push_back(measured_state_current_.front_right_steer.data);
    msg.name.push_back("steer_rear_left_joint");
    msg.position.push_back(measured_state_current_.rear_left_steer.data);
    msg.name.push_back("steer_rear_right_joint");
    msg.position.push_back(measured_state_current_.rear_right_steer.data);

    // Active drive joints
    msg.name.push_back("drive_front_left_joint");
    msg.velocity.push_back(measured_state_current_.front_left_drive.data);
    msg.name.push_back("drive_front_right_joint");
    msg.velocity.push_back(measured_state_current_.front_right_drive.data);
    msg.name.push_back("drive_rear_left_joint");
    msg.velocity.push_back(measured_state_current_.rear_left_drive.data);
    msg.name.push_back("drive_rear_right_joint");
    msg.velocity.push_back(measured_state_current_.rear_right_drive.data);
    
    // Fill in drive positions (since they are continuous, we can just supply 0.0 if we don't track them)
    // and velocities for steering if needed, but RViz mainly needs position.
    // Ensure position array matches name array for active joints that we care about position.
    for (int i = 0; i < 4; ++i) msg.velocity.insert(msg.velocity.begin(), 0.0); // 0 vel for steering
    for (int i = 0; i < 4; ++i) msg.position.push_back(0.0); // 0 pos for drive wheels
    
    // Passive suspension joints (locked at 0.0)
    const std::vector<std::string> passive_joints = {
        "swivel_left_joint", "swivel_right_joint",
        "arm_front_left_pivot", "arm_rear_left_pivot", 
        "arm_front_right_pivot", "arm_rear_right_pivot"
    };
    
    for (const auto& joint : passive_joints) {
        msg.name.push_back(joint);
        msg.position.push_back(0.0);
        msg.velocity.push_back(0.0);
    }
    
    joint_state_pub_->publish(msg);
}

void DigitalShadowMode::hardwareFeedbackCallback(const rex_interfaces::msg::VescStatus::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    switch (msg->vesc_id) {
        case core::FRONT_LEFT_DRIVE:
            measured_state_buffor_.front_left_drive.data  = conversions_->erpmToRadPerSec(msg->erpm); break;
        case core::FRONT_RIGHT_DRIVE:
            measured_state_buffor_.front_right_drive.data = conversions_->erpmToRadPerSec(msg->erpm); break;
        case core::REAR_LEFT_DRIVE:
            measured_state_buffor_.rear_left_drive.data   = conversions_->erpmToRadPerSec(msg->erpm); break;
        case core::REAR_RIGHT_DRIVE:
            measured_state_buffor_.rear_right_drive.data  = conversions_->erpmToRadPerSec(msg->erpm); break;

        case core::FRONT_LEFT_TURN:
            measured_state_buffor_.front_left_steer.data  = msg->precise_pos * M_PI / 180.0 / core::STEER_PRESCALE; break;
        case core::FRONT_RIGHT_TURN:
            measured_state_buffor_.front_right_steer.data = msg->precise_pos * M_PI / 180.0 / core::STEER_PRESCALE; break;
        case core::REAR_RIGHT_TURN:
            measured_state_buffor_.rear_right_steer.data  = msg->precise_pos * M_PI / 180.0 / core::STEER_PRESCALE; break;
        case core::REAR_LEFT_TURN:
            measured_state_buffor_.rear_left_steer.data   = msg->precise_pos * M_PI / 180.0 / core::STEER_PRESCALE; break;
        default:
            break;
    }
    
    if (config_.invert_steering_fl && msg->vesc_id == core::FRONT_LEFT_TURN)  measured_state_buffor_.front_left_steer.data = -measured_state_buffor_.front_left_steer.data;
    if (config_.invert_steering_fr && msg->vesc_id == core::FRONT_RIGHT_TURN) measured_state_buffor_.front_right_steer.data = -measured_state_buffor_.front_right_steer.data;
    if (config_.invert_steering_rl && msg->vesc_id == core::REAR_LEFT_TURN)   measured_state_buffor_.rear_left_steer.data = -measured_state_buffor_.rear_left_steer.data;
    if (config_.invert_steering_rr && msg->vesc_id == core::REAR_RIGHT_TURN)  measured_state_buffor_.rear_right_steer.data = -measured_state_buffor_.rear_right_steer.data;

    if (config_.invert_drive_fl && msg->vesc_id == core::FRONT_LEFT_DRIVE)  measured_state_buffor_.front_left_drive.data = -measured_state_buffor_.front_left_drive.data;
    if (config_.invert_drive_fr && msg->vesc_id == core::FRONT_RIGHT_DRIVE) measured_state_buffor_.front_right_drive.data = -measured_state_buffor_.front_right_drive.data;
    if (config_.invert_drive_rl && msg->vesc_id == core::REAR_LEFT_DRIVE)   measured_state_buffor_.rear_left_drive.data = -measured_state_buffor_.rear_left_drive.data;
    if (config_.invert_drive_rr && msg->vesc_id == core::REAR_RIGHT_DRIVE)  measured_state_buffor_.rear_right_drive.data = -measured_state_buffor_.rear_right_drive.data;
}

} // namespace bridge_modes
} // namespace rover_kinematics_bridge
