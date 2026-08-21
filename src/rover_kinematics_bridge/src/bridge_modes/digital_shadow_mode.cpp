#include "rover_kinematics_bridge/bridge_modes/digital_shadow_mode.hpp"
#include "rover_kinematics_bridge/core/vesc_constants.hpp"

namespace rover_kinematics_bridge {
namespace bridge_modes {

DigitalShadowMode::DigitalShadowMode(
    rclcpp::Node* node,
    std::shared_ptr<gazebo::GazeboCommandPublisher> gazebo_pub,
    std::shared_ptr<core::Conversions> conversions,
    const BridgeConfig& config)
    : node_(node)
    , gazebo_pub_(gazebo_pub)
    , conversions_(conversions)
    , config_(config)
{
    hardware_sub_ = node_->create_subscription<rex_interfaces::msg::VescStatus>(
        "/CAN/RX/vesc_status", rclcpp::QoS(1000).reliable(),
        std::bind(&DigitalShadowMode::hardwareFeedbackCallback, this, std::placeholders::_1));
}

void DigitalShadowMode::update()
{
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        measured_state_current_ = measured_state_buffor_;
    }
    gazebo_pub_->publish(measured_state_current_);
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
    
    if (config_.invert_left_steering) {
        if (msg->vesc_id == core::FRONT_LEFT_TURN)  measured_state_buffor_.front_left_steer.data = -measured_state_buffor_.front_left_steer.data;
        if (msg->vesc_id == core::REAR_LEFT_TURN)   measured_state_buffor_.rear_left_steer.data = -measured_state_buffor_.rear_left_steer.data;
    }
    if (config_.invert_right_steering) {
        if (msg->vesc_id == core::FRONT_RIGHT_TURN) measured_state_buffor_.front_right_steer.data = -measured_state_buffor_.front_right_steer.data;
        if (msg->vesc_id == core::REAR_RIGHT_TURN)  measured_state_buffor_.rear_right_steer.data = -measured_state_buffor_.rear_right_steer.data;
    }
}

} // namespace bridge_modes
} // namespace rover_kinematics_bridge
