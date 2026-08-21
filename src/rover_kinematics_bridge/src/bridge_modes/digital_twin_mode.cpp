#include "rover_kinematics_bridge/bridge_modes/digital_twin_mode.hpp"
#include "rover_kinematics_bridge/core/vesc_constants.hpp"

namespace rover_kinematics_bridge {
namespace bridge_modes {

DigitalTwinMode::DigitalTwinMode(
    rclcpp::Node* node,
    std::shared_ptr<gazebo::GazeboCommandPublisher> gazebo_pub,
    std::shared_ptr<core::Conversions> conversions,
    double hardware_timeout_sec,
    const BridgeConfig& config)
    : node_(node)
    , gazebo_pub_(gazebo_pub)
    , conversions_(conversions)
    , hardware_timeout_sec_(hardware_timeout_sec)
    , config_(config)
{
    kinematics_sub_ = node_->create_subscription<rex_interfaces::msg::Wheels>(
        "/CAN/TX/set_motor_vel", 10,
        std::bind(&DigitalTwinMode::kinematicsCallback, this, std::placeholders::_1));

    hardware_sub_ = node_->create_subscription<rex_interfaces::msg::VescStatus>(
        "/CAN/RX/vesc_status", rclcpp::QoS(1000).reliable(),
        std::bind(&DigitalTwinMode::hardwareFeedbackCallback, this, std::placeholders::_1));
}

void DigitalTwinMode::update()
{
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        predicted_state_current_ = predicted_state_buffor_;
        measured_state_current_  = measured_state_buffor_;
    }

    const int64_t last_hw_ns  = last_hardware_time_ns_.load(std::memory_order_acquire);
    const int64_t current_ns  = static_cast<int64_t>(node_->get_clock()->now().nanoseconds());
    const int64_t timeout_ns  = static_cast<int64_t>(hardware_timeout_sec_ * 1.0e9);
    const bool    never_rcvd  = (last_hw_ns == 0);
    const bool    hw_stale    = never_rcvd || ((current_ns - last_hw_ns) > timeout_ns);

    if (hw_stale) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
            "[bridge] DIGITAL_TWIN: hardware feedback stale (>%.2f s) — "
            "propagating predicted state as continuity fallback.",
            hardware_timeout_sec_);
        gazebo_pub_->publish(predicted_state_current_);
    } else {
        gazebo_pub_->publish(measured_state_current_);
    }
}

void DigitalTwinMode::kinematicsCallback(const rex_interfaces::msg::Wheels::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    predicted_state_buffor_.front_left_drive.data  = conversions_->erpmToRadPerSec(msg->front_left.drive.set_value);
    predicted_state_buffor_.front_right_drive.data = conversions_->erpmToRadPerSec(msg->front_right.drive.set_value);
    predicted_state_buffor_.rear_right_drive.data  = conversions_->erpmToRadPerSec(msg->rear_right.drive.set_value);
    predicted_state_buffor_.rear_left_drive.data   = conversions_->erpmToRadPerSec(msg->rear_left.drive.set_value);

    predicted_state_buffor_.front_left_steer.data  = msg->front_left.turn.set_value  * M_PI / 180.0 / core::STEER_PRESCALE;
    predicted_state_buffor_.front_right_steer.data = msg->front_right.turn.set_value * M_PI / 180.0 / core::STEER_PRESCALE;
    predicted_state_buffor_.rear_right_steer.data  = msg->rear_right.turn.set_value  * M_PI / 180.0 / core::STEER_PRESCALE;
    predicted_state_buffor_.rear_left_steer.data   = msg->rear_left.turn.set_value   * M_PI / 180.0 / core::STEER_PRESCALE;

    if (config_.invert_left_steering) {
        predicted_state_buffor_.front_left_steer.data = -predicted_state_buffor_.front_left_steer.data;
        predicted_state_buffor_.rear_left_steer.data = -predicted_state_buffor_.rear_left_steer.data;
    }
    if (config_.invert_right_steering) {
        predicted_state_buffor_.front_right_steer.data = -predicted_state_buffor_.front_right_steer.data;
        predicted_state_buffor_.rear_right_steer.data = -predicted_state_buffor_.rear_right_steer.data;
    }
}

void DigitalTwinMode::hardwareFeedbackCallback(const rex_interfaces::msg::VescStatus::SharedPtr msg)
{
    bool known_id = true;

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        switch (msg->vesc_id) {
            case core::FRONT_LEFT_DRIVE:  measured_state_buffor_.front_left_drive.data  = conversions_->erpmToRadPerSec(msg->erpm); break;
            case core::FRONT_RIGHT_DRIVE: measured_state_buffor_.front_right_drive.data = conversions_->erpmToRadPerSec(msg->erpm); break;
            case core::REAR_LEFT_DRIVE:   measured_state_buffor_.rear_left_drive.data   = conversions_->erpmToRadPerSec(msg->erpm); break;
            case core::REAR_RIGHT_DRIVE:  measured_state_buffor_.rear_right_drive.data  = conversions_->erpmToRadPerSec(msg->erpm); break;
            case core::FRONT_LEFT_TURN:   measured_state_buffor_.front_left_steer.data  = msg->precise_pos * M_PI / 180.0 / core::STEER_PRESCALE; break;
            case core::FRONT_RIGHT_TURN:  measured_state_buffor_.front_right_steer.data = msg->precise_pos * M_PI / 180.0 / core::STEER_PRESCALE; break;
            case core::REAR_RIGHT_TURN:   measured_state_buffor_.rear_right_steer.data  = msg->precise_pos * M_PI / 180.0 / core::STEER_PRESCALE; break;
            case core::REAR_LEFT_TURN:    measured_state_buffor_.rear_left_steer.data   = msg->precise_pos * M_PI / 180.0 / core::STEER_PRESCALE; break;
            default:                      known_id = false; break;
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

    if (known_id) {
        last_hardware_time_ns_.store(
            static_cast<int64_t>(node_->get_clock()->now().nanoseconds()),
            std::memory_order_release);
    }
}

} // namespace bridge_modes
} // namespace rover_kinematics_bridge
