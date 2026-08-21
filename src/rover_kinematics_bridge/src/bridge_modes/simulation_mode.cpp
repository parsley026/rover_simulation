#include "rover_kinematics_bridge/bridge_modes/simulation_mode.hpp"
#include "rover_kinematics_bridge/core/vesc_constants.hpp"
#include <functional>
#include <unordered_map>

namespace rover_kinematics_bridge {
namespace bridge_modes {

SimulationMode::SimulationMode(
    rclcpp::Node* node,
    std::shared_ptr<gazebo::GazeboCommandPublisher> gazebo_pub,
    std::shared_ptr<core::Conversions> conversions,
    const BridgeConfig& config)
    : node_(node)
    , gazebo_pub_(gazebo_pub)
    , conversions_(conversions)
    , config_(config)
{
    kinematics_sub_ = node_->create_subscription<rex_interfaces::msg::Wheels>(
        "/CAN/TX/set_motor_vel", 10,
        std::bind(&SimulationMode::kinematicsCallback, this, std::placeholders::_1));

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

    if (node_->count_publishers("/CAN/TX/set_motor_vel") == 0) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
            "[bridge] No publishers on /CAN/TX/set_motor_vel — zeroing drive commands.");
        predicted_state_current_.front_left_drive.data  = 0.0;
        predicted_state_current_.front_right_drive.data = 0.0;
        predicted_state_current_.rear_right_drive.data  = 0.0;
        predicted_state_current_.rear_left_drive.data   = 0.0;
    }

    publishSimFeedback();
    gazebo_pub_->publish(predicted_state_current_);
}

void SimulationMode::kinematicsCallback(const rex_interfaces::msg::Wheels::SharedPtr msg)
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

void SimulationMode::feedbackCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    using SetterFn = std::function<void(SimulationMode*, double, double)>;
    static const std::unordered_map<std::string, SetterFn> joint_map = {
        {"arm_front_left_to_steer_front_left",   [](SimulationMode* s, double p, double){ s->simulated_feedback_buffor_.front_left_steer.data  = p; }},
        {"arm_front_right_to_steer_front_right", [](SimulationMode* s, double p, double){ s->simulated_feedback_buffor_.front_right_steer.data = p; }},
        {"arm_rear_right_to_steer_rear_right",   [](SimulationMode* s, double p, double){ s->simulated_feedback_buffor_.rear_right_steer.data  = p; }},
        {"arm_rear_left_to_steer_rear_left",     [](SimulationMode* s, double p, double){ s->simulated_feedback_buffor_.rear_left_steer.data   = p; }},
        {"steer_front_left_to_wheel_front_left",   [](SimulationMode* s, double, double v){ s->simulated_feedback_buffor_.front_left_drive.data  = v; }},
        {"steer_front_right_to_wheel_front_right", [](SimulationMode* s, double, double v){ s->simulated_feedback_buffor_.front_right_drive.data = v; }},
        {"steer_rear_right_to_wheel_rear_right",   [](SimulationMode* s, double, double v){ s->simulated_feedback_buffor_.rear_right_drive.data  = v; }},
        {"steer_rear_left_to_wheel_rear_left",     [](SimulationMode* s, double, double v){ s->simulated_feedback_buffor_.rear_left_drive.data   = v; }},
    };

    for (std::size_t i = 0; i < msg->name.size(); ++i) {
        auto it = joint_map.find(msg->name[i]);
        if (it != joint_map.end()) {
            double pos = (i < msg->position.size()) ? msg->position[i] : 0.0;
            double vel = (i < msg->velocity.size()) ? msg->velocity[i] : 0.0;
            it->second(this, pos, vel);
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
        msg.erpm         = erpm;
        msg.precise_pos  = precise_pos;
        vesc_feedback_pub_->publish(msg);
    };

    pub(core::REAR_LEFT_DRIVE,   conversions_->radPerSecToErpm(simulated_feedback_current_.rear_left_drive.data));
    pub(core::REAR_RIGHT_DRIVE,  conversions_->radPerSecToErpm(simulated_feedback_current_.rear_right_drive.data));
    pub(core::FRONT_LEFT_DRIVE,  conversions_->radPerSecToErpm(simulated_feedback_current_.front_left_drive.data));
    pub(core::FRONT_RIGHT_DRIVE, conversions_->radPerSecToErpm(simulated_feedback_current_.front_right_drive.data));

    // Gazebo joint_states positions are already in the physical (uninverted) frame.
    // rover_kinematics applies its own inversion internally when reading VescStatus,
    // so we must NOT invert here or the polarity will be applied twice.
    const double fl_steer = simulated_feedback_current_.front_left_steer.data;
    const double fr_steer = simulated_feedback_current_.front_right_steer.data;
    const double rl_steer = simulated_feedback_current_.rear_left_steer.data;
    const double rr_steer = simulated_feedback_current_.rear_right_steer.data;

    pub(core::REAR_LEFT_TURN,   0.0, rl_steer * 180.0 / M_PI * core::STEER_PRESCALE);
    pub(core::REAR_RIGHT_TURN,  0.0, rr_steer * 180.0 / M_PI * core::STEER_PRESCALE);
    pub(core::FRONT_LEFT_TURN,  0.0, fl_steer * 180.0 / M_PI * core::STEER_PRESCALE);
    pub(core::FRONT_RIGHT_TURN, 0.0, fr_steer * 180.0 / M_PI * core::STEER_PRESCALE);
}

} // namespace bridge_modes
} // namespace rover_kinematics_bridge
