#include "rover_kinematics_bridge/rover_kinematics_bridge.h"
#include "rover_kinematics_bridge/bridge_modes/simulation_mode.hpp"
#include "rover_kinematics_bridge/bridge_modes/digital_shadow_mode.hpp"
#include "rover_kinematics_bridge/bridge_modes/digital_twin_mode.hpp"

namespace rover_kinematics_bridge {

rover_kinematics_bridge::rover_kinematics_bridge(const rclcpp::NodeOptions& options)
    : Node("rover_kinematics_bridge", options)
{
    loadParameters();

    publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_hz_);

    gazebo_pub_ = std::make_shared<gazebo::GazeboCommandPublisher>(this);
    conversions_ = std::make_shared<core::Conversions>(pole_numbers_, motor_gear_ratio_);

    if (mode_str_ == "simulation") {
        active_mode_ = std::make_unique<bridge_modes::SimulationMode>(this, gazebo_pub_, conversions_, config_);
        RCLCPP_INFO(get_logger(),
            "\n[bridge] ========================================\n"
            "         Mode  : SIMULATION\n"
            "         Config: pole_numbers=%d | rate=%.0f Hz\n"
            "         Flow  : rover_kinematics -> Gazebo -> fake VescStatus\n"
            "         ========================================",
            pole_numbers_, publish_rate_hz_);
    } 
    else if (mode_str_ == "digital_shadow") {
        active_mode_ = std::make_unique<bridge_modes::DigitalShadowMode>(this, gazebo_pub_, conversions_, config_);
        RCLCPP_INFO(get_logger(),
            "\n[bridge] ========================================\n"
            "         Mode  : DIGITAL_SHADOW\n"
            "         Config: pole_numbers=%d | rate=%.0f Hz\n"
            "         Flow  : Real VescStatus -> Gazebo  [one-way observer]\n"
            "         ========================================",
            pole_numbers_, publish_rate_hz_);
    } 
    else if (mode_str_ == "digital_twin") {
        active_mode_ = std::make_unique<bridge_modes::DigitalTwinMode>(this, gazebo_pub_, conversions_, hardware_timeout_sec_, config_);
        RCLCPP_INFO(get_logger(),
            "\n[bridge] ========================================\n"
            "         Mode  : DIGITAL_TWIN\n"
            "         Config: pole_numbers=%d | rate=%.0f Hz | hw_timeout=%.2f s\n"
            "         Flow  : Measured state (HW priority) -> Gazebo\n"
            "                 Predicted state (fallback)   -> Gazebo\n"
            "         ========================================",
            pole_numbers_, publish_rate_hz_, hardware_timeout_sec_);
    } 
    else {
        RCLCPP_WARN(get_logger(), "[bridge] Unknown mode '%s' — defaulting to 'simulation'.", mode_str_.c_str());
        active_mode_ = std::make_unique<bridge_modes::SimulationMode>(this, gazebo_pub_, conversions_, config_);
    }

    timer_ = create_wall_timer(
        publish_period_.to_chrono<std::chrono::milliseconds>(),
        std::bind(&rover_kinematics_bridge::onUpdate, this));
}

void rover_kinematics_bridge::loadParameters()
{
    declare_parameter<std::string>("mode",                "simulation");
    declare_parameter<int>        ("pole_numbers",         7);
    declare_parameter<double>     ("motor_gear_ratio",     31.0);
    declare_parameter<double>     ("publish_rate",         20.0);
    declare_parameter<double>     ("hardware_timeout_sec", 0.5);
    declare_parameter<bool>       ("invert_left_steering",  true);
    declare_parameter<bool>       ("invert_right_steering", true);

    mode_str_             = get_parameter("mode").as_string();
    pole_numbers_         = get_parameter("pole_numbers").as_int();
    motor_gear_ratio_     = get_parameter("motor_gear_ratio").as_double();
    publish_rate_hz_      = get_parameter("publish_rate").as_double();
    hardware_timeout_sec_ = get_parameter("hardware_timeout_sec").as_double();
    config_.invert_left_steering  = get_parameter("invert_left_steering").as_bool();
    config_.invert_right_steering = get_parameter("invert_right_steering").as_bool();
}

void rover_kinematics_bridge::onUpdate()
{
    if (active_mode_) {
        active_mode_->update();
    }
}

} // namespace rover_kinematics_bridge
