#include "rover_robot_arm_kinematics_bridge/rover_robot_arm_kinematics_bridge.h"
#include "rover_robot_arm_kinematics_bridge/bridge_modes/simulation_mode.hpp"
#include "rover_robot_arm_kinematics_bridge/bridge_modes/digital_shadow_mode.hpp"

namespace rover_robot_arm_kinematics_bridge {

rover_robot_arm_kinematics_bridge::rover_robot_arm_kinematics_bridge(const rclcpp::NodeOptions& options)
    : Node("rover_robot_arm_kinematics_bridge", options)
{
    loadParameters();

    publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_hz_);

    gazebo_pub_ = std::make_shared<gazebo::GazeboArmPublisher>(this);
    conversions_ = std::make_shared<core::Conversions>(
        pole_numbers_, default_gear_ratio_, config_.gripper_linear_scale);

    if (mode_str_ == "simulation") {
        active_mode_ = std::make_unique<bridge_modes::SimulationMode>(
            this, gazebo_pub_, conversions_, config_);
        RCLCPP_INFO(get_logger(),
            "\n[rover_arm_bridge] ========================================\n"
            "                   Mode  : SIMULATION\n"
            "                   Config: pole_numbers=%d | rate=%.0f Hz\n"
            "                   Flow  : arm kinematics -> Gazebo -> fake VescStatus\n"
            "                   ========================================",
            pole_numbers_, publish_rate_hz_);
    } 
    else if (mode_str_ == "digital_shadow") {
        active_mode_ = std::make_unique<bridge_modes::DigitalShadowMode>(
            this, conversions_, config_);
        RCLCPP_INFO(get_logger(),
            "\n[rover_arm_bridge] ========================================\n"
            "                   Mode  : DIGITAL_SHADOW\n"
            "                   Config: pole_numbers=%d | rate=%.0f Hz\n"
            "                   Flow  : Real VescStatus (112-118) -> /joint_states [RViz]\n"
            "                   ========================================",
            pole_numbers_, publish_rate_hz_);
    }
    else {
        RCLCPP_WARN(get_logger(), "[rover_arm_bridge] Unknown mode '%s' — defaulting to 'simulation'.", mode_str_.c_str());
        active_mode_ = std::make_unique<bridge_modes::SimulationMode>(
            this, gazebo_pub_, conversions_, config_);
    }

    timer_ = create_wall_timer(
        publish_period_.to_chrono<std::chrono::milliseconds>(),
        std::bind(&rover_robot_arm_kinematics_bridge::onUpdate, this));
}

void rover_robot_arm_kinematics_bridge::loadParameters()
{
    declare_parameter<std::string>("mode",                 "simulation");
    declare_parameter<int>        ("pole_numbers",          7);
    declare_parameter<double>     ("gear_ratio_arm",        1000.0);
    declare_parameter<double>     ("gear_ratio_gripper",    1000.0);
    declare_parameter<double>     ("gripper_linear_scale",  0.0001);
    declare_parameter<double>     ("publish_rate",          20.0);
    declare_parameter<double>     ("hardware_timeout_sec",  0.5);

    declare_parameter<bool>("invert_link1",   false);
    declare_parameter<bool>("invert_link2",   false);
    declare_parameter<bool>("invert_link3",   false);
    declare_parameter<bool>("invert_link4",   false);
    declare_parameter<bool>("invert_link5",   false);
    declare_parameter<bool>("invert_link6",   false);
    declare_parameter<bool>("invert_gripper", false);

    declare_parameter<double>("gear_ratio_link1", 1000.0);
    declare_parameter<double>("gear_ratio_link2", 1000.0);
    declare_parameter<double>("gear_ratio_link3", 1000.0);
    declare_parameter<double>("gear_ratio_link4", 1000.0);
    declare_parameter<double>("gear_ratio_link5", 1000.0);
    declare_parameter<double>("gear_ratio_link6", 1000.0);

    mode_str_             = get_parameter("mode").as_string();
    pole_numbers_         = get_parameter("pole_numbers").as_int();
    default_gear_ratio_   = get_parameter("gear_ratio_arm").as_double();
    publish_rate_hz_      = get_parameter("publish_rate").as_double();
    hardware_timeout_sec_ = get_parameter("hardware_timeout_sec").as_double();

    config_.pole_numbers         = pole_numbers_;
    config_.publish_rate         = publish_rate_hz_;
    config_.hardware_timeout_sec = hardware_timeout_sec_;
    config_.gripper_gear_ratio   = get_parameter("gear_ratio_gripper").as_double();
    config_.gripper_linear_scale = get_parameter("gripper_linear_scale").as_double();

    config_.invert_joints[0] = get_parameter("invert_link1").as_bool();
    config_.invert_joints[1] = get_parameter("invert_link2").as_bool();
    config_.invert_joints[2] = get_parameter("invert_link3").as_bool();
    config_.invert_joints[3] = get_parameter("invert_link4").as_bool();
    config_.invert_joints[4] = get_parameter("invert_link5").as_bool();
    config_.invert_joints[5] = get_parameter("invert_link6").as_bool();
    config_.invert_gripper   = get_parameter("invert_gripper").as_bool();

    config_.gear_ratios[0] = get_parameter("gear_ratio_link1").as_double();
    config_.gear_ratios[1] = get_parameter("gear_ratio_link2").as_double();
    config_.gear_ratios[2] = get_parameter("gear_ratio_link3").as_double();
    config_.gear_ratios[3] = get_parameter("gear_ratio_link4").as_double();
    config_.gear_ratios[4] = get_parameter("gear_ratio_link5").as_double();
    config_.gear_ratios[5] = get_parameter("gear_ratio_link6").as_double();
}

void rover_robot_arm_kinematics_bridge::onUpdate()
{
    if (active_mode_) {
        active_mode_->update();
    }
}

} // namespace rover_robot_arm_kinematics_bridge
