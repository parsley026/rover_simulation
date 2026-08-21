// =============================================================================
// VescDrivePlugin
//
// Custom Gazebo Harmonic (gz-sim8) plugin that replaces the stock
// gz-sim-joint-controller-system for the REX rover drive wheels.
//
// It simulates a VESC-driven brushless motor in velocity-control mode.
// Command input is received via gz-transport (gz.msgs.Double) on the same
// topic that ros_gz_bridge maps from /wheel/{module}/cmd_vel, so zero changes
// to rover_kinematics_bridge or _bridge_core.yaml are required.
//
// ----------------------------------------------------------------------------
// Core features (always active):
//   - Velocity PID → JointForceCmd
//   - max_effort clamp
//
// Optional features (toggle in URDF with <enable_*>true</enable_*>):
//   - enable_torque_curve    : Back-EMF torque de-rating at high speed
//   - enable_current_limits  : VESC amperage cap
//   - enable_encoder_noise   : Quantization + Gaussian noise on measurement
//   - enable_stall_detection : Publishes /motor_fault (gz.msgs.Int32) on stall
// =============================================================================

#include <algorithm>
#include <atomic>
#include <cmath>
#include <mutex>
#include <random>
#include <string>

#include <gz/common/Console.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/int32.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/transport/Node.hh>

#include "pid_controller.hpp"

namespace rover_description {

class VescDrivePlugin
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemUpdate
{
public:
    // =========================================================================
    // Configure — read SDF, wire up transport, initialise ECM components
    // =========================================================================
    void Configure(
        const gz::sim::Entity          & entity,
        const std::shared_ptr<const sdf::Element> & sdf,
        gz::sim::EntityComponentManager & ecm,
        gz::sim::EventManager          & /*eventMgr*/) override
    {
        model_ = gz::sim::Model(entity);
        if (!model_.Valid(ecm)) {
            gzerr << "[VescDrivePlugin] Must be attached to a model.\n";
            return;
        }

        // ---- Required parameters ----
        if (!sdf->HasElement("joint_name") || !sdf->HasElement("cmd_topic")) {
            gzerr << "[VescDrivePlugin] <joint_name> and <cmd_topic> are required.\n";
            return;
        }

        std::string joint_name = sdf->Get<std::string>("joint_name");
        cmd_topic_             = sdf->Get<std::string>("cmd_topic");

        joint_ = model_.JointByName(ecm, joint_name);
        if (joint_ == gz::sim::kNullEntity) {
            gzerr << "[VescDrivePlugin] Joint not found: " << joint_name << "\n";
            return;
        }

        // ---- Core parameters ----
        double p   = sdf->Get<double>("p_gain",    0.45).first;
        double i   = sdf->Get<double>("i_gain",    0.35).first;
        double d   = sdf->Get<double>("d_gain",    0.0001).first;
        max_effort_ = sdf->Get<double>("max_effort", 45.0).first;

        pid_.setGains(p, i, d);
        pid_.setIntegralLimits(max_effort_, -max_effort_);

        // ---- Optional: Torque curve (Back-EMF de-rating) ----
        enable_torque_curve_ = sdf->Get<bool>("enable_torque_curve", false).first;
        if (enable_torque_curve_) {
            nominal_voltage_  = sdf->Get<double>("nominal_voltage",  22.2).first;
            kv_rating_        = sdf->Get<double>("kv_rating",        300.0).first;
            motor_resistance_ = sdf->Get<double>("motor_resistance",  0.05).first;
            gzmsg << "[VescDrivePlugin] Torque curve enabled (V=" << nominal_voltage_
                  << " Kv=" << kv_rating_ << " R=" << motor_resistance_ << ")\n";
        }

        // ---- Optional: Current limits ----
        enable_current_limits_  = sdf->Get<bool>("enable_current_limits", false).first;
        if (enable_current_limits_) {
            max_motor_current_amp_ = sdf->Get<double>("max_motor_current_amp", 40.0).first;
            torque_constant_       = sdf->Get<double>("torque_constant",        0.05).first;
            gzmsg << "[VescDrivePlugin] Current limits enabled (Imax="
                  << max_motor_current_amp_ << "A Kt=" << torque_constant_ << ")\n";
        }

        // ---- Optional: Encoder noise / quantization ----
        enable_encoder_noise_ = sdf->Get<bool>("enable_encoder_noise", false).first;
        if (enable_encoder_noise_) {
            encoder_cpr_   = sdf->Get<double>("encoder_cpr",    4096.0).first;
            noise_std_dev_ = sdf->Get<double>("noise_std_dev",   0.001).first;
            gzmsg << "[VescDrivePlugin] Encoder noise enabled (CPR="
                  << encoder_cpr_ << " σ=" << noise_std_dev_ << ")\n";
        }

        // ---- Optional: Stall detection ----
        enable_stall_detection_   = sdf->Get<bool>("enable_stall_detection", false).first;
        if (enable_stall_detection_) {
            stall_vel_threshold_ = sdf->Get<double>("stall_velocity_threshold", 0.01).first;
            stall_time_sec_      = sdf->Get<double>("stall_time_sec",            0.5).first;
            fault_topic_         = sdf->Get<std::string>("fault_topic", "/motor_fault").first;
            fault_vesc_id_       = sdf->Get<int>("fault_vesc_id", 0).first;
            fault_pub_ = transport_node_.Advertise<gz::msgs::Int32>(fault_topic_);
            gzmsg << "[VescDrivePlugin] Stall detection enabled (thresh="
                  << stall_vel_threshold_ << " t=" << stall_time_sec_ << "s)\n";
        }

        // ---- ECM component registration ----
        if (!ecm.EntityHasComponentType(joint_, gz::sim::components::JointVelocity().TypeId())) {
            ecm.CreateComponent(joint_, gz::sim::components::JointVelocity());
        }
        if (!ecm.EntityHasComponentType(joint_, gz::sim::components::JointForceCmd().TypeId())) {
            ecm.CreateComponent(joint_, gz::sim::components::JointForceCmd({0.0}));
        }

        // ---- gz-transport command subscription ----
        // Topic carries gz.msgs.Double — same type that ros_gz_bridge converts
        // from std_msgs/Float64 published by rover_kinematics_bridge.
        transport_node_.Subscribe(
            cmd_topic_,
            &VescDrivePlugin::onCmdVel,
            this);

        gzmsg << "[VescDrivePlugin] Joint '" << joint_name
              << "' listening on gz-transport: " << cmd_topic_ << "\n";

        configured_ = true;
    }

    // =========================================================================
    // Update — runs every physics step
    // =========================================================================
    void Update(
        const gz::sim::UpdateInfo          & info,
        gz::sim::EntityComponentManager    & ecm) override
    {
        if (!configured_ || info.paused) { return; }

        const double dt = std::chrono::duration<double>(info.dt).count();
        if (dt <= 0.0) { return; }

        // ---- Read current velocity ----
        auto vel_comp = ecm.Component<gz::sim::components::JointVelocity>(joint_);
        double measured_vel = (vel_comp && !vel_comp->Data().empty())
                              ? vel_comp->Data()[0] : 0.0;

        // ---- Optional: encoder quantization + noise ----
        if (enable_encoder_noise_) {
            // Quantise to encoder resolution
            const double rad_per_count = (2.0 * M_PI) / encoder_cpr_;
            measured_vel = std::round(measured_vel / rad_per_count) * rad_per_count;
            // Add Gaussian noise
            measured_vel += noise_dist_(rng_) * noise_std_dev_;
        }

        // ---- PID ----
        const double target_vel = target_vel_.load(std::memory_order_relaxed);
        const double error      = target_vel - measured_vel;
        double effort           = pid_.update(error, dt);

        // ---- Optional: Back-EMF torque curve ----
        double dynamic_max_effort = max_effort_;
        if (enable_torque_curve_) {
            // Available torque decreases linearly with speed (simplified BLDC model).
            // Torque_max = (V_nom - |ω| * 30/(π * Kv)) / R * Kt
            // Here we compute a normalised de-rating factor on max_effort_.
            const double back_emf   = std::fabs(measured_vel) * 30.0 / (M_PI * kv_rating_);
            const double v_effective = std::max(0.0, nominal_voltage_ - back_emf);
            const double derate      = v_effective / nominal_voltage_;
            dynamic_max_effort       = max_effort_ * derate;
        }

        // ---- Optional: Current (amperage) limit ----
        if (enable_current_limits_) {
            const double max_effort_from_amps = max_motor_current_amp_ * torque_constant_;
            dynamic_max_effort = std::min(dynamic_max_effort, max_effort_from_amps);
        }

        // ---- Clamp final effort ----
        effort = std::clamp(effort, -dynamic_max_effort, dynamic_max_effort);

        // ---- Optional: stall detection ----
        if (enable_stall_detection_) {
            const bool at_max  = std::fabs(effort) >= dynamic_max_effort * 0.98;
            const bool no_move = std::fabs(measured_vel) < stall_vel_threshold_;
            if (at_max && no_move) {
                stall_accum_ += dt;
                if (stall_accum_ >= stall_time_sec_ && !stall_reported_) {
                    gz::msgs::Int32 msg;
                    msg.set_data(fault_vesc_id_);
                    fault_pub_.Publish(msg);
                    stall_reported_ = true;
                    gzwarn << "[VescDrivePlugin] STALL detected on VESC ID "
                           << fault_vesc_id_ << " — published to " << fault_topic_ << "\n";
                }
            } else {
                stall_accum_    = 0.0;
                stall_reported_ = false;
            }
        }

        // ---- Write force command to ECM ----
        auto force_comp = ecm.Component<gz::sim::components::JointForceCmd>(joint_);
        if (force_comp) {
            *force_comp = gz::sim::components::JointForceCmd({effort});
        }
    }

private:
    // ---- gz-transport callback (called from transport thread) ----
    void onCmdVel(const gz::msgs::Double & msg)
    {
        target_vel_.store(msg.data(), std::memory_order_relaxed);
    }

    // ---- State ----
    gz::sim::Model  model_{gz::sim::kNullEntity};
    gz::sim::Entity joint_{gz::sim::kNullEntity};
    bool            configured_{false};
    std::string     cmd_topic_;

    // ---- Transport ----
    gz::transport::Node               transport_node_;
    gz::transport::Node::Publisher    fault_pub_;
    std::string                       fault_topic_{"/motor_fault"};
    int                               fault_vesc_id_{0};

    // ---- Command (written from transport callback, read in Update) ----
    std::atomic<double> target_vel_{0.0};

    // ---- Core PID ----
    PidController pid_;
    double        max_effort_{45.0};

    // ---- Optional: Torque curve ----
    bool   enable_torque_curve_{false};
    double nominal_voltage_{22.2};
    double kv_rating_{300.0};
    double motor_resistance_{0.05};

    // ---- Optional: Current limits ----
    bool   enable_current_limits_{false};
    double max_motor_current_amp_{40.0};
    double torque_constant_{0.05};

    // ---- Optional: Encoder noise ----
    bool   enable_encoder_noise_{false};
    double encoder_cpr_{4096.0};
    double noise_std_dev_{0.001};
    std::default_random_engine               rng_{std::random_device{}()};
    std::normal_distribution<double>         noise_dist_{0.0, 1.0};

    // ---- Optional: Stall detection ----
    bool   enable_stall_detection_{false};
    double stall_vel_threshold_{0.01};
    double stall_time_sec_{0.5};
    double stall_accum_{0.0};
    bool   stall_reported_{false};
};

} // namespace rover_description

// ---------------------------------------------------------------------------
// Plugin registration
// ---------------------------------------------------------------------------
GZ_ADD_PLUGIN(
    rover_description::VescDrivePlugin,
    gz::sim::System,
    rover_description::VescDrivePlugin::ISystemConfigure,
    rover_description::VescDrivePlugin::ISystemUpdate)

GZ_ADD_PLUGIN_ALIAS(
    rover_description::VescDrivePlugin,
    "vesc_drive_plugin")
