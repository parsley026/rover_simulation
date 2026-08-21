// =============================================================================
// VescSteerPlugin
//
// Custom Gazebo Harmonic (gz-sim8) plugin that replaces the stock
// gz-sim-joint-position-controller-system for the REX rover steering shafts.
//
// It simulates a position-controlled VESC/servo acting on a mechanical
// steering column. Command input is received via gz-transport (gz.msgs.Double)
// on the same topic that ros_gz_bridge maps from /steer/{module}/cmd_pos, so
// zero changes to rover_kinematics_bridge or _bridge_core.yaml are required.
//
// ----------------------------------------------------------------------------
// Core features (always active):
//   - Position PID → JointForceCmd
//   - max_velocity internal velocity clamp
//   - max_effort clamp
//
// Optional features (toggle in URDF with <enable_*>true</enable_*>):
//   - enable_stiction        : Static friction + positional deadband
//   - enable_slew_rate       : Acceleration rate limiter on velocity command
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
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/transport/Node.hh>

#include "pid_controller.hpp"

namespace rover_description {

class VescSteerPlugin
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
            gzerr << "[VescSteerPlugin] Must be attached to a model.\n";
            return;
        }

        // ---- Required parameters ----
        if (!sdf->HasElement("joint_name") || !sdf->HasElement("cmd_topic")) {
            gzerr << "[VescSteerPlugin] <joint_name> and <cmd_topic> are required.\n";
            return;
        }

        std::string joint_name = sdf->Get<std::string>("joint_name");
        cmd_topic_             = sdf->Get<std::string>("cmd_topic");

        joint_ = model_.JointByName(ecm, joint_name);
        if (joint_ == gz::sim::kNullEntity) {
            gzerr << "[VescSteerPlugin] Joint not found: " << joint_name << "\n";
            return;
        }

        // ---- Core parameters ----
        double p   = sdf->Get<double>("p_gain",       10.0).first;
        double i   = sdf->Get<double>("i_gain",         0.1).first;
        double d   = sdf->Get<double>("d_gain",         0.01).first;
        max_velocity_ = sdf->Get<double>("max_velocity", 0.76).first;
        max_effort_   = sdf->Get<double>("max_effort",  10.0).first;

        pid_.setGains(p, i, d);
        pid_.setIntegralLimits(max_effort_, -max_effort_);

        // ---- Optional: Stiction + deadband ----
        enable_stiction_ = sdf->Get<bool>("enable_stiction", false).first;
        if (enable_stiction_) {
            static_friction_torque_ = sdf->Get<double>("static_friction_torque", 0.5).first;
            deadband_rad_           = sdf->Get<double>("deadband_rad",            0.01).first;
            gzmsg << "[VescSteerPlugin] Stiction enabled (τ_s="
                  << static_friction_torque_ << " deadband=" << deadband_rad_ << " rad)\n";
        }

        // ---- Optional: Slew-rate limiter ----
        enable_slew_rate_ = sdf->Get<bool>("enable_slew_rate", false).first;
        if (enable_slew_rate_) {
            max_acceleration_ = sdf->Get<double>("max_acceleration", 2.0).first;
            gzmsg << "[VescSteerPlugin] Slew rate enabled (α_max="
                  << max_acceleration_ << " rad/s²)\n";
        }

        // ---- Optional: Encoder noise / quantization ----
        enable_encoder_noise_ = sdf->Get<bool>("enable_encoder_noise", false).first;
        if (enable_encoder_noise_) {
            encoder_cpr_   = sdf->Get<double>("encoder_cpr",    4096.0).first;
            noise_std_dev_ = sdf->Get<double>("noise_std_dev",   0.0005).first;
            gzmsg << "[VescSteerPlugin] Encoder noise enabled (CPR="
                  << encoder_cpr_ << " σ=" << noise_std_dev_ << ")\n";
        }

        // ---- Optional: Stall detection ----
        enable_stall_detection_  = sdf->Get<bool>("enable_stall_detection", false).first;
        if (enable_stall_detection_) {
            stall_vel_threshold_ = sdf->Get<double>("stall_velocity_threshold", 0.01).first;
            stall_time_sec_      = sdf->Get<double>("stall_time_sec",            0.5).first;
            fault_topic_         = sdf->Get<std::string>("fault_topic", "/motor_fault").first;
            fault_vesc_id_       = sdf->Get<int>("fault_vesc_id", 0).first;
            fault_pub_ = transport_node_.Advertise<gz::msgs::Int32>(fault_topic_);
            gzmsg << "[VescSteerPlugin] Stall detection enabled (thresh="
                  << stall_vel_threshold_ << " t=" << stall_time_sec_ << "s)\n";
        }

        // ---- ECM component registration ----
        if (!ecm.EntityHasComponentType(joint_, gz::sim::components::JointPosition().TypeId())) {
            ecm.CreateComponent(joint_, gz::sim::components::JointPosition());
        }
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
            &VescSteerPlugin::onCmdPos,
            this);

        gzmsg << "[VescSteerPlugin] Joint '" << joint_name
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

        // ---- Read current position & velocity ----
        auto pos_comp = ecm.Component<gz::sim::components::JointPosition>(joint_);
        auto vel_comp = ecm.Component<gz::sim::components::JointVelocity>(joint_);

        double measured_pos = (pos_comp && !pos_comp->Data().empty())
                              ? pos_comp->Data()[0] : 0.0;
        double measured_vel = (vel_comp && !vel_comp->Data().empty())
                              ? vel_comp->Data()[0] : 0.0;

        // ---- Optional: encoder quantization + noise ----
        if (enable_encoder_noise_) {
            const double rad_per_count = (2.0 * M_PI) / encoder_cpr_;
            measured_pos = std::round(measured_pos / rad_per_count) * rad_per_count;
            measured_pos += noise_dist_(rng_) * noise_std_dev_;
        }

        const double target_pos = target_pos_.load(std::memory_order_relaxed);
        const double error      = target_pos - measured_pos;

        // ---- Optional: stiction deadband ----
        // If the position error is within the deadband, hold — do not move.
        if (enable_stiction_ && std::fabs(error) < deadband_rad_) {
            auto force_comp = ecm.Component<gz::sim::components::JointForceCmd>(joint_);
            if (force_comp) {
                *force_comp = gz::sim::components::JointForceCmd({0.0});
            }
            pid_.reset();
            return;
        }

        // ---- Position PID ----
        double effort = pid_.update(error, dt);

        // ---- Optional: stiction — require minimum breakout force ----
        if (enable_stiction_ && std::fabs(effort) < static_friction_torque_) {
            effort = 0.0;
        }

        // ---- Clamp effort to max ----
        effort = std::clamp(effort, -max_effort_, max_effort_);

        // ---- Optional: slew rate limit ----
        // We limit how quickly the internal velocity integrator changes, which
        // naturally limits the acceleration of the joint.
        if (enable_slew_rate_) {
            const double max_delta_vel = max_acceleration_ * dt;
            // Estimate velocity command from effort sign; constrain change.
            const double desired_vel   = (effort >= 0.0) ?  max_velocity_ : -max_velocity_;
            const double vel_change    = desired_vel - slew_vel_state_;
            slew_vel_state_ += std::clamp(vel_change, -max_delta_vel, max_delta_vel);
            // Scale effort by slew factor so the joint cannot overshoot speed cap.
            const double slew_factor = std::fabs(slew_vel_state_) / max_velocity_;
            effort *= slew_factor;
        }

        // ---- Optional: stall detection ----
        if (enable_stall_detection_) {
            const bool at_max  = std::fabs(effort) >= max_effort_ * 0.98;
            const bool no_move = std::fabs(measured_vel) < stall_vel_threshold_;
            if (at_max && no_move) {
                stall_accum_ += dt;
                if (stall_accum_ >= stall_time_sec_ && !stall_reported_) {
                    gz::msgs::Int32 msg;
                    msg.set_data(fault_vesc_id_);
                    fault_pub_.Publish(msg);
                    stall_reported_ = true;
                    gzwarn << "[VescSteerPlugin] STALL detected on VESC ID "
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
    void onCmdPos(const gz::msgs::Double & msg)
    {
        target_pos_.store(msg.data(), std::memory_order_relaxed);
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
    std::atomic<double> target_pos_{0.0};

    // ---- Core PID ----
    PidController pid_;
    double        max_velocity_{0.76};
    double        max_effort_{10.0};

    // ---- Optional: Stiction ----
    bool   enable_stiction_{false};
    double static_friction_torque_{0.5};
    double deadband_rad_{0.01};

    // ---- Optional: Slew rate ----
    bool   enable_slew_rate_{false};
    double max_acceleration_{2.0};
    double slew_vel_state_{0.0};

    // ---- Optional: Encoder noise ----
    bool   enable_encoder_noise_{false};
    double encoder_cpr_{4096.0};
    double noise_std_dev_{0.0005};
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
    rover_description::VescSteerPlugin,
    gz::sim::System,
    rover_description::VescSteerPlugin::ISystemConfigure,
    rover_description::VescSteerPlugin::ISystemUpdate)

GZ_ADD_PLUGIN_ALIAS(
    rover_description::VescSteerPlugin,
    "vesc_steer_plugin")
