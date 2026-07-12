/**
 * @file RoverConfig.hpp
 * @brief Runtime configuration container for the quad rover kinematics package.
 */

#pragma once

#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

/**
 * @struct RoverConfig
 * @brief Single-source-of-truth runtime configuration for the kinematics stack.
 *
 * @details
 * RoverConfig holds all tunable parameters describing the physical
 * and operational characteristics of the rover used by `InverseKinematics`,
 * `ForwardKinematics`, and `HardwareInterface`.
 *
 * Responsibilities:
 * - Provide safe defaults for development and simulation.
 * - Load and validate parameters from a YAML file.
 * - Be immutable after a successful `loadFromFile()` call to avoid
 *   dynamic reconfiguration races between components.
 *
 * Units and frames:
 * - Lengths: meters (m)
 * - Angles: radians (rad)
 * - Speeds: meters/sec (m/s)
 * - ERPM/drive set values: hardware-specific unit (ERPM)
 * - Frames: `base_frame_id_` (robot base), `odom_frame_id_` (odometry)
 *
 * Thread-safety: the struct is intended to be copied into components at
 * initialization; after `initialized_` becomes true callers should treat
 * the object as effectively immutable. Loading should happen before the
 * ROS node spins to avoid races.
 */
struct RoverConfig {
    /**
     * @brief Default constructor that populates safe defaults.
     */
    RoverConfig() { resetDefaults(); }

    /**
     * @brief Resets all configuration fields to conservative, simulation-friendly defaults.
     *
     * Sets the configuration to an uninitialized state so it can be loaded or modified again.
     */
    void resetDefaults() {
        wheelbase_ = 1.0; // meters
        track_width_ = 1.0; // meters
        wheel_diameter_ = 0.25; // meters
        wheel_radius_ = wheel_diameter_ / 2.0; // meters
        poles_pairs_number_ = 15; // motor pole-pair count
        motor_gear_ratio_ = 1.0; // unitless
        min_erpm_ = 0.0; // ERPM offset applied to sign
        max_steering_radius_ = 5.0; // meters
        min_steering_radius_ = 1.0; // meters
        pose_covariance_diagonal_ = {0.001, 0.001, 0.001, 0.001, 0.001, 0.001};
        twist_covariance_diagonal_ = {0.001, 0.001, 0.001, 0.001, 0.001, 0.001};
        use_sim_time_ = false;
        publish_rate_ = 10.0; // Hz
        base_frame_id_ = "/base_footprint";
        odom_frame_id_ = "/odom";
        enable_odom_tf_ = false;
        invert_right_drive_ = false;
        invert_right_steering_ = false;
        invert_left_drive_ = false;
        invert_left_steering_ = false;
        feedback_timeout_sec_ = 0.5; // seconds: time without feedback considered stale
        initialized_ = false;
    }

    /**
     * @brief Applies supported configuration values from a parsed YAML node.
     *
     * Resets existing values to defaults before applying available overrides. The
     * configuration remains mutable until successfully loaded and validated from a
     * file.
     *
     * @param config Parsed YAML configuration node.
     * @return true if the node is valid and the configuration is mutable, false otherwise.
     */
    bool loadFromYaml(const YAML::Node& config) {
        if (!config) {
            return false;
        }

        if (initialized_) return false; // already initialized, immutability enforced

        resetDefaults();

        if (config["wheels_distance_length"]) {
            wheelbase_ = config["wheels_distance_length"].as<double>();
        }
        if (config["wheels_distance_width"]) {
            track_width_ = config["wheels_distance_width"].as<double>();
        }
        if (config["wheel_diameter"]) {
            wheel_diameter_ = config["wheel_diameter"].as<double>();
            wheel_radius_ = wheel_diameter_ / 2.0;
        }
        if (config["poles_pair_number"]) {
            poles_pairs_number_ = config["poles_pair_number"].as<int>();
        }
        if (config["motor_gear_ratio"]) {
            motor_gear_ratio_ = config["motor_gear_ratio"].as<double>();
        }
        if (config["min_erpm"]) {
            min_erpm_ = config["min_erpm"].as<double>();
        }
        if (config["max_steering_radius"]) {
            max_steering_radius_ = config["max_steering_radius"].as<double>();
        }
        if (config["min_steering_radius"]) {
            min_steering_radius_ = config["min_steering_radius"].as<double>();
        }
        if (config["pose_covariance_diagonal"]) {
            pose_covariance_diagonal_ = config["pose_covariance_diagonal"].as<std::vector<double>>();
        }
        if (config["twist_covariance_diagonal"]) {
            twist_covariance_diagonal_ = config["twist_covariance_diagonal"].as<std::vector<double>>();
        }
        if (config["publish_rate"]) {
            publish_rate_ = config["publish_rate"].as<double>();
        }
        if (config["feedback_timeout_sec"]) {
            feedback_timeout_sec_ = config["feedback_timeout_sec"].as<double>();
        }
        if (config["use_sim_time"]) {
            use_sim_time_ = config["use_sim_time"].as<bool>();
        }
        if (config["base_frame_id"]) {
            base_frame_id_ = config["base_frame_id"].as<std::string>();
        }
        if (config["odom_frame_id"]) {
            odom_frame_id_ = config["odom_frame_id"].as<std::string>();
        }
        if (config["enable_odom_tf"]) {
            enable_odom_tf_ = config["enable_odom_tf"].as<bool>();
        }

        return true;
    }

    /**
     * @brief Loads configuration from a YAML file and validates it.
     *
     * Marks the configuration as initialized only after the file is loaded and
     * all validation checks pass.
     *
     * @param path Filesystem path to the YAML configuration file.
     * @return `true` if the configuration is loaded and valid, `false` if the file
     *         cannot be parsed, the YAML configuration is rejected, or validation
     *         fails.
     */
    bool loadFromFile(const std::string& path) {
        try {
            YAML::Node config = YAML::LoadFile(path);
            if (!loadFromYaml(config)) return false;
            if (!validate()) return false;
            initialized_ = true;
            return true;
        } catch (const YAML::Exception&) {
            return false;
        }
    }

    /**
     * @brief Checks whether the configuration satisfies basic validity requirements.
     *
     * @return `true` if the geometry and timing values are greater than zero and both
     *         covariance vectors contain exactly six elements, `false` otherwise.
     */
    bool validate() const {
        if (wheelbase_ <= 0.0) return false;
        if (track_width_ <= 0.0) return false;
        if (wheel_radius_ <= 0.0) return false;
        if (pose_covariance_diagonal_.size() != 6) return false;
        if (twist_covariance_diagonal_.size() != 6) return false;
        if (publish_rate_ <= 0.0) return false;
        if (feedback_timeout_sec_ <= 0.0) return false;
        return true;
    }

    /// Physical geometry: wheelbase length (meters)
    double wheelbase_{1.0};
    /// Physical geometry: track width (meters)
    double track_width_{1.0};
    /// Wheel diameter (meters)
    double wheel_diameter_{0.25};
    /// Wheel radius (meters)
    double wheel_radius_{0.125};
    /// Motor pole pairs (used to convert ERPM <-> m/s)
    int poles_pairs_number_{15};
    /// Motor gear ratio (unitless)
    double motor_gear_ratio_{1.0};
    /// Minimum ERPM offset applied to drive set values (hardware-specific)
    double min_erpm_{0.0};
    /// Maximum steering radius (meters)
    double max_steering_radius_{5.0};
    /// Minimum steering radius (meters)
    double min_steering_radius_{1.0};
    /// Diagonal elements for pose covariance matrix (x,y,z,roll,pitch,yaw)
    std::vector<double> pose_covariance_diagonal_{0.001, 0.001, 0.001, 0.001, 0.001, 0.001};
    /// Diagonal elements for twist covariance matrix (vx,vy,vz,wx,wy,wz)
    std::vector<double> twist_covariance_diagonal_{0.001, 0.001, 0.001, 0.001, 0.001, 0.001};
    /// Use simulated `/clock` topic for timestamps
    bool use_sim_time_{false};
    /// Desired publish rate for the kinematics node (Hz)
    double publish_rate_{10.0};
    /// Timeout (seconds) after which wheel feedback is considered stale
    double feedback_timeout_sec_{0.5};
    /// TF frame id for the robot base
    std::string base_frame_id_{"/base_footprint"};
    /// TF frame id for odometry
    std::string odom_frame_id_{"/odom"};
    /// Publish odom->base TF when true
    bool enable_odom_tf_{false};
    /// Per-wheel polarity options for drive and steering
    bool invert_right_drive_{false};
    bool invert_right_steering_{false};
    bool invert_left_drive_{false};
    bool invert_left_steering_{false};

    /**
     * @brief Flag set to true after a successful `loadFromFile()`.
     *
     * @note Once set, callers should avoid mutating this object concurrently
     * as it is treated as configuration locked for the running system.
     */
    bool initialized_{false};
};
