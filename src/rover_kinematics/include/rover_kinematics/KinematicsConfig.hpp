#pragma once

#include <string>
#include <vector>
#include <ceres/ceres.h>
#include <yaml-cpp/yaml.h>


struct KinematicsConfig {

    KinematicsConfig() { resetDefaults(); }

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

        bool loadFromYaml(const YAML::Node& config) {
        if (!config) {
            return false;
        }

        if (initialized_) return false;

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
        if (config["max_erpm"]) {
            max_erpm_ = config["max_erpm"].as<double>();
        }
        if (config["max_mechanical_angle"]) {
            max_mechanical_angle_ = config["max_mechanical_angle"].as<double>();
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
        if (config["use_measurement_timestamp"]) {
            use_measurement_timestamp_ = config["use_measurement_timestamp"].as<bool>();
        }
        if (config["huber_loss_threshold"]) {
            huber_loss_threshold_ = config["huber_loss_threshold"].as<double>();
        }
        if (config["twist_ema_alpha"]) {
            twist_ema_alpha_ = config["twist_ema_alpha"].as<double>();
        }
        if (config["se2_integration_omega_threshold"]) {
            se2_integration_omega_threshold_ = config["se2_integration_omega_threshold"].as<double>();
        }
        if (config["solver_max_num_iterations"]) {
            solver_max_num_iterations_ = config["solver_max_num_iterations"].as<int>();
        }
        if (config["solver_function_tolerance"]) {
            solver_function_tolerance_ = config["solver_function_tolerance"].as<double>();
        }
        if (config["solver_gradient_tolerance"]) {
            solver_gradient_tolerance_ = config["solver_gradient_tolerance"].as<double>();
        }
        if (config["solver_parameter_tolerance"]) {
            solver_parameter_tolerance_ = config["solver_parameter_tolerance"].as<double>();
        }
        if (config["solver_minimizer_progress_to_stdout"]) {
            solver_minimizer_progress_to_stdout_ = config["solver_minimizer_progress_to_stdout"].as<bool>();
        }
        if (config["solver_linear_solver_type"]) {
            solver_linear_solver_type_ = parseLinearSolverType(config["solver_linear_solver_type"].as<std::string>());
        }
        if (config["solver_trust_region_strategy_type"]) {
            solver_trust_region_strategy_type_ = parseTrustRegionStrategyType(config["solver_trust_region_strategy_type"].as<std::string>());
        }
        if (config["enable_dynamic_wheel_weighting"]) {
            enable_dynamic_wheel_weighting_ = config["enable_dynamic_wheel_weighting"].as<bool>();
        }
        if (config["wheel_quality_low_erpm_threshold"]) {
            wheel_quality_low_erpm_threshold_ = config["wheel_quality_low_erpm_threshold"].as<double>();
        }
        if (config["wheel_quality_high_current_threshold"]) {
            wheel_quality_high_current_threshold_ = config["wheel_quality_high_current_threshold"].as<double>();
        }
        if (config["wheel_quality_high_duty_threshold"]) {
            wheel_quality_high_duty_threshold_ = config["wheel_quality_high_duty_threshold"].as<double>();
        }
        if (config["wheel_quality_min_weight"]) {
            wheel_quality_min_weight_ = config["wheel_quality_min_weight"].as<double>();
        }
        if (config["cmd_vel_autonomy_topic"]) {
            cmd_vel_autonomy_topic_ = config["cmd_vel_autonomy_topic"].as<std::string>();
        } else if (config["cmd_vel_autonomy_topic_"]) {
            cmd_vel_autonomy_topic_ = config["cmd_vel_autonomy_topic_"].as<std::string>();
        }

        updateDerivedParameters();
        return true;
    }

    void resetDefaults() {
        wheelbase_      = 1.0; 
        track_width_    = 1.0;
        wheel_radius_   = 0.125;
        wheel_diameter_ = 0.25;

        poles_pairs_number_ = 15; 
        motor_gear_ratio_   = 1.0;
        min_erpm_           = 0.0000001; 
        max_erpm_           = 1000000.0;
        
        invert_right_drive_    = false;
        invert_right_steering_ = false;
        invert_left_drive_     = false;
        invert_left_steering_  = false;

        max_steering_radius_ = 5.0; 
        min_steering_radius_ = 1.0; 
        max_steering_angle_deg_ = 90.0;
        max_steering_angle_rad_ = 1.57;
        max_wheel_speed_mps_    = 0.0;
        
        pose_covariance_diagonal_  = {0.001, 0.001, 0.001, 0.001, 0.001, 0.001};
        twist_covariance_diagonal_ = {0.001, 0.001, 0.001, 0.001, 0.001, 0.001};

        publish_rate_ = 10.0;
        feedback_timeout_sec_ = 0.5;

        base_frame_id_          = "/base_footprint";
        odom_frame_id_          = "/odom";
        enable_odom_tf_         = false;
        use_measurement_timestamp_ = false;

        cmd_vel_autonomy_topic_ = "/cmd_vel";
        
        use_sim_time_ = false;
        enable_dynamic_wheel_weighting_ = false;
        wheel_quality_low_erpm_threshold_ = 500.0;
        wheel_quality_high_current_threshold_ = 20.0;
        wheel_quality_high_duty_threshold_ = 0.85;
        wheel_quality_min_weight_ = 0.0;

        solver_max_num_iterations_ = 200;
        solver_function_tolerance_ = 1e-8;
        solver_gradient_tolerance_ = 1e-10;
        solver_parameter_tolerance_ = 1e-9;
        solver_minimizer_progress_to_stdout_ = false;
        solver_linear_solver_type_ = ceres::DENSE_QR;
        solver_trust_region_strategy_type_ = ceres::DOGLEG;

        max_allowed_dt_ = 0.5;

        initialized_ = false;
    }

    void updateDerivedParameters() {
        wheel_radius_ = wheel_diameter_ / 2.0;
        
        double max_mech_rpm = max_erpm_ / (static_cast<double>(poles_pairs_number_) * motor_gear_ratio_);
        double max_rad_s = max_mech_rpm * (M_PI / 30.0);
        max_wheel_speed_mps_ = max_rad_s * wheel_radius_;
    }

    static ceres::LinearSolverType parseLinearSolverType(const std::string& value) {
        if (value == "DENSE_QR") return ceres::DENSE_QR;
        if (value == "DENSE_NORMAL_CHOLESKY") return ceres::DENSE_NORMAL_CHOLESKY;
        if (value == "SPARSE_NORMAL_CHOLESKY") return ceres::SPARSE_NORMAL_CHOLESKY;
        if (value == "ITERATIVE_SCHUR") return ceres::ITERATIVE_SCHUR;
        return ceres::DENSE_QR;
    }

    static ceres::TrustRegionStrategyType parseTrustRegionStrategyType(const std::string& value) {
        if (value == "LEVENBERG_MARQUARDT") return ceres::LEVENBERG_MARQUARDT;
        if (value == "DOGLEG") return ceres::DOGLEG;
        return ceres::DOGLEG;
    }

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

    double wheelbase_{1.0};
    double track_width_{1.0};
    double wheel_radius_{0.125};
    double wheel_diameter_{0.25};
    
    int poles_pairs_number_{15};
    double motor_gear_ratio_{1.0};
    double min_erpm_{0.0};
    double max_erpm_{100000.0};
    double max_wheel_speed_mps_{0.0};

    double max_mechanical_angle_{1.57};
    double max_steering_radius_{5.0};
    double min_steering_radius_{1.0};
    double max_steering_angle_deg_{90.0};
    double max_steering_angle_rad_{1.57};
    std::vector<double> pose_covariance_diagonal_{0.001, 0.001, 0.001, 0.001, 0.001, 0.001};
    std::vector<double> twist_covariance_diagonal_{0.001, 0.001, 0.001, 0.001, 0.001, 0.001};
    bool use_sim_time_{false};
    double publish_rate_{10.0};
    double feedback_timeout_sec_{0.5};
    std::string base_frame_id_{"/base_footprint"};
    std::string odom_frame_id_{"/odom"};
    bool enable_odom_tf_{false};
    bool use_measurement_timestamp_{false};

    std::string cmd_vel_autonomy_topic_{"/cmd_vel"};

    bool enable_dynamic_wheel_weighting_{false};
    double wheel_quality_low_erpm_threshold_{500.0};
    double wheel_quality_high_current_threshold_{20.0};
    double wheel_quality_high_duty_threshold_{0.85};
    double wheel_quality_min_weight_{0.0};

    double max_allowed_dt_{0.5};

    bool invert_right_drive_{false};
    bool invert_right_steering_{false};
    bool invert_left_drive_{false};
    bool invert_left_steering_{false};

    double steering_angle_deadband_deg_{0.5};
    double huber_loss_threshold_{0.5};
    double se2_integration_omega_threshold_{1e-4};
    double twist_ema_alpha_{0.8};

    int solver_max_num_iterations_{200};
    double solver_function_tolerance_{1e-8};
    double solver_gradient_tolerance_{1e-10};
    double solver_parameter_tolerance_{1e-9};
    bool solver_minimizer_progress_to_stdout_{false};
    ceres::LinearSolverType solver_linear_solver_type_{ceres::DENSE_QR};
    ceres::TrustRegionStrategyType solver_trust_region_strategy_type_{ceres::DOGLEG};

    bool initialized_{false};
};
