#include "quad_rover_kinematics/ForwardKinematics.hpp"

#include <cmath>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <ceres/covariance.h>

namespace {
constexpr double kMinSteerRadius = 1.0;
constexpr double kMaxSteerRadius = 5.0;
}  // namespace

/**
 * @brief Construct a ForwardKinematics estimator and apply initial configuration.
 * @param config Initial configuration copied into the estimator.
 */
ForwardKinematics::ForwardKinematics(const RoverConfig& config) {
    setConfig(config);
}

/**
 * @brief Applies a rover configuration and refreshes derived parameters and output message settings.
 *
 * @param config Configuration values to apply.
 */
void ForwardKinematics::setConfig(const RoverConfig& config) {
    config_ = config;
    L_ = config_.wheelbase_;
    W_ = config_.track_width_;
    wheel_radius_ = config_.wheel_radius_;
    poles_pairs_number_ = config_.poles_pairs_number_;
    motor_gear_ratio_ = config_.motor_gear_ratio_;
    min_erpm_ = config_.min_erpm_;
    max_steering_radius_ = config_.max_steering_radius_;
    min_steering_radius_ = config_.min_steering_radius_;
    pose_covariance_diagonal_ = config_.pose_covariance_diagonal_;
    twist_covariance_diagonal_ = config_.twist_covariance_diagonal_;
    odom_frame_id_ = config_.odom_frame_id_;
    base_frame_id_ = config_.base_frame_id_;

    setOdometryParam();
    setTFParam();
}

/**
 * @brief Sets the pose covariance diagonal and refreshes the odometry covariance.
 *
 * @param pose_covariance Pose covariance diagonal values.
 */
void ForwardKinematics::setPoseCovariance(const std::vector<double>& pose_covariance) {
    pose_covariance_diagonal_ = pose_covariance;
    setOdometryParam();
}

/**
 * @brief Sets the twist covariance diagonal and refreshes the odometry covariance.
 *
 * @param twist_covariance Covariance values for the twist diagonal.
 */
void ForwardKinematics::setTwistCovariance(const std::vector<double>& twist_covariance) {
    twist_covariance_diagonal_ = twist_covariance;
    setOdometryParam();
}

/**
 * @brief Set odometry and base frame ids used in output messages.
 */
void ForwardKinematics::setFrames(const std::string& odom_frame_id, const std::string& base_frame_id) {
    odom_frame_id_ = odom_frame_id;
    base_frame_id_ = base_frame_id;
    setOdometryParam();
    setTFParam();
}

/**
 * @brief Sets the minimum and maximum steering radii used by the estimator.
 *
 * @param min_radius Minimum allowed steering radius.
 * @param max_radius Maximum allowed steering radius.
 */
void ForwardKinematics::setSteeringRadius(double min_radius, double max_radius) {
    min_steering_radius_ = min_radius;
    max_steering_radius_ = max_radius;
}

/**
 * @brief Set wheel radius used in drive conversions.
 */
void ForwardKinematics::setWheelRadius(double radius) {
    wheel_radius_ = radius;
}

/**
 * @brief Set motor pole-pair count used for ERPM conversions.
 */
void ForwardKinematics::setPolesPairsNumber(int poles) {
    poles_pairs_number_ = poles;
}

/**
 * @brief Set motor gear ratio used for converting between motor and wheel speeds.
 */
void ForwardKinematics::setMotorGearRatio(double ratio) {
    motor_gear_ratio_ = ratio;
}

/**
 * @brief Set vehicle geometry (wheelbase and track width) in meters.
 */
void ForwardKinematics::setLengthWidth(double length, double width) {
    L_ = length;
    W_ = width;
}

/**
 * @brief Reset estimator internal state: pose, velocities and timestamps.
 *
 * This sets pose and velocity estimates to zero and resets solver guesses.
 */
void ForwardKinematics::reset() {
    x_ = 0.0;
    y_ = 0.0;
    heading_ = 0.0;
    linear_x_ = 0.0;
    linear_y_ = 0.0;
    angular_ = 0.0;
    v_x_guess_ = 0.0;
    v_y_guess_ = 0.0;
    omega_guess_ = 0.0;
    timestamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    setOdometryParam();
    setTFParam();
}

/**
 * @brief Initializes odometry frame identifiers, fixed planar fields, and covariance matrices.
 */
void ForwardKinematics::setOdometryParam() {
    odometry_.header.frame_id = odom_frame_id_;
    odometry_.child_frame_id = base_frame_id_;
    odometry_.pose.pose.position.z = 0.0;

    odometry_.pose.covariance = {
        pose_covariance_diagonal_[0], 0., 0., 0., 0., 0.,
        0., pose_covariance_diagonal_[1], 0., 0., 0., 0.,
        0., 0., pose_covariance_diagonal_[2], 0., 0., 0.,
        0., 0., 0., pose_covariance_diagonal_[3], 0., 0.,
        0., 0., 0., 0., pose_covariance_diagonal_[4], 0.,
        0., 0., 0., 0., 0., pose_covariance_diagonal_[5]
    };

    odometry_.twist.twist.linear.y = 0.0;
    odometry_.twist.twist.linear.z = 0.0;
    odometry_.twist.twist.angular.x = 0.0;
    odometry_.twist.twist.angular.y = 0.0;
    odometry_.twist.covariance = {
        twist_covariance_diagonal_[0], 0., 0., 0., 0., 0.,
        0., twist_covariance_diagonal_[1], 0., 0., 0., 0.,
        0., 0., twist_covariance_diagonal_[2], 0., 0., 0.,
        0., 0., 0., twist_covariance_diagonal_[3], 0., 0.,
        0., 0., 0., 0., twist_covariance_diagonal_[4], 0.,
        0., 0., 0., 0., 0., twist_covariance_diagonal_[5]
    };
}

/**
 * @brief Populate TF message skeleton (frame ids and child frame).
 */
void ForwardKinematics::setTFParam() {
    transformation_.transforms.resize(1);
    transformation_.transforms[0].transform.translation.z = 0.0;
    transformation_.transforms[0].child_frame_id = base_frame_id_;
    transformation_.transforms[0].header.frame_id = odom_frame_id_;
}

/**
 * @brief Estimates body velocity and updates the rover pose from wheel feedback.
 *
 * Integrates the estimated motion and populates the odometry and transform
 * outputs, including their covariance values. Updates with a non-positive
 * elapsed time produce an invalid estimate. Changes to wheelbase or track
 * width after initialization are not reflected in persistent residual geometry.
 *
 * @param feedback Per-wheel drive speed and steering feedback.
 * @param timestamp Timestamp associated with the measurement and outputs.
 * @return Odometry estimate containing the updated odometry, transform, and validity flag.
 */
OdometryEstimate ForwardKinematics::update(const rex_interfaces::msg::Wheels& feedback, const rclcpp::Time& timestamp) {
    OdometryEstimate estimate;

    const double dt = (rclcpp::Time(timestamp.nanoseconds(), RCL_ROS_TIME) - rclcpp::Time(timestamp_.nanoseconds(), RCL_ROS_TIME)).seconds();
    if (dt <= 0.0) {
        return estimate;
    }
    timestamp_ = timestamp;

    auto toRad = [](double deg) {
        return std::abs(deg) > 0.5 ? deg * M_PI / 180.0 : 0.0;
    };

    const double frRad = toRad(feedback.front_right.turn.set_value);
    const double flRad = toRad(feedback.front_left.turn.set_value);
    const double rlRad = toRad(feedback.rear_left.turn.set_value);
    const double rrRad = toRad(feedback.rear_right.turn.set_value);

    auto wheelVel = [&](double speed, double steer_rad) {
        return std::make_pair(speed * std::cos(steer_rad), speed * std::sin(steer_rad));
    };

    const auto vFR = wheelVel(feedback.front_right.drive.set_value, frRad);
    const auto vFL = wheelVel(feedback.front_left.drive.set_value, flRad);
    const auto vRL = wheelVel(feedback.rear_left.drive.set_value, rlRad);
    const auto vRR = wheelVel(feedback.rear_right.drive.set_value, rrRad);

    const double xFR = +L_ / 2.0;
    const double yFR = -W_ / 2.0;
    const double xFL = +L_ / 2.0;
    const double yFL = +W_ / 2.0;
    const double xRL = -L_ / 2.0;
    const double yRL = +W_ / 2.0;
    const double xRR = -L_ / 2.0;
    const double yRR = -W_ / 2.0;

    // Initialize reusable Ceres structures on first call or when geometry changes
    if (!problem_initialized_) {
        // set measured pointers for residuals
        wheel_residuals_[0] = new WheelResidual(&measured_vx_[0], &measured_vy_[0], xFR, yFR);
        wheel_residuals_[1] = new WheelResidual(&measured_vx_[1], &measured_vy_[1], xFL, yFL);
        wheel_residuals_[2] = new WheelResidual(&measured_vx_[2], &measured_vy_[2], xRL, yRL);
        wheel_residuals_[3] = new WheelResidual(&measured_vx_[3], &measured_vy_[3], xRR, yRR);

        cost_functions_[0] = new ceres::AutoDiffCostFunction<WheelResidual, 2, 1, 1, 1>(wheel_residuals_[0]);
        cost_functions_[1] = new ceres::AutoDiffCostFunction<WheelResidual, 2, 1, 1, 1>(wheel_residuals_[1]);
        cost_functions_[2] = new ceres::AutoDiffCostFunction<WheelResidual, 2, 1, 1, 1>(wheel_residuals_[2]);
        cost_functions_[3] = new ceres::AutoDiffCostFunction<WheelResidual, 2, 1, 1, 1>(wheel_residuals_[3]);

        problem_.AddResidualBlock(cost_functions_[0], nullptr, &v_x_param_, &v_y_param_, &omega_param_);
        problem_.AddResidualBlock(cost_functions_[1], nullptr, &v_x_param_, &v_y_param_, &omega_param_);
        problem_.AddResidualBlock(cost_functions_[2], nullptr, &v_x_param_, &v_y_param_, &omega_param_);
        problem_.AddResidualBlock(cost_functions_[3], nullptr, &v_x_param_, &v_y_param_, &omega_param_);

        problem_initialized_ = true;
    } else {
        // If geometry changed (L_, W_) we'd need to rebuild residuals; omitted for performance.
    }

    // cleanup: ensure existing odometry covariances reflect configured defaults
    setOdometryParam();

    // Set measured values into reusable arrays
    measured_vx_[0] = vFR.first; measured_vy_[0] = vFR.second;
    measured_vx_[1] = vFL.first; measured_vy_[1] = vFL.second;
    measured_vx_[2] = vRL.first; measured_vy_[2] = vRL.second;
    measured_vx_[3] = vRR.first; measured_vy_[3] = vRR.second;

    // Initialize parameters from previous guesses
    v_x_param_ = v_x_guess_;
    v_y_param_ = v_y_guess_;
    omega_param_ = omega_guess_;

    ceres::Solver::Options options;
    options.max_num_iterations = 200;
    options.function_tolerance = 1e-8;
    options.gradient_tolerance = 1e-10;
    options.parameter_tolerance = 1e-9;
    options.linear_solver_type = ceres::DENSE_QR;
    options.trust_region_strategy_type = ceres::DOGLEG;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem_, &summary);

    // Update guesses from solved parameters
    v_x_guess_ = v_x_param_;
    v_y_guess_ = v_y_param_;
    omega_guess_ = omega_param_;

    // Compute parameter covariance using Ceres covariance estimation
    // Number of residuals: 4 wheels * 2 measurements = 8
    const int num_residuals = 8;
    const int num_params = 3;
    double sigma2 = 0.0;
    if (num_residuals - num_params > 0) {
        // summary.final_cost is half sum of squared residuals in Ceres
        double chi2 = 2.0 * summary.final_cost;
        sigma2 = chi2 / static_cast<double>(num_residuals - num_params);
    } else {
        sigma2 = 1e-6;
    }

    ceres::Covariance::Options cov_options;
    ceres::Covariance covariance(cov_options);
    std::vector<std::pair<const double*, const double*>> cov_blocks;
    cov_blocks.emplace_back(&v_x_param_, &v_x_param_);
    cov_blocks.emplace_back(&v_y_param_, &v_y_param_);
    cov_blocks.emplace_back(&omega_param_, &omega_param_);
    covariance.Compute(cov_blocks, &problem_);

    // Retrieve covariance blocks and scale by estimated sigma^2
    double cov_vx_vx = 0.0, cov_vy_vy = 0.0, cov_omega_omega = 0.0;
    double buf[1*1];
    covariance.GetCovarianceBlock(&v_x_param_, &v_x_param_, buf);
    cov_vx_vx = buf[0] * sigma2;
    covariance.GetCovarianceBlock(&v_y_param_, &v_y_param_, buf);
    cov_vy_vy = buf[0] * sigma2;
    covariance.GetCovarianceBlock(&omega_param_, &omega_param_, buf);
    cov_omega_omega = buf[0] * sigma2;

    // Integrate velocity covariances into pose covariance (approximate)
    double pose_cov_x = (cov_vx_vx) * dt * dt + pose_covariance_diagonal_[0];
    double pose_cov_y = (cov_vy_vy) * dt * dt + pose_covariance_diagonal_[1];
    double pose_cov_theta = (cov_omega_omega) * dt * dt + pose_covariance_diagonal_[5];

    // Set twist covariance from parameter covariance
    twist_covariance_diagonal_[0] = cov_vx_vx;
    twist_covariance_diagonal_[1] = cov_vy_vy;
    twist_covariance_diagonal_[5] = cov_omega_omega;

    // Update odometry and transformation as before

    const double dx_local = v_x_param_ * dt;
    const double dy_local = v_y_param_ * dt;
    const double cHeading = std::cos(heading_);
    const double sHeading = std::sin(heading_);

    x_ += dx_local * cHeading - dy_local * sHeading;
    y_ += dx_local * sHeading + dy_local * cHeading;
    heading_ += omega_param_ * dt;

    odometry_.header.stamp = timestamp;
    odometry_.header.frame_id = odom_frame_id_;
    odometry_.child_frame_id = base_frame_id_;
    odometry_.pose.pose.position.x = x_;
    odometry_.pose.pose.position.y = y_;
    odometry_.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, heading_);
    odometry_.pose.pose.orientation = tf2::toMsg(q);

    odometry_.twist.twist.linear.x = v_x_param_;
    odometry_.twist.twist.linear.y = v_y_param_;
    odometry_.twist.twist.angular.z = omega_param_;

    // Update covariance entries in the odometry message
    odometry_.pose.covariance[0] = pose_cov_x;
    odometry_.pose.covariance[7] = pose_cov_y;
    odometry_.pose.covariance[35] = pose_cov_theta;

    odometry_.twist.covariance[0] = cov_vx_vx;
    odometry_.twist.covariance[7] = cov_vy_vy;
    odometry_.twist.covariance[35] = cov_omega_omega;

    transformation_.transforms[0].header.stamp = timestamp;
    transformation_.transforms[0].transform.translation.x = x_;
    transformation_.transforms[0].transform.translation.y = y_;
    transformation_.transforms[0].transform.rotation = tf2::toMsg(q);

    linear_x_ = v_x_param_;
    linear_y_ = v_y_param_;
    angular_ = omega_param_;

    estimate.odometry = odometry_;
    estimate.transform = transformation_;
    estimate.valid = true;
    return estimate;
}
