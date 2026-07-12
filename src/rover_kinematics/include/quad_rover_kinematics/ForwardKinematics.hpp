#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rex_interfaces/msg/wheels.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <ceres/ceres.h>

#include "quad_rover_kinematics/RoverConfig.hpp"

/**
 * @struct OdometryEstimate
 * @brief Result container returned by ForwardKinematics::update().
 *
 * @details
 * Holds the computed `nav_msgs::msg::Odometry` message, an optional TF
 * transform (as `tf2_msgs::msg::TFMessage`) and a boolean `valid` flag
 * indicating whether the estimator produced a meaningful output for the
 * provided input. This struct is small and intended to be returned by value
 * from `ForwardKinematics::update()`.
 */
struct OdometryEstimate {
    /// Computed odometry (pose, twist and covariances)
    nav_msgs::msg::Odometry odometry;
    /// TF transform representing odom -> base transform
    tf2_msgs::msg::TFMessage transform;
    /// True when the estimator successfully produced a valid estimate
    bool valid{false};
};

/**
 * @class ForwardKinematics
 * @brief Estimate robot pose and velocity from per-wheel feedback.
 *
 * @details
 * ForwardKinematics fuses steer and drive feedback from each wheel to
 * estimate body-frame velocities (v_x, v_y, omega) using a small nonlinear
 * least-squares problem solved by Ceres Solver. The estimator maps wheel
 * tangential velocities into body-frame velocity constraints and solves for
 * the best-fit motion parameters, then integrates them to produce odometry.
 *
 * Important algorithmic notes:
 * - Residuals encode the relation measured_v = R * [v_x v_y omega]^T where
 *   R depends on wheel positions (x_i, y_i). See `WheelResidual` for
 *   explicit equations.
 * - Ceres is used with `AutoDiffCostFunction` for the residuals and
 *   `ceres::Covariance` to estimate parameter covariances. The covariance is
 *   mapped into odometry pose/twist covariances as an approximation.
 * - Residuals and AutoDiff wrappers are allocated once and reused to
 *   minimize realtime allocations. If geometry (L_, W_) changes during
 *   runtime the residuals must be rebuilt (not implemented by default).
 *
 * Thread-safety: Not thread-safe. Callers should update the estimator from a
 * single thread (typically the ROS timer callback in `RoverNode`).
 */
class ForwardKinematics {
public:
    /**
     * @brief Construct the ForwardKinematics estimator.
     * @param config Initial `RoverConfig` used to populate geometry and covariances.
     */
    explicit ForwardKinematics(const RoverConfig& config = RoverConfig{});

    /**
     * @brief Update internal configuration and derived geometry.
     * @param config Rover configuration containing geometry, covariances and flags.
     */
    void setConfig(const RoverConfig& config);

    /**
     * @brief Set pose covariance diagonal used to populate `nav_msgs::msg::Odometry`.
     * @param pose_covariance Six-element vector in the order (x,y,z,roll,pitch,yaw).
     */
    void setPoseCovariance(const std::vector<double>& pose_covariance);

    /**
     * @brief Set twist covariance diagonal used to populate `nav_msgs::msg::Odometry`.
     * @param twist_covariance Six-element vector in the order (vx,vy,vz,wx,wy,wz).
     */
    void setTwistCovariance(const std::vector<double>& twist_covariance);

    /**
     * @brief Configure odometry and base frame ids for outputs.
     * @param odom_frame_id Frame id for odometry (parent frame).
     * @param base_frame_id Child frame representing the robot base.
     */
    void setFrames(const std::string& odom_frame_id, const std::string& base_frame_id);

    /**
     * @brief Set admissible steering radius range (meters).
     */
    void setSteeringRadius(double min_radius, double max_radius);

    /**
     * @brief Set wheel radius used to convert ERPM <-> m/s.
     * @param radius Wheel radius in meters.
     */
    void setWheelRadius(double radius);

    /**
     * @brief Set the number of motor pole pairs used by conversion formulas.
     */
    void setPolesPairsNumber(int poles);

    /**
     * @brief Set the motor gear ratio used when computing set-values.
     */
    void setMotorGearRatio(double ratio);

    /**
     * @brief Set vehicle wheelbase (length) and track (width) in meters.
     */
    void setLengthWidth(double length, double width);

    /**
     * @brief Reset pose, velocity and estimator internal state to zero.
     */
    void reset();

    /**
     * @brief Populate odometry message header/frame and pose covariance arrays.
     *
     * Called internally whenever covariance or frame configuration changes.
     */
    void setOdometryParam();

    /**
     * @brief Populate TF message skeleton (child frame and parent frame id).
     */
    void setTFParam();

    /**
     * @brief Fuse wheel feedback into an odometry estimate.
     *
     * @param feedback Wheel feedback message containing drive and turn set_values.
     * @param timestamp ROS timestamp to attach to output messages.
     * @return OdometryEstimate containing odometry, TF and a validity flag.
     */
    OdometryEstimate update(const rex_interfaces::msg::Wheels& feedback, const rclcpp::Time& timestamp);

    /**
 * @brief Provides the current odometry estimate.
 *
 * @return const nav_msgs::msg::Odometry& The stored odometry message.
 */
const nav_msgs::msg::Odometry& odometry() const { return odometry_; }
    /**
 * @brief Provides the current odometry-to-base transform.
 *
 * @return const tf2_msgs::msg::TFMessage& Current transform message.
 */
const tf2_msgs::msg::TFMessage& transform() const { return transformation_; }

    /**
 * @brief Gets the estimated x position.
 *
 * @return double Estimated x position in the odometry frame.
 */
double x() const { return x_; }
    /**
 * @brief Gets the estimated y position.
 *
 * @return double Estimated y position in the odometry frame.
 */
double y() const { return y_; }
    /**
 * @brief Gets the estimated heading.
 *
 * @return double Estimated heading in radians.
 */
double heading() const { return heading_; }
    /**
 * @brief Gets the estimated longitudinal velocity.
 *
 * @return double Estimated body-frame velocity along the x-axis, in metres per second.
 */
double linearX() const { return linear_x_; }
    /**
 * @brief Gets the estimated lateral velocity.
 *
 * @return double Estimated lateral velocity in metres per second.
 */
double linearY() const { return linear_y_; }
    /**
 * @brief Gets the estimated angular velocity.
 *
 * @return double Estimated angular velocity.
 */
double angular() const { return angular_; }

private:
    struct WheelResidual {
        /**
             * @brief Initializes a wheel residual with measured velocity pointers and wheel position.
             *
             * @param v_xi_ptr Pointer to the measured wheel velocity in the x direction.
             * @param v_yi_ptr Pointer to the measured wheel velocity in the y direction.
             * @param x_i Wheel x-coordinate relative to the vehicle frame.
             * @param y_i Wheel y-coordinate relative to the vehicle frame.
             */
            WheelResidual(const double* v_xi_ptr, const double* v_yi_ptr, double x_i, double y_i)
            : v_xi_ptr_(v_xi_ptr), v_yi_ptr_(v_yi_ptr), x_i_(x_i), y_i_(y_i) {}

        template <typename T>
        /**
         * @brief Computes residuals between measured wheel velocities and predicted body motion.
         *
         * @param v_x Estimated body-frame velocity along the x-axis.
         * @param v_y Estimated body-frame velocity along the y-axis.
         * @param omega Estimated angular velocity.
         * @param residual Output residuals for the wheel velocity components.
         * @return true after computing the residuals.
         */
        bool operator()(const T* const v_x, const T* const v_y, const T* const omega, T* residual) const {
            // measured values are provided via pointers to doubles and read at evaluation time
            const T measured_vx = T(*v_xi_ptr_);
            const T measured_vy = T(*v_yi_ptr_);
            residual[0] = measured_vx - (*v_x - *omega * T(y_i_));
            residual[1] = measured_vy - (*v_y + *omega * T(x_i_));
            return true;
        }

    private:
        const double* v_xi_ptr_;
        const double* v_yi_ptr_;
        const double x_i_;
        const double y_i_;
    };

    RoverConfig config_;
    nav_msgs::msg::Odometry odometry_;
    tf2_msgs::msg::TFMessage transformation_;
    rclcpp::Time timestamp_;

    double max_steering_radius_{2.0};
    double min_steering_radius_{1.0};

    double x_{0.0};
    double y_{0.0};
    double heading_{0.0};
    double linear_x_{0.0};
    double linear_y_{0.0};
    double angular_{0.0};

    double L_{0.0};
    double W_{0.0};
    double wheel_radius_{0.0};
    int poles_pairs_number_{0};
    double motor_gear_ratio_{0.0};
    double min_erpm_{0.0};

    double v_x_guess_{0.0};
    double v_y_guess_{0.0};
    double omega_guess_{0.0};

    // Reusable parameter storage for Ceres
    double v_x_param_{0.0};
    double v_y_param_{0.0};
    double omega_param_{0.0};

    // Measured wheel velocities (reused to avoid allocations)
    double measured_vx_[4] = {0.0, 0.0, 0.0, 0.0};
    double measured_vy_[4] = {0.0, 0.0, 0.0, 0.0};

    // Reusable residuals and cost functions
    WheelResidual* wheel_residuals_[4] = {nullptr, nullptr, nullptr, nullptr};
    ceres::CostFunction* cost_functions_[4] = {nullptr, nullptr, nullptr, nullptr};
    ceres::Problem problem_;
    bool problem_initialized_{false};
    // Ensure proper cleanup of allocated residuals
public:
    /**
     * @brief Releases the resources allocated for wheel residuals and cost functions.
     */
    ~ForwardKinematics() {
        for (int i = 0; i < 4; ++i) {
            if (cost_functions_[i]) delete cost_functions_[i];
            if (wheel_residuals_[i]) delete wheel_residuals_[i];
        }
    }

    std::string odom_frame_id_{"/odom"};
    std::string base_frame_id_{"/base_link"};
    std::vector<double> pose_covariance_diagonal_{0.001, 0.001, 0.001, 0.001, 0.001, 0.001};
    std::vector<double> twist_covariance_diagonal_{0.001, 0.001, 0.001, 0.001, 0.001, 0.001};
};
