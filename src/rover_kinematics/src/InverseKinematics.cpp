#include "quad_rover_kinematics/InverseKinematics.hpp"

namespace {
constexpr double kMinSteerRadius = 1.0;
constexpr double kMaxSteerRadius = 5.0;
constexpr double kWheelSpeedConversion = 30.0 / M_PI;

inline double clampToZero(double value, double epsilon = 1e-9) {
    return std::fabs(value) < epsilon ? 0.0 : value;
}
}  // namespace

/**
 * @brief Construct InverseKinematics and apply initial configuration.
 */
InverseKinematics::InverseKinematics(const RoverConfig& config) { setConfig(config); }

/**
 * @brief Apply a RoverConfig and precompute internal ratios used by steering mapping.
 */
void InverseKinematics::setConfig(const RoverConfig& config) {
    config_ = config;
    radius_a_ratio_ = (kMinSteerRadius - kMaxSteerRadius) / 0.99;
    radius_b_ratio_ = kMinSteerRadius - radius_a_ratio_;
}

/**
 * @brief Compute a bounded steering angle in the range [-pi/2, pi/2].
 *
 * This helper wraps std::atan2 and folds angles outside the principal
 * steering domain into a reversible equivalent to avoid large jump discontinuities.
 */
double InverseKinematics::tangent360(double y, double x) const {
    double angle_rad = std::atan2(y, x);
    if (angle_rad > M_PI / 2.0) {
        angle_rad -= M_PI;
    }
    if (angle_rad < -M_PI / 2.0) {
        angle_rad += M_PI;
    }
    return angle_rad;
}

/**
 * @brief Compute wheel steering angles and drive speeds for an arc motion.
 *
 * Given a turning radius (meters) and a desired forward speed (m/s), this
 * routine computes per-wheel steering angles and linear wheel speeds based on
 * simple geometric relations. The steering radius mapping applies an internal
 * scaling to remain within mechanical steering limits.
 *
 * @param radius_m Signed turning radius in meters. Positive/negative indicates turn direction.
 * @param drive_m_s Desired linear speed (m/s) for the reference trajectory.
 * @return WheelCommand containing per-wheel drive speed (m/s) and steering angle (rad).
 */
WheelCommand InverseKinematics::computeAdvance(double radius_m, double drive_m_s) const {
    WheelCommand command;
    const double radius_abs = std::fabs(radius_m);
    const double sign_radius = radius_m >= 0.0 ? 1.0 : -1.0;

    if (radius_abs > 1e-9) {
        const double steering_radius = radius_a_ratio_ * radius_abs + radius_b_ratio_;
        const double theta_front_left = sign_radius * tangent360(config_.wheelbase_, 2.0 * steering_radius - config_.track_width_ * sign_radius);
        const double theta_front_right = sign_radius * tangent360(config_.wheelbase_, 2.0 * steering_radius + config_.track_width_ * sign_radius);

        command.steer_rad[0] = theta_front_left;
        command.steer_rad[1] = theta_front_right;
        command.steer_rad[2] = -theta_front_left;
        command.steer_rad[3] = -theta_front_right;

        if (std::fabs(drive_m_s) > 1e-9) {
            const double omega = drive_m_s / std::fabs(steering_radius);
            const double outer_radius = std::sqrt(std::pow(steering_radius + config_.track_width_ / 2.0, 2.0) + std::pow(config_.wheelbase_ / 2.0, 2.0));
            const double inner_radius = std::sqrt(std::pow(steering_radius - config_.track_width_ / 2.0, 2.0) + std::pow(config_.wheelbase_ / 2.0, 2.0));

            const double inner_speed = inner_radius * omega;
            const double outer_speed = outer_radius * omega;

            if (sign_radius > 0.0) {
                command.drive_m_s[0] = inner_speed;
                command.drive_m_s[1] = outer_speed;
                command.drive_m_s[2] = inner_speed;
                command.drive_m_s[3] = outer_speed;
            } else {
                command.drive_m_s[0] = outer_speed;
                command.drive_m_s[1] = inner_speed;
                command.drive_m_s[2] = outer_speed;
                command.drive_m_s[3] = inner_speed;
            }
        }
    } else {
        command.steer_rad.fill(0.0);
        if (std::fabs(drive_m_s) > 1e-9) {
            command.drive_m_s.fill(drive_m_s);
        }
    }

    return command;
}

/**
 * @brief Compute wheel commands to perform a crab (lateral) translation.
 *
 * The algorithm computes a single steering angle applied to all wheels and
 * assigns equal speeds so the vehicle translates without rotation when
 * mechanically feasible.
 */
WheelCommand InverseKinematics::computeCrab(double vector_x, double vector_y, double drive_m_s) const {
    WheelCommand command;
    const double axis_y = vector_y < 0.0 ? 0.0 : vector_y;
    const double theta = std::fabs(axis_y) < 1e-9 && std::fabs(vector_x) < 1e-9 ? 0.0 : std::atan2(vector_x, axis_y);

    command.steer_rad.fill(theta);
    command.drive_m_s.fill(drive_m_s);
    return command;
}

/**
 * @brief Compute wheel commands for in-place spin rotation.
 *
 * The steering angles are chosen so wheels are oriented to maximize rotational
 * leverage and wheel speeds are set proportionally to the requested angular rate.
 */
WheelCommand InverseKinematics::computeSpin(double angular_rate_rad_s) const {
    WheelCommand command;
    const double spin_angle = std::atan2(config_.wheelbase_ / 2.0, config_.track_width_ / 2.0);
    
    // BUG FIX: Calculate the geometric radius from the center of the rover to the wheel
    const double turning_radius = std::hypot(config_.wheelbase_ / 2.0, config_.track_width_ / 2.0);
    
    // BUG FIX: Multiply the requested angular rate by the radius to get the true linear wheel speed (v = w * r)
    const double speed = angular_rate_rad_s * turning_radius;

    command.steer_rad[0] = -spin_angle;
    command.steer_rad[1] = spin_angle;
    command.steer_rad[2] = spin_angle;
    command.steer_rad[3] = -spin_angle;

    command.drive_m_s[0] = -speed;
    command.drive_m_s[1] = speed;
    command.drive_m_s[2] = -speed;
    command.drive_m_s[3] = speed;

    return command;
}