#pragma once

#include <array>
#include <cmath>
#include <cstddef>

#include "quad_rover_kinematics/RoverConfig.hpp"

/**
 * @struct WheelCommand
 * @brief Low-level wheel command in physical units used by the inverse kinematics.
 *
 * drive_m_s: per-wheel linear speed in meters/sec (m/s)
 * steer_rad: per-wheel steering angle in radians (rad)
 */
struct WheelCommand {
    std::array<double, 4> drive_m_s{};
    std::array<double, 4> steer_rad{};
};

/**
 * @class InverseKinematics
 * @brief Compute per-wheel steering angles and drive speeds from high-level commands.
 *
 * @details
 * InverseKinematics exposes three driving primitives used by `RoverNode`:
 * - `computeAdvance(radius_m, drive_m_s)`: compute wheel steering and speeds for
 *   an arc of given turning radius (meters) and forward speed (m/s).
 * - `computeCrab(vector_x, vector_y, drive_m_s)`: compute wheel commands to
 *   translate laterally (crab) along a vector while maintaining orientation.
 * - `computeSpin(angular_rate_rad_s)`: compute wheel commands for in-place
 *   rotation at angular rate (rad/s).
 *
 * The implementation uses simple geometric relationships between wheel
 * positions, steering radii and linear velocities. Results are returned in
 * physical units and later converted to hardware set values by
 * `HardwareInterface`.
 */
class InverseKinematics {
public:
    /**
     * @brief Construct with optional configuration.
     * @param config Rover configuration used for geometry and limits.
     */
    explicit InverseKinematics(const RoverConfig& config = RoverConfig{});

    /**
     * @brief Update internal configuration.
     * @param config RoverConfig copy used for subsequent computations.
     */
    void setConfig(const RoverConfig& config);

    /**
     * @brief Compute wheel commands for an advance motion with turning radius.
     *
     * @param radius_m Turning radius in meters. Positive/negative indicates direction.
     * @param drive_m_s Forward driving speed in meters/sec.
     * @return WheelCommand per-wheel speeds (m/s) and steering angles (rad).
     */
    WheelCommand computeAdvance(double radius_m, double drive_m_s) const;

    /**
     * @brief Compute wheel commands for crab (lateral) motion.
     *
     * @param vector_x X component of the desired translation vector (unitless ratio).
     * @param vector_y Y component of the desired translation vector (unitless ratio).
     * @param drive_m_s Linear speed magnitude in meters/sec.
     * @return WheelCommand per-wheel speeds (m/s) and steering angles (rad).
     */
    WheelCommand computeCrab(double vector_x, double vector_y, double drive_m_s) const;

    /**
     * @brief Compute wheel commands to spin in place at `angular_rate_rad_s`.
     * @param angular_rate_rad_s Angular rate in radians/sec.
     * @return WheelCommand per-wheel speeds (m/s) and steering angles (rad).
     */
    WheelCommand computeSpin(double angular_rate_rad_s) const;

private:
    RoverConfig config_;
    double radius_a_ratio_{0.0};
    double radius_b_ratio_{0.0};

    /**
     * @brief Helper that returns a limited tangent angle in the range [-pi/2, pi/2].
     *
     * Used to compute steering angles that are continuous and bounded for the
     * vehicle's steering geometry.
     */
    double tangent360(double y, double x) const;
};
