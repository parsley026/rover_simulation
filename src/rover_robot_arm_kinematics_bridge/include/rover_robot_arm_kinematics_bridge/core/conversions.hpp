#pragma once

#include <cmath>

namespace rover_robot_arm_kinematics_bridge {
namespace core {

class Conversions {
public:
    /// @param pole_numbers       Number of motor pole pairs.
    /// @param default_gear_ratio Mechanical gear reduction (default 1000.0 for arm joints).
    /// @param linear_scale       Meters per motor degree for gripper (default 0.0001 m/deg).
    explicit Conversions(
        int pole_numbers = 7,
        double default_gear_ratio = 1000.0,
        double linear_scale = 0.0001);

    /// ERPM -> Joint angular velocity (rad/s)
    double erpmToRadPerSec(double erpm, double gear_ratio = -1.0) const;

    /// Joint angular velocity (rad/s) -> ERPM
    double radPerSecToErpm(double rads, double gear_ratio = -1.0) const;

    /// VESC precise_pos (degrees at motor shaft) -> joint angle (rad)
    double precisePosToRad(double precise_pos_deg, double gear_ratio = -1.0) const;

    /// Joint angle (rad) -> VESC precise_pos (degrees at motor shaft)
    double radToPrecisePos(double rad, double gear_ratio = -1.0) const;

    /// VESC precise_pos (degrees) -> gripper linear position (meters)
    double precisePosToMeters(double precise_pos_deg, double scale = -1.0) const;

    /// Gripper linear position (meters) -> VESC precise_pos (degrees)
    double metersToPrecisePos(double meters, double scale = -1.0) const;

private:
    int    pole_numbers_;
    double default_gear_ratio_;
    double linear_scale_;
};

} // namespace core
} // namespace rover_robot_arm_kinematics_bridge
