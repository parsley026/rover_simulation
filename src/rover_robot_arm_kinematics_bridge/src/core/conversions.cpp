#include "rover_robot_arm_kinematics_bridge/core/conversions.hpp"

namespace rover_robot_arm_kinematics_bridge {
namespace core {

Conversions::Conversions(int pole_numbers, double default_gear_ratio, double linear_scale)
    : pole_numbers_(pole_numbers)
    , default_gear_ratio_(default_gear_ratio)
    , linear_scale_(linear_scale)
{
}

double Conversions::erpmToRadPerSec(double erpm, double gear_ratio) const
{
    const double ratio = (gear_ratio > 0.0) ? gear_ratio : default_gear_ratio_;
    const double mech_rpm = erpm / (static_cast<double>(pole_numbers_) * ratio);
    return mech_rpm * M_PI / 30.0;
}

double Conversions::radPerSecToErpm(double rads, double gear_ratio) const
{
    const double ratio = (gear_ratio > 0.0) ? gear_ratio : default_gear_ratio_;
    const double mech_rpm = rads * 30.0 / M_PI;
    return mech_rpm * static_cast<double>(pole_numbers_) * ratio;
}

double Conversions::precisePosToRad(double precise_pos_deg, double gear_ratio) const
{
    const double ratio = (gear_ratio > 0.0) ? gear_ratio : default_gear_ratio_;
    return (precise_pos_deg * M_PI / 180.0) / ratio;
}

double Conversions::radToPrecisePos(double rad, double gear_ratio) const
{
    const double ratio = (gear_ratio > 0.0) ? gear_ratio : default_gear_ratio_;
    return (rad * 180.0 / M_PI) * ratio;
}

double Conversions::precisePosToMeters(double precise_pos_deg, double scale) const
{
    const double s = (scale > 0.0) ? scale : linear_scale_;
    return precise_pos_deg * s;
}

double Conversions::metersToPrecisePos(double meters, double scale) const
{
    const double s = (scale > 0.0) ? scale : linear_scale_;
    return (s != 0.0) ? (meters / s) : 0.0;
}

} // namespace core
} // namespace rover_robot_arm_kinematics_bridge
