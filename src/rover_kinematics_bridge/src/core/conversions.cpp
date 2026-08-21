#include "rover_kinematics_bridge/core/conversions.hpp"
#include <cmath>

namespace rover_kinematics_bridge {
namespace core {

Conversions::Conversions(int pole_numbers, double gear_ratio)
    : pole_numbers_(pole_numbers)
    , gear_ratio_(gear_ratio)
{
}

double Conversions::erpmToRadPerSec(double erpm) const
{
    // Mirror of hardware_interface.cpp:metersPerSecondFromErpm pipeline:
    //   ERPM -> mech_rpm: mech_rpm = ERPM / (poles * gear_ratio)
    //   mech_rpm -> rad/s: omega = mech_rpm * π / 30
    const double mech_rpm = erpm / (static_cast<double>(pole_numbers_) * gear_ratio_);
    return mech_rpm * M_PI / 30.0;
}

double Conversions::radPerSecToErpm(double rads) const
{
    // Mirror of hardware_interface.cpp:driveSetFromMetersPerSecond pipeline:
    //   rad/s -> mech_rpm: mech_rpm = omega * 30 / π
    //   mech_rpm -> ERPM: ERPM = mech_rpm * poles * gear_ratio
    const double mech_rpm = rads * 30.0 / M_PI;
    return mech_rpm * static_cast<double>(pole_numbers_) * gear_ratio_;
}

} // namespace core
} // namespace rover_kinematics_bridge
