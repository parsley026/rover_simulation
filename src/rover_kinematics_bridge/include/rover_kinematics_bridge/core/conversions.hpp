#pragma once

namespace rover_kinematics_bridge {
namespace core {

class Conversions {
public:
    /// @param pole_numbers  Number of electrical pole-pairs in the drive motor.
    /// @param gear_ratio    Mechanical gear reduction between motor and wheel shaft.
    ///                      Must match motor_gear_ratio in rover_config.yaml.
    explicit Conversions(int pole_numbers, double gear_ratio = 1.0);

    /// ERPM -> wheel angular velocity (rad/s)
    double erpmToRadPerSec(double erpm) const;
    
    /// Wheel angular velocity (rad/s) -> ERPM
    double radPerSecToErpm(double rads) const;

private:
    int    pole_numbers_;
    double gear_ratio_;
};

} // namespace core
} // namespace rover_kinematics_bridge
