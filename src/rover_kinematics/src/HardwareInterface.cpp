#include "rover_kinematics/HardwareInterface.hpp"
#include <cmath>

/**
 * @brief Maps a drive VESC identifier to its wheel index.
 *
 * @param vesc_id VESC identifier for the drive actuator.
 * @return std::size_t Wheel index from 0 to 3; returns 0 for an unrecognized identifier.
 */
std::size_t HardwareInterface::vescIdToDriveWheelIndex(uint8_t vesc_id) {
    switch (vesc_id) {
        case 0x50: return 0; // front_left
        case 0x51: return 1; // front_right
        case 0x52: return 2; // rear_left
        case 0x53: return 3; // rear_right
        default: return 0;
    }
}

/**
 * @brief Maps a steering VESC identifier to its wheel index.
 *
 * @param vesc_id VESC identifier for the steering actuator.
 * @return std::size_t Wheel index from 0 to 3; returns 0 for an unrecognized identifier.
 */
std::size_t HardwareInterface::vescIdToTurnWheelIndex(uint8_t vesc_id) {
    switch (vesc_id) {
        case 0x60: return 0; // front_left
        case 0x61: return 1; // front_right
        case 0x62: return 3; // rear_right (note: original mapping used 2 for rear_right in some places)
        case 0x63: return 2; // rear_left
        default: return 0;
    }
}

/**
 * @brief Map reported ERPM-like value to linear speed (m/s).
 * * Pipeline: Motor ERPM -> Mechanical RPM -> Angular Velocity (rad/s) -> Linear Speed (m/s)
 */
double HardwareInterface::metersPerSecondFromErpm(double erpm, std::size_t wheel_index) const {
    
    // Step 1: Convert Electrical RPM (ERPM) to Mechanical RPM
    double poles = static_cast<double>(config_.poles_pairs_number_);
    double mech_rpm = erpm / (poles * config_.motor_gear_ratio_);

    // Step 2: Convert Mechanical RPM to Angular Velocity (rad/s)
    // Formula: RPM * (2 * PI / 60) simplifies to RPM * (PI / 30)
    double angular_velocity_rad_s = mech_rpm * (M_PI / 30.0);

    // Step 3: Convert Angular Velocity to Linear Speed (m/s)
    // Formula: v = omega * r
    double mps = angular_velocity_rad_s * config_.wheel_radius_;

    // Step 4: Apply drive polarity inversion if configured
    bool is_right = (wheel_index == 1 || wheel_index == 3);
    if ( is_right && config_.invert_right_drive_) mps = -mps;
    if (!is_right && config_.invert_left_drive_ ) mps = -mps;
    
    return mps;
}

/**
 * @brief Converts a linear wheel speed into a hardware drive command.
 * * Pipeline: Linear Speed (m/s) -> Angular Velocity (rad/s) -> Mechanical RPM -> Motor ERPM
 */
double HardwareInterface::driveSetFromMetersPerSecond(double mps, std::size_t wheel_index) const {
    
    // Bug fix: Strictly enforce zero velocity to prevent static friction offset creep
    if (std::fabs(mps) < 1e-6) {
        return 0.0; 
    }

    // Step 1: Convert Linear Speed (m/s) to Angular Velocity (rad/s)
    // Formula: omega = v / r
    double angular_velocity_rad_s = mps / config_.wheel_radius_;

    // Step 2: Convert Angular Velocity to Mechanical RPM
    // Formula: rad/s * (60 / 2 * PI) simplifies to rad/s * (30 / PI)
    double mech_rpm = angular_velocity_rad_s * (30.0 / M_PI);

    // Step 3: Convert Mechanical RPM to Electrical RPM (ERPM)
    double poles = static_cast<double>(config_.poles_pairs_number_);
    double target_erpm = mech_rpm * poles * config_.motor_gear_ratio_;

    // Step 4: Apply minimum ERPM offset to overcome static friction
    double sign_offset = (target_erpm > 0.0) ? 1.0 : -1.0;
    double set_value = target_erpm + (sign_offset * config_.min_erpm_);

    // Step 5: Apply drive polarity inversion if configured
    bool is_right = (wheel_index == 1 || wheel_index == 3);
    if ( is_right && config_.invert_right_drive_) set_value = -set_value;
    if (!is_right && config_.invert_left_drive_ ) set_value = -set_value;

    return set_value;
}

/**
 * @brief Converts a steering angle to the hardware steering set-unit.
 *
 * @param rad Steering angle in radians.
 * @param wheel_index Index of the wheel receiving the command.
 * @return Steering set-unit value in degrees, with configured wheel-side polarity applied.
 */
double HardwareInterface::steeringSetFromRadians(double rad, std::size_t wheel_index) const {
    double deg = rad * (180.0 / M_PI);
    
    bool is_right = (wheel_index == 1 || wheel_index == 3);
    if ( is_right && config_.invert_right_steering_) deg = -deg;
    if (!is_right && config_.invert_left_steering_ ) deg = -deg;
    
    return deg;
}

/**
 * @brief Converts a steering position reading to a steering set-value with wheel-specific polarity.
 *
 * @param precise_pos Steering encoder position.
 * @param wheel_index Wheel index used to select the steering polarity.
 * @return The steering set-value, with polarity applied for the specified wheel.
 */
double HardwareInterface::steeringSetFromPrecisePos(double precise_pos, std::size_t wheel_index) const {
    double val = precise_pos; 
    
    bool is_right = (wheel_index == 1 || wheel_index == 3);
    if ( is_right && config_.invert_right_steering_) val = -val;
    if (!is_right && config_.invert_left_steering_ ) val = -val;
    
    return val;
}