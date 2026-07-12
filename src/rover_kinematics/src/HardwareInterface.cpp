#include "quad_rover_kinematics/HardwareInterface.hpp"

#include <cmath>

/**
 * @brief Maps a drive VESC identifier to its wheel index.
 *
 * @param vesc_id Drive VESC identifier.
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
 * @brief Maps a VESC steering identifier to its turn-wheel index.
 *
 * @param vesc_id VESC identifier for the steering actuator.
 * @return std::size_t Corresponding turn-wheel index, or `0` for an unrecognized identifier.
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
 *
 * Formula used (legacy behaviour): mps = erpm * pi / 30 * wheel_radius / poles_pairs_number
 * Polarity inversions configured in `RoverConfig` are applied per-wheel.
 */
double HardwareInterface::metersPerSecondFromErpm(double erpm, std::size_t wheel_index) const {
    // preserve original behaviour: mps = erpm * pi / 30 * wheel_radius / poles_pairs_number
    double mps = erpm * M_PI / 30.0 * config_.wheel_radius_ / static_cast<double>(config_.poles_pairs_number_);
    // apply drive polarity inversion if configured
    bool is_right = (wheel_index == 1 || wheel_index == 3);
    if (is_right && config_.invert_right_drive_) mps = -mps;
    if (!is_right && config_.invert_left_drive_) mps = -mps;
    return mps;
}

/**
 * @brief Converts a linear wheel speed into a hardware drive command.
 *
 * Applies the configured drive conversion, minimum ERPM offset, and
 * wheel-specific drive polarity.
 *
 * @param mps Linear wheel speed in meters per second.
 * @param wheel_index Zero-based wheel index used to select drive polarity.
 * @return Hardware drive set-value in ERPM-like units.
 */
double HardwareInterface::driveSetFromMetersPerSecond(double mps, std::size_t wheel_index) const {
    // preserve original drive_scale: 30.0 / wheel_radius / M_PI * poles_pairs_number * motor_gear_ratio
    double drive_scale = 30.0 / config_.wheel_radius_ / M_PI * static_cast<double>(config_.poles_pairs_number_) * config_.motor_gear_ratio_;
    double base = mps * drive_scale;
    double sign_offset = (base >= 0.0) ? 1.0 : -1.0;
    double set_value = base + sign_offset * config_.min_erpm_;

    // apply drive inversion per wheel
    bool is_right = (wheel_index == 1 || wheel_index == 3);
    if (is_right && config_.invert_right_drive_) set_value = -set_value;
    if (!is_right && config_.invert_left_drive_) set_value = -set_value;

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
    double deg = rad * 180.0 / M_PI;
    bool is_right = (wheel_index == 1 || wheel_index == 3);
    if (is_right && config_.invert_right_steering_) deg = -deg;
    if (!is_right && config_.invert_left_steering_) deg = -deg;
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
    double val = precise_pos; // preserve default behavior
    bool is_right = (wheel_index == 1 || wheel_index == 3);
    if (is_right && config_.invert_right_steering_) val = -val;
    if (!is_right && config_.invert_left_steering_) val = -val;
    return val;
}
