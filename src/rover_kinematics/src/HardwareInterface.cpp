#include "quad_rover_kinematics/HardwareInterface.hpp"

#include <cmath>

std::size_t HardwareInterface::vescIdToDriveWheelIndex(uint8_t vesc_id) {
    switch (vesc_id) {
        case 0x50: return 0; // front_left
        case 0x51: return 1; // front_right
        case 0x52: return 2; // rear_left
        case 0x53: return 3; // rear_right
        default: return 0;
    }
}

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
 * @brief Convert linear speed (m/s) to a hardware drive set_value (ERPM-like).
 *
 * This preserves the original drive scale used by the codebase and applies
 * a sign-based offset `min_erpm_` to ensure non-zero command magnitudes.
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
 * @brief Convert steering angle (rad) to the hardware steering set-unit.
 *
 * Legacy code used degrees; this function converts radians to degrees and
 * applies configured polarity inversion for left/right wheels.
 */
double HardwareInterface::steeringSetFromRadians(double rad, std::size_t wheel_index) const {
    double deg = rad * 180.0 / M_PI;
    bool is_right = (wheel_index == 1 || wheel_index == 3);
    if (is_right && config_.invert_right_steering_) deg = -deg;
    if (!is_right && config_.invert_left_steering_) deg = -deg;
    return deg;
}

/**
 * @brief Map a hardware `precise_pos` reading to a steering set-value, applying polarity.
 *
 * This function preserves the default mapping while allowing inverting
 * the reported value for wheels where the encoder/polarity is reversed.
 */
double HardwareInterface::steeringSetFromPrecisePos(double precise_pos, std::size_t wheel_index) const {
    double val = precise_pos; // preserve default behavior
    bool is_right = (wheel_index == 1 || wheel_index == 3);
    if (is_right && config_.invert_right_steering_) val = -val;
    if (!is_right && config_.invert_left_steering_) val = -val;
    return val;
}
