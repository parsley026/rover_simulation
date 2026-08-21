#pragma once

namespace rover_kinematics_bridge {
namespace core {

// ---------------------------------------------------------------------------
// VESC CAN bus IDs — drive motors
// ---------------------------------------------------------------------------
constexpr int FRONT_LEFT_DRIVE  = 0x50;
constexpr int FRONT_RIGHT_DRIVE = 0x51;
constexpr int REAR_LEFT_DRIVE   = 0x52;
constexpr int REAR_RIGHT_DRIVE  = 0x53;

// ---------------------------------------------------------------------------
// VESC CAN bus IDs — steering motors
// ---------------------------------------------------------------------------
constexpr int FRONT_LEFT_TURN  = 0x60;
constexpr int FRONT_RIGHT_TURN = 0x61;
constexpr int REAR_RIGHT_TURN  = 0x62;
constexpr int REAR_LEFT_TURN   = 0x63;

constexpr int ERPM_MODE    = 1;
constexpr int CURRENT_MODE = 3;

constexpr double STEER_PRESCALE = 0.01;

} // namespace core
} // namespace rover_kinematics_bridge
