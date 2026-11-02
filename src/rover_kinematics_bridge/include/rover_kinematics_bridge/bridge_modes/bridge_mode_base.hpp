#pragma once

namespace rover_kinematics_bridge {
namespace bridge_modes {

struct BridgeConfig {
    bool invert_drive_fl{false};
    bool invert_drive_fr{false};
    bool invert_drive_rl{false};
    bool invert_drive_rr{false};

    bool invert_steering_fl{true};
    bool invert_steering_fr{true};
    bool invert_steering_rl{true};
    bool invert_steering_rr{true};
};

class IBridgeMode {
public:
    virtual ~IBridgeMode() = default;

    /// Called periodically by the node's main timer.
    virtual void update() = 0;
};

} // namespace bridge_modes
} // namespace rover_kinematics_bridge
