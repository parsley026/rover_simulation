#pragma once

namespace rover_kinematics_bridge {
namespace bridge_modes {

struct BridgeConfig {
    bool invert_left_steering{false};
    bool invert_right_steering{false};
};

class IBridgeMode {
public:
    virtual ~IBridgeMode() = default;

    /// Called periodically by the node's main timer.
    virtual void update() = 0;
};

} // namespace bridge_modes
} // namespace rover_kinematics_bridge
