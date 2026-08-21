#pragma once

// ----------------------------------------------------------------------------
// PidController — lightweight header-only PID with integral windup clamping.
//
// Designed to be dependency-free (no ROS 2, no Gazebo) so it can be embedded
// directly inside any plugin or utility without pulling in extra headers.
//
// Usage:
//   PidController pid(p, i, d, i_max, i_min);
//   double effort = pid.update(error, dt_seconds);
//   pid.reset();
// ----------------------------------------------------------------------------

#include <algorithm>
#include <cmath>

namespace rover_description {

class PidController {
public:
    // -------------------------------------------------------------------------
    // Constructor
    // -------------------------------------------------------------------------
    PidController() = default;

    PidController(double p, double i, double d,
                  double i_max = 1e9, double i_min = -1e9)
        : p_(p), i_(i), d_(d), i_max_(i_max), i_min_(i_min)
    {}

    // -------------------------------------------------------------------------
    // update() — call once per control tick
    //   error : setpoint − measured
    //   dt    : elapsed time in seconds (must be > 0)
    // Returns the computed effort (un-clamped — caller applies effort clamp).
    // -------------------------------------------------------------------------
    double update(double error, double dt)
    {
        if (dt <= 0.0) { return 0.0; }

        // Proportional
        double p_term = p_ * error;

        // Integral with windup clamping
        i_accum_ += i_ * error * dt;
        i_accum_  = std::clamp(i_accum_, i_min_, i_max_);

        // Derivative (on error, not measurement, to avoid derivative kick on
        // setpoint changes)
        double d_term = 0.0;
        if (first_update_) {
            first_update_ = false;
        } else {
            d_term = d_ * (error - prev_error_) / dt;
        }
        prev_error_ = error;

        return p_term + i_accum_ + d_term;
    }

    // -------------------------------------------------------------------------
    // reset() — clear integrator and derivative history
    // -------------------------------------------------------------------------
    void reset()
    {
        i_accum_      = 0.0;
        prev_error_   = 0.0;
        first_update_ = true;
    }

    // -------------------------------------------------------------------------
    // Setters — allow reconfiguration after construction
    // -------------------------------------------------------------------------
    void setGains(double p, double i, double d)
    {
        p_ = p;  i_ = i;  d_ = d;
        reset();
    }

    void setIntegralLimits(double i_max, double i_min)
    {
        i_max_ = i_max;
        i_min_ = i_min;
    }

private:
    double p_{0.0};
    double i_{0.0};
    double d_{0.0};
    double i_max_{1e9};
    double i_min_{-1e9};

    double i_accum_{0.0};
    double prev_error_{0.0};
    bool   first_update_{true};
};

} // namespace rover_description
