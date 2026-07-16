#include "rover_kinematics/KinematicsSolver.hpp"

KinematicsSolver::KinematicsSolver(const KinematicsConfig& config) { 
    setConfig(config); 
}

void KinematicsSolver::setConfig(const KinematicsConfig& config) {
    config_ = config;
}

void KinematicsSolver::enforceDriveVelocityLimit(
    WheelCommand& command
) const {
    double max_wheel_speed_m_s = 0.0;
    for (double wheel_speed_m_s : command.drive_velocity_mps) {
        max_wheel_speed_m_s = std::max(max_wheel_speed_m_s, std::fabs(wheel_speed_m_s));
    }

    if (max_wheel_speed_m_s > config_.max_wheel_speed_mps_) {
        for (int wheel_index = 0; wheel_index < 4; ++wheel_index) {
            command.drive_velocity_mps[wheel_index] = command.drive_velocity_mps[wheel_index] * (config_.max_wheel_speed_mps_ / max_wheel_speed_m_s);
        }
    }
}

void KinematicsSolver::normalizeSteerAngle(
    WheelCommand& command
) const {
    for (int wheel_index = 0; wheel_index < 4; ++wheel_index) {
        command.steering_angle_rad[wheel_index] = std::atan2(
            std::sin(command.steering_angle_rad[wheel_index]),
            std::cos(command.steering_angle_rad[wheel_index])
        );
    }
}

void KinematicsSolver::optimizeSteerAngle(
    WheelCommand& command
) const {
    for (int wheel_index = 0; wheel_index < 4; ++wheel_index) {
        double& steering_angle_rad = command.steering_angle_rad[wheel_index];
        double& wheel_speed_m_s    = command.drive_velocity_mps[wheel_index];

        if (steering_angle_rad >  std::numbers::pi / 2.0) {
            steering_angle_rad -= std::numbers::pi;
            wheel_speed_m_s = -wheel_speed_m_s;
        }
        if (steering_angle_rad < -std::numbers::pi / 2.0) {
            steering_angle_rad += std::numbers::pi;
            wheel_speed_m_s = -wheel_speed_m_s;
        }
    }
}

void KinematicsSolver::clampDriveVelocity(
    WheelCommand& command,
    double max_velocity_m_s
) const {
    for (double& velocity_m_s : command.drive_velocity_mps) {
        velocity_m_s = std::clamp(
            velocity_m_s,
            -max_velocity_m_s,
             max_velocity_m_s
        );
    }
}

void KinematicsSolver::clampSteerAngle(
    WheelCommand& command,
    double max_angle_rad
) const {
    for (double& angle_rad : command.steering_angle_rad) {
        angle_rad = std::clamp(
            angle_rad,
            -max_angle_rad,
             max_angle_rad
        );
    }
}

void KinematicsSolver::enforceMinimumTurningRadius(
    double velocity_x_mps, 
    double& velocity_omega_rad_s
) const {
    if (std::fabs(velocity_omega_rad_s) < 1e-6) return;

    const double turning_radius = std::fabs(velocity_x_mps / velocity_omega_rad_s);
    if (turning_radius < config_.min_steering_radius_) {
        velocity_omega_rad_s = std::copysign(
            std::fabs(velocity_x_mps) / config_.min_steering_radius_, 
            velocity_omega_rad_s
        );
    }
}

WheelCommand KinematicsSolver::computeSymmetricAckermann(
    double velocity_x_mps, 
    double velocity_omega_rad_s
) const {
    WheelCommand command;
    const double half_wb = config_.wheelbase_ / 2.0;
    const double half_tw = config_.track_width_ / 2.0;
    
    const double wheel_x[4] = { half_wb,  half_wb, -half_wb, -half_wb };
    const double wheel_y[4] = { half_tw, -half_tw,  half_tw, -half_tw };

    // ICR is perfectly in the center of the robot (X = 0.0)
    for (int i = 0; i < 4; ++i) {
        const double wheel_v_x = velocity_x_mps - velocity_omega_rad_s * wheel_y[i];
        const double wheel_v_y = velocity_omega_rad_s * (wheel_x[i] - 0.0); 

        command.steering_angle_rad[i] = std::atan2(wheel_v_y, wheel_v_x);
        command.drive_velocity_mps[i] = std::hypot(wheel_v_x, wheel_v_y);
    }

    double ackermann_limit_rad;
    ackermann_limit_rad = std::atan2(
         1.0 * config_.wheelbase_, 
        (2.0 * config_.min_steering_radius_) - config_.track_width_
    );

    normalizeSteerAngle(command);
     optimizeSteerAngle(command);
        clampSteerAngle(command, ackermann_limit_rad);

    enforceDriveVelocityLimit(command);

    return command;
}

WheelCommand KinematicsSolver::computeFrontOnlyAckermann(
    double velocity_x_mps, 
    double velocity_omega_rad_s
) const {
    WheelCommand command;
    const double half_wb = config_.wheelbase_ / 2.0;
    const double half_tw = config_.track_width_ / 2.0;
    
    const double wheel_x[4] = { half_wb,  half_wb, -half_wb, -half_wb };
    const double wheel_y[4] = { half_tw, -half_tw,  half_tw, -half_tw };

    // ICR is locked to the rear axle (X = -half_wb)
    for (int i = 0; i < 4; ++i) {
        const double wheel_v_x = velocity_x_mps - velocity_omega_rad_s * wheel_y[i];
        const double wheel_v_y = velocity_omega_rad_s * (wheel_x[i] - (-half_wb)); 

        command.steering_angle_rad[i] = std::atan2(wheel_v_y, wheel_v_x);
        command.drive_velocity_mps[i] = std::hypot(wheel_v_x, wheel_v_y);
    }

    double ackermann_limit_rad;
    ackermann_limit_rad = std::atan2(
         2.0 * config_.wheelbase_, 
        (2.0 * config_.min_steering_radius_) - config_.track_width_
    );

    normalizeSteerAngle(command);
     optimizeSteerAngle(command);
        clampSteerAngle(command, ackermann_limit_rad);

    enforceDriveVelocityLimit(command);

    return command;
}

WheelCommand KinematicsSolver::computeRearOnlyAckermann(
    double velocity_x_mps, 
    double velocity_omega_rad_s
) const {
    WheelCommand command;
    const double half_wb = config_.wheelbase_ / 2.0;
    const double half_tw = config_.track_width_ / 2.0;
    
    const double wheel_x[4] = { half_wb,  half_wb, -half_wb, -half_wb };
    const double wheel_y[4] = { half_tw, -half_tw,  half_tw, -half_tw };

    // ICR is locked to the front axle (X = half_wb)
    for (int i = 0; i < 4; ++i) {
        const double wheel_v_x = velocity_x_mps - velocity_omega_rad_s * wheel_y[i];
        const double wheel_v_y = velocity_omega_rad_s * (wheel_x[i] - half_wb); 

        command.steering_angle_rad[i] = std::atan2(wheel_v_y, wheel_v_x);
        command.drive_velocity_mps[i] = std::hypot(wheel_v_x, wheel_v_y);
    }

    double ackermann_limit_rad;
    ackermann_limit_rad = std::atan2(
         2.0 * config_.wheelbase_, 
        (2.0 * config_.min_steering_radius_) - config_.track_width_
    );

    normalizeSteerAngle(command);
     optimizeSteerAngle(command);
        clampSteerAngle(command, ackermann_limit_rad);

    enforceDriveVelocityLimit(command);

    return command;
}

WheelCommand KinematicsSolver::computeCrab(
    double velocity_x_mps, 
    double velocity_y_mps
) const {
    WheelCommand command;
    const double half_wb = config_.wheelbase_ / 2.0;
    const double half_tw = config_.track_width_ / 2.0;
    
    const double wheel_x[4] = { half_wb,  half_wb, -half_wb, -half_wb };
    const double wheel_y[4] = { half_tw, -half_tw,  half_tw, -half_tw };

    const double omega = 0.0; 

    for (int i = 0; i < 4; ++i) {
        const double wheel_v_x = velocity_x_mps - (omega * wheel_y[i]);
        const double wheel_v_y = velocity_y_mps + (omega * wheel_x[i]); 

        command.steering_angle_rad[i] = std::atan2(wheel_v_y, wheel_v_x);
        command.drive_velocity_mps[i] = std::hypot(wheel_v_x, wheel_v_y);
    }

    normalizeSteerAngle(command);
     optimizeSteerAngle(command);
        
    enforceDriveVelocityLimit(command);

    return command;
}

WheelCommand KinematicsSolver::computeSymmetricSpin(
    double velocity_omega_rad_s
) const {
    WheelCommand command;
    const double half_wb = config_.wheelbase_ / 2.0;
    const double half_tw = config_.track_width_ / 2.0;
    
    const double wheel_x[4] = { half_wb,  half_wb, -half_wb, -half_wb };
    const double wheel_y[4] = { half_tw, -half_tw,  half_tw, -half_tw };

    const double v_x = 0.0;
    const double v_y = 0.0;
    
    const double omega = velocity_omega_rad_s;

    for (int i = 0; i < 4; ++i) {
        const double wheel_v_x = v_x - (omega * wheel_y[i]);
        const double wheel_v_y = v_y + (omega * wheel_x[i]); 

        command.steering_angle_rad[i] = std::atan2(wheel_v_y, wheel_v_x);
        command.drive_velocity_mps[i] = std::hypot(wheel_v_x, wheel_v_y);
    }

    normalizeSteerAngle(command);
     optimizeSteerAngle(command);
     
    enforceDriveVelocityLimit(command);

    return command;
}

WheelCommand KinematicsSolver::computeXConfiguration() const {
    WheelCommand command;

    const double half_wb = config_.wheelbase_ / 2.0;
    const double half_tw = config_.track_width_ / 2.0;

    const double wheel_x[4] = { half_wb,  half_wb, -half_wb, -half_wb };
    const double wheel_y[4] = { half_tw, -half_tw,  half_tw, -half_tw };

    for (int i = 0; i < 4; ++i) {
        const double wheel_v_x = -wheel_x[i];
        const double wheel_v_y = -wheel_y[i];

        command.steering_angle_rad[i] = std::atan2(wheel_v_y, wheel_v_x);
        command.drive_velocity_mps[i] = 0.0;
    }

    return command;
}