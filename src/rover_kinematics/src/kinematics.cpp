#include "quad_rover_kinematics/kinematics.h"

Kinematics::Kinematics()
: odom_frame_id_("/odom"),
  base_frame_id_("/base_link"),
  W_(0), L_(0),
  timestamp_(0.0),
  x_(0.0),
  y_(0.0),
  max_steering_radius_(2.0),
  min_steering_radius_(1.0),
  heading_(0.0),
  linear_x_(0.0),
  linear_y_(0.0),
  angular_(0.0),
  thetha_spin(0.0),
  min_erpm_(0.0)
{
    RCLCPP_INFO(rclcpp::get_logger("quad_rover_kinematics"),
                "Kinematics node default-constructed");
}

Kinematics::Kinematics(double width, double length, std::vector<double> pose_covariance, std::vector<double> twist_covariance)
: odom_frame_id_("/odom"),
  base_frame_id_("/base_link"),
  W_(width), L_(length),
  pose_covariance_diagonal_(pose_covariance),
  twist_covariance_diagonal_(twist_covariance),
  timestamp_(0.0),
  max_steering_radius_(2.0),
  min_steering_radius_(1.0),
  x_(0.0),
  y_(0.0),
  heading_(0.0),
  linear_x_(0.0),
  linear_y_(0.0),
  angular_(0.0),
  thetha_spin(0.0),
  min_erpm_(0.0)
{

    setOdometryParam();
    setTFParam();
    calculateTrajectoryParams();
    thetha_spin = tangent360(L_ / 2, W_ / 2);

    RCLCPP_INFO(rclcpp::get_logger("quad_rover_kinematics"), "Kinematics initialized with width=%f, length=%f", width, length);
}

// ---------------- Getters -----------------
/**
 * @brief Gets the current heading in radians.
 * @return The heading angle in radians.
 */
double Kinematics::getHeading() { return heading_; }

/**
 * @brief Gets the current x position of the robot.
 * @return The x coordinate in meters.
 */
double Kinematics::getX() { return x_; }

/**
 * @brief Gets the current y position of the robot.
 * @return The y coordinate in meters.
 */
double Kinematics::getY() { return y_; }

/**
 * @brief Gets the current linear velocity along the x-axis.
 * @return The x-axis linear velocity in m/s.
 */
double Kinematics::getLinearX() { return linear_x_; }

/**
 * @brief Gets the current linear velocity along the y-axis.
 * @return The y-axis linear velocity in m/s.
 */
double Kinematics::getLinearY() { return linear_y_; }

/**
 * @brief Gets the current angular velocity around the z-axis.
 * @return The angular velocity in rad/s.
 */
double Kinematics::getAngular() { return angular_; }
/**
 * @brief Retrieves the latest computed odometry data.
 *
 * @details Provides the current position, orientation, and velocity
 *          information for the robotic system.
 *
 * @return The current odometry as a nav_msgs::msg::Odometry object.
 */
nav_msgs::msg::Odometry Kinematics::getOdom() { return odometry_; }
/**
 * @brief Retrieves the current transformation message for the Kinematics instance.
 *
 * @return A tf2_msgs::msg::TFMessage representing the current transformation data.
 */
tf2_msgs::msg::TFMessage Kinematics::getTF() { return transformation_; }
/**
 * @brief Sets the length and width of the robot.
 * @param length The length between front and back wheels.
 * @param width The width between left and right wheels.
 */
void Kinematics::setLengthWidth(const double length, const double width){
    L_ = length;
    W_ = width;
}
/**
 * @brief Sets the pose covariance diagonal values.
 * @param pose_covariance A vector containing the pose covariance diagonal values.
 */
void Kinematics::setPoseCovariance(const std::vector<double> pose_covariance){
    pose_covariance_diagonal_ = pose_covariance;
}
/**
 * @brief Sets the twist covariance diagonal values.
 * @param twist_covariance A vector containing the twist covariance diagonal values.
 */
void Kinematics::setTwistCovariance(const std::vector<double> twist_covariance){
    twist_covariance_diagonal_ = twist_covariance;
}
void Kinematics::setSteeringRadius(const double min_radius, const double max_radius){
    min_steering_radius_ = min_radius;
    max_steering_radius_ = max_radius;
}
void Kinematics::setMinERPM(const double min_erpm) {
    min_erpm_ = min_erpm;
}
/**
 * @brief Sets the frame IDs for odometry and base.
 * @param odom_frame_id The frame ID for odometry.
 * @param base_frame_id The frame ID for the base.
 */
void Kinematics::setFrames(const std::string odom_frame_id, const std::string base_frame_id){
    odom_frame_id_ = odom_frame_id;
    base_frame_id_ = base_frame_id;
}
/**
 * @brief Sets the radius of the wheels.
 * @param radius The radius of the wheels.
 */
void Kinematics::setWheelRadius(const double radius){
    wheel_radius_ = radius;
}
/**
 * @brief Sets the number of pole pairs in the motor.
 * @param poles The number of pole pairs.
 */
void Kinematics::setPolesPairsNumber(const int poles){
    poles_pairs_number_ = poles;
}
/**
 * @brief Sets the gear ratio of the motor.
 * @param ratio The gear ratio.
 */
void Kinematics::setMotorGearRatio(const double ratio){
    motor_gear_ratio_ = ratio;
}
/**
 * @brief Retrieves the current steering angles from the given wheel feedback.
 *
 * Extracts the steering angles for the front-left, front-right,
 * rear-right, and rear-left wheels.
 *
 * @param feedback Wheel feedback message containing steering angles.
 * @return A vector of size 4 with the angles in the order:
 *         front-left, front-right, rear-right, rear-left.
 */
std::vector<double> getCurrentThethas(const rex_interfaces::msg::Wheels &feedback)
{
    std::vector<double> thethas(4);
    thethas[0] = feedback.front_left.turn.set_value;
    thethas[1] = feedback.front_right.turn.set_value;
    thethas[2] = feedback.rear_right.turn.set_value;
    thethas[3] = feedback.rear_left.turn.set_value;
    return thethas;
}

/**
 * @brief Resets the odometry to its initial state.
 *
 * This function resets the internal kinematic states of the robot, including
 * the position (x, y), heading, and linear and angular velocities. It also
 * logs a message indicating that the odometry has been reset.
 */
void Kinematics::resetOdometry()
{
    // Reset relevant internal kinematic states
    x_ = 0;
    y_ = 0;
    heading_ = 0;

    linear_x_ = 0;
    linear_y_ = 0;
    angular_  = 0;

    RCLCPP_INFO(rclcpp::get_logger("Kinematics"), "Odometry has been reset");
}
/**
 * @brief Converts the given y and x values to a tangent angle in the range [-180, 180].
 *
 * This function calculates the tangent angle in degrees from the given y and x values.
 * The angle is normalized to the range [-180, 180] degrees.
 *
 * @param y The y-coordinate value.
 * @param x The x-coordinate value.
 * @return The tangent angle in degrees.
 */
double Kinematics::tangent360(double y, double x) {
    double angle_rad = std::atan2(y, x);

    if (angle_rad > M_PI / 2) angle_rad -= M_PI;
    if (angle_rad < -M_PI / 2) angle_rad += M_PI;

    return angle_rad * 180.0 / M_PI;
}

// ---------------- Linear params for kinematcis -----------------

/**
 * @brief Calculates the parameters for the steering trajectory.
 * 
 * This function calculates the linear parameters 'a' and 'b' for the steering radius
 * based on the maximum and minimum steering radius values. The relationship is defined as:
 * 
 * MAX_STEER_RADIUS = a * 0.01 + b
 * MIN_STEER_RADIUS = a * 1 + b
 * 
 * The function solves these equations to determine the values of 'a' and 'b', and then
 * uses these values to linearize the input from the joystick to the steering radius.
 * 
 * The calculated parameters are stored in the member variables `radius_a_ratio` and `radius_b_ratio`.
 */
void Kinematics::calculateTrajectoryParams()
{
    this->radius_a_ratio = (MIN_STEER_RADIOUS - MAX_STEER_RADIOUS) / 0.99;
    this->radius_b_ratio = MIN_STEER_RADIOUS - radius_a_ratio;
}

// ---------------- Valid fnuctions -----------------

/**
 * @brief Checks whether the wheel turn angles are within a specified threshold.
 *
 * Compares each wheel's turn angle against a center angle ± threshold to determine if
 * they fall within the acceptable range.
 *
 * @param feedback The structure containing the turn angles of all wheels.
 * @param centerAngle The reference angle about which the threshold is evaluated.
 * @param threshold The allowable deviation on either side of the center angle.
 * @return true if all wheel angles are within the threshold; false otherwise.
 */
bool Kinematics::areWheelsInThreshold(const rex_interfaces::msg::Wheels &feedback,
                                      float centerAngle, float threshold)
{
    auto isWithinThreshold = [&](float angle) {
        return (angle < centerAngle + threshold && angle > centerAngle - threshold);
    };

    return isWithinThreshold(std::fabs(feedback.front_left.turn.set_value)) &&
           isWithinThreshold(std::fabs(feedback.front_right.turn.set_value)) &&
           isWithinThreshold(std::fabs(feedback.rear_right.turn.set_value)) &&
           isWithinThreshold(std::fabs(feedback.rear_left.turn.set_value));
}

/**
 * @brief Determines if the desired wheel turn angles are in the opposite direction
 *        compared to the current wheel turn angles.
 *
 * This function checks the sign of each pair of angles from the given vectors, ignoring
 * very small angle values (within a certain precision range). If any angle pair has
 * opposite signs (one positive, one negative), it returns true, indicating an opposite turn.
 *
 * @param thetas_goal Vector of desired wheel turn angles.
 * @param thetas_current Vector of current wheel turn angles.
 * @return True if any wheel turn angles are opposite in sign, otherwise false.
 */
bool isOppositeTurn(std::vector<double> &thetas_goal, std::vector<double> &thetas_current)
{
    double precision = 5.0;
    for (size_t i = 0; i < thetas_current.size(); i++)
    {
        if (thetas_current[i] < precision && thetas_current[i] > -precision)
        {
            thetas_current[i] = 0.0;
            continue;
        }
        if (std::signbit(thetas_current[i]) != std::signbit(thetas_goal[i])) {
            return true;
        }
    }
    return false;
}




/**
 * @brief Applies brake command by setting all drive outputs to zero and their respective turning commands to idle.
 *
 * This function prepares a Wheels message to stop the rover by commanding zero current control
 * and zero step motor movement for each wheel. The output message contains header information
 * and control values used for quick braking.
 *
 * @param time The current ROS2 time stamp used to set the header stamp of the Wheels message.
 * @return A Wheels message containing the necessary commands for braking all wheels.
 */
rex_interfaces::msg::Wheels Kinematics::brake(const rclcpp::Time &time)
{
    rex_interfaces::msg::Wheels wheel_velocities;

    wheel_velocities.header.stamp = time;

    // TO DO:
    // Should I use VESC_COMMAND_SET_CURRENT_BRAKE 2 or
    // VESC_COMMAND_SET_CURRENT_HANDBRAKE 12 ???

    wheel_velocities.front_left.drive.command_id = CURRENT_CONTROL;
    wheel_velocities.front_left.turn.command_id = STEP_MOTOR_CONTROL;
    wheel_velocities.front_left.turn.set_value = 0.0;

    wheel_velocities.front_right.drive.command_id = CURRENT_CONTROL;
    wheel_velocities.front_right.turn.command_id = STEP_MOTOR_CONTROL;
    wheel_velocities.front_right.turn.set_value = 0.0;

    wheel_velocities.rear_right.drive.command_id = CURRENT_CONTROL;
    wheel_velocities.rear_right.turn.command_id = STEP_MOTOR_CONTROL;
    wheel_velocities.rear_right.turn.set_value = 0.0;

    wheel_velocities.rear_left.drive.command_id = CURRENT_CONTROL;
    wheel_velocities.rear_left.turn.command_id = STEP_MOTOR_CONTROL;
    wheel_velocities.rear_left.turn.set_value = 0.0;

    wheel_velocities.front_left.drive.set_value = 0.0;
    wheel_velocities.front_right.drive.set_value = 0.0;
    wheel_velocities.rear_right.drive.set_value = 0.0;
    wheel_velocities.rear_left.drive.set_value = 0.0;

    return wheel_velocities;
}


/**
 * @brief Computes the wheel velocities and steering angles for advancing in a curved path.
 *
 * This function calculates the required wheel velocities and steering angles to follow a curved path
 * based on the given radius and drive speed. It takes into account the current wheel feedback and
 * adjusts the wheel velocities and angles accordingly.
 *
 * @param time The current ROS2 time stamp used to set the header stamp of the Wheels message.
 * @param radius The radius of the curved path.
 * @param drive The drive speed.
 * @param feedback The current wheel feedback containing the steering angles.
 * @return A Wheels message containing the computed wheel velocities and steering angles.
 */
rex_interfaces::msg::Wheels Kinematics::advanceRPMKinematics(const rclcpp::Time &time, 
const double &radius, const double &drive, const rex_interfaces::msg::Wheels& feedback)
{
    const double const_value = 30.0 / wheel_radius_ / M_PI * poles_pairs_number_ * motor_gear_ratio_;
    
    std::vector<double> velocities = {0, 0, 0, 0};
    std::vector<double> thetas_goal = {0, 0, 0, 0};
    std::vector<double> thetas_current = {0, 0, 0, 0};

    rex_interfaces::msg::Wheels vel;

    thetas_current = getCurrentThethas(feedback);

    if (radius != 0.0)
    {

        // TODO: change the sign of the radius to match the direction of the turn
        
        double sign_radius = (radius > 0.0) ? 1.0 : -1.0;
        double R = 0;

        R = radius_a_ratio * std::abs(radius) + radius_b_ratio;
        
        thetas_goal[0] = sign_radius * tangent360(L_, 2*R - W_ * sign_radius);
        thetas_goal[1] = sign_radius * tangent360(L_, 2*R + W_ * sign_radius); //(-1)*
        thetas_goal[2] = (-1) * thetas_goal[0];
        thetas_goal[3] = (-1) * thetas_goal[1]; //(-1)*

        //Problem wth function below
        //if (!areWheelsInThreshold(feedback, thetas_goal, WHEEL_STEER_ERROR))
        //{
        //    if (isOppositeTurn(thetas_goal, thetas_current))
        //    {
        //        for (int i = 0; i < 4; i++)
        //        {
        //            thetas_goal[i] = 0.0;
        //        }
        //    }
        //}

        if (drive != 0.0)
        {
            double omega = drive / std::abs(R);
            double sign_drive = (drive > 0.0) ? 1.0 : -1.0;

            double Rout = std::sqrt(std::pow(R + W_ / 2, 2) + std::pow(L_ / 2, 2));
            double Rinn = std::sqrt(std::pow(R - W_ / 2, 2) + std::pow(L_ / 2, 2));

           

            if(sign_radius > 0){
                velocities[0] = Rinn * omega * const_value + sign_drive * min_erpm_;
                velocities[1] = Rout * omega * const_value + sign_drive * min_erpm_;
                velocities[2] = velocities[0];
                velocities[3] = velocities[1];
            }
            else{
                velocities[0] = Rout * omega * const_value + sign_drive * min_erpm_;
                velocities[1] = Rinn * omega * const_value + sign_drive * min_erpm_; 
                velocities[2] = velocities[0];
                velocities[3] = velocities[1];
            }
            
        }
        else
        {
            velocities[0] = velocities[1] = velocities[2] = velocities[3] = 0.0;
        }
    }
    else
    {
        thetas_goal[0] = thetas_goal[1] = thetas_goal[2] = thetas_goal[3] = 0.0;

        if (drive != 0.0)
        {

            double speed = drive * 30.0 / wheel_radius_ / M_PI * poles_pairs_number_ * motor_gear_ratio_;
            double sign_drive = (drive > 0.0) ? 1.0 : -1.0;
            
            velocities[0] = velocities[1] = velocities[2] = velocities[3] = speed + sign_drive * min_erpm_;

        }
        else
        {
            velocities[0] = velocities[1] = velocities[2] = velocities[3] = 0.0;
        }
    }

    vel.header.stamp = time;

    auto setWheelParams = [](rex_interfaces::msg::Wheel &wheel, double value, double angle) {
        wheel.drive.set_value = value;
        wheel.turn.set_value = angle;
        wheel.turn.command_id = STEP_MOTOR_CONTROL;
        wheel.drive.command_id = RPM_CONTROL;
    };

    setWheelParams(vel.front_left, velocities[0], thetas_goal[0]);
    setWheelParams(vel.front_right, velocities[1], thetas_goal[1]);
    setWheelParams(vel.rear_left, velocities[2], thetas_goal[2]);
    setWheelParams(vel.rear_right, velocities[3], thetas_goal[3]);

    return vel;
}

/**
 * @brief Computes the wheel velocities and steering angles for crab drive mode.
 *
 * This function calculates the required wheel velocities and steering angles to follow a straight path
 * in any direction based on the given vector components and drive speed. It takes into account the current
 * wheel feedback and adjusts the wheel velocities and angles accordingly.
 *
 * @param time The current ROS2 time stamp used to set the header stamp of the Wheels message.
 * @param vectorX The x component of the direction vector.
 * @param vectorY The y component of the direction vector.
 * @param drive The drive speed.
 * @param feedback The current wheel feedback containing the steering angles.
 * @return A Wheels message containing the computed wheel velocities and steering angles.
 */
rex_interfaces::msg::Wheels Kinematics::crabDriveKinematics(const rclcpp::Time &time,
const double &vectorX, const double &vectorY, double drive, const rex_interfaces::msg::Wheels& /*feedback*/)
{
    rex_interfaces::msg::Wheels vel;
    const double const_value = 30.0 / wheel_radius_ / M_PI * poles_pairs_number_ * motor_gear_ratio_;
    double axisY = vectorY;
    double axisX = vectorX;
    
    // Clamp negative Y-axis and compute steering angle
    if (vectorY < 0.0) {
        axisY = 0.0;
    }
    double theta = 0.0;
    if (std::fabs(axisX) < 1e-6 && std::fabs(axisY) < 1e-6) {
        theta = 0.0;
    } else {
        theta = std::atan2(axisX, axisY) * (180.0 / M_PI);
    }

    // Compute wheel speed based on drive input
    double speed = drive * const_value;

    // Lambda to set wheel parameters
    auto setWheelParams = [](rex_interfaces::msg::Wheel &wheel, double value, double angle) {
        wheel.drive.set_value = value;
        wheel.turn.set_value = angle;
        wheel.turn.command_id = STEP_MOTOR_CONTROL;
        wheel.drive.command_id = RPM_CONTROL;
    };

    vel.header.stamp = time;

    // Apply same steering angle and speed to all wheels
    setWheelParams(vel.front_left,  speed, theta);
    setWheelParams(vel.front_right, speed, theta); //(-1)*
    setWheelParams(vel.rear_left,   speed, theta);
    setWheelParams(vel.rear_right,  speed, theta); //(-1)*

    return vel;
}

/**
 * @brief Computes the wheel velocities and steering angles for spin drive mode.
 *
 * This function calculates the required wheel velocities and steering angles to spin the robot
 * around its center based on the given rotational speed. It takes into account the current
 * wheel feedback and adjusts the wheel velocities and angles accordingly.
 *
 * @param time The current ROS2 time stamp used to set the header stamp of the Wheels message.
 * @param rot_Z The rotational speed around the Z-axis.
 * @param feedback The current wheel feedback containing the steering angles.
 * @return A Wheels message containing the computed wheel velocities and steering angles.
 */
rex_interfaces::msg::Wheels Kinematics::spinDriveKinematics(const rclcpp::Time &time,
const double &rot_Z, const rex_interfaces::msg::Wheels& feedback)
{
    rex_interfaces::msg::Wheels vel;
    const double const_value = 30.0 / wheel_radius_ / M_PI * poles_pairs_number_ * motor_gear_ratio_;

    //TODO : Check the actual steering angle for spin drive are achieved

    double speed = rot_Z * const_value;

    auto setWheelParams = [](rex_interfaces::msg::Wheel &wheel, double value, double angle) {
        wheel.drive.set_value = value;
        wheel.turn.set_value = angle;
        wheel.turn.command_id = STEP_MOTOR_CONTROL;
        wheel.drive.command_id = RPM_CONTROL;
    };

    vel.header.stamp = time;

    if (std::fabs(speed) < 1e-6) { // Negative rotation
        setWheelParams(vel.front_left, speed, -thetha_spin);
        setWheelParams(vel.front_right, -speed, thetha_spin); //(-1)*
        setWheelParams(vel.rear_left, speed, thetha_spin);
        setWheelParams(vel.rear_right, -speed, -thetha_spin); //(-1)*
    } else { // Positive rotation
        setWheelParams(vel.front_left, -speed, -thetha_spin);
        setWheelParams(vel.front_right, speed, thetha_spin); //(-1)*
        setWheelParams(vel.rear_left, -speed, thetha_spin);
        setWheelParams(vel.rear_right, speed, -thetha_spin); //(-1)*
    }

    return vel;
}


/**
 * @brief Sets the odometry parameters for the Kinematics object.
 *
 * This function initializes the odometry parameters including the frame IDs,
 * pose position, pose covariance, and twist covariance. It sets the frame IDs
 * for the odometry header and child frame, sets the z position of the pose to 0,
 * and assigns the pose and twist covariance matrices using the provided diagonal
 * values.
 *
 * The pose covariance matrix is set using the values from the pose_covariance_diagonal_
 * array, and the twist covariance matrix is set using the values from the
 * twist_covariance_diagonal_ array. The linear y and z components of the twist,
 * as well as the angular x and y components of the twist, are set to 0.
 */
void Kinematics::setOdometryParam() {

    odometry_.header.frame_id = odom_frame_id_;
    odometry_.child_frame_id = base_frame_id_;
    odometry_.pose.pose.position.z = 0.0;

    odometry_.pose.covariance = {
        pose_covariance_diagonal_[0], 0., 0., 0., 0., 0.,
        0., pose_covariance_diagonal_[1], 0., 0., 0., 0.,
        0., 0., pose_covariance_diagonal_[2], 0., 0., 0.,
        0., 0., 0., pose_covariance_diagonal_[3], 0., 0.,
        0., 0., 0., 0., pose_covariance_diagonal_[4], 0.,
        0., 0., 0., 0., 0., pose_covariance_diagonal_[5]
    };

    odometry_.twist.twist.linear.y = 0.0;
    odometry_.twist.twist.linear.z = 0.0;
    odometry_.twist.twist.angular.x = 0.0;
    odometry_.twist.twist.angular.y = 0.0;
    odometry_.twist.covariance = {
        twist_covariance_diagonal_[0], 0., 0., 0., 0., 0.,
        0., twist_covariance_diagonal_[1], 0., 0., 0., 0.,
        0., 0., twist_covariance_diagonal_[2], 0., 0., 0.,
        0., 0., 0., twist_covariance_diagonal_[3], 0., 0.,
        0., 0., 0., 0., twist_covariance_diagonal_[4], 0.,
        0., 0., 0., 0., 0., twist_covariance_diagonal_[5]
    };
}

/**
 * @brief Sets the transformation parameters for the kinematics.
 * 
 * This function initializes the transformation parameters by resizing the 
 * transforms vector to hold one transform. It sets the translation in the z-axis 
 * to 0.0, assigns the child frame ID to the base frame ID, and sets the header 
 * frame ID to the odom frame ID.
 */
void Kinematics::setTFParam()
{
    transformation_.transforms.resize(1);

    transformation_.transforms[0].transform.translation.z = 0.0;
    transformation_.transforms[0].child_frame_id = base_frame_id_;
    transformation_.transforms[0].header.frame_id = odom_frame_id_;
}


/**
 * @brief Updates the odometry based on the current time and wheel feedback.
 *
 * This function computes the change in position and orientation of the robot
 * using the feedback from the wheels and the elapsed time since the last update.
 * It uses the Ceres solver to optimize the local velocities and angular velocity
 * of the robot, and then integrates these velocities to update the robot's pose.
 *
 * @param time The current time.
 * @param feedback The feedback from the wheels, containing the drive and turn set values.
 * @return True if the odometry was successfully updated, false otherwise.
 */
bool Kinematics::updateOdometry(const rclcpp::Time &time,
                                const rex_interfaces::msg::Wheels &feedback)
{
    //rclcpp::Time current_time = use_sim_time_ ? sim_time_ : time;

    // 1. Compute dt
    double dt = (rclcpp::Time(time.nanoseconds(), RCL_ROS_TIME) -
                 rclcpp::Time(timestamp_.nanoseconds(), RCL_ROS_TIME)).seconds();
    if (dt <= 0.0) {
        RCLCPP_INFO(rclcpp::get_logger("quad_rover_kinematics"), "ABORT: dt <= 0.0");
        return false;
    }
    timestamp_ = time;
    //timestamp_ = current_time;

    // ... (Logging feedback, etc.)

    // 2. Convert steering angles to radians
    auto toRad = [](double deg) {
    if (std::abs(deg) > 0.5) {
            return deg * M_PI / 180.0; 
        }
        return 0.0; // Default return value for other cases
    };
    double frRad = toRad(feedback.front_right.turn.set_value);
    double flRad = toRad(feedback.front_left.turn.set_value);
    double rlRad = toRad(feedback.rear_left.turn.set_value);
    double rrRad = toRad(feedback.rear_right.turn.set_value);

    // 3. Compute local velocities for each wheel
    auto wheelVel = [&](double speed, double steer_rad) {
        return std::make_pair(speed * std::cos(steer_rad), speed * std::sin(steer_rad));
    };
    auto vFR = wheelVel(feedback.front_right.drive.set_value, frRad);
    auto vFL = wheelVel(feedback.front_left.drive.set_value, flRad);
    auto vRL = wheelVel(feedback.rear_left.drive.set_value, rlRad);
    auto vRR = wheelVel(feedback.rear_right.drive.set_value, rrRad);

    // 4. Wheel positions (x_i, y_i)
    double xFR = +L_/2.0;
    double yFR = -W_/2.0;
    double xFL = +L_/2.0;
    double yFL = +W_/2.0;
    double xRL = -L_/2.0;
    double yRL = +W_/2.0;
    double xRR = -L_/2.0;
    double yRR = -W_/2.0;

    // 5. Set up the Ceres problem
    ceres::Problem problem;

    // ---- Instead of double v_x=0, v_y=0, omega=0; we do: ----
    // Use the stored guesses from the last iteration
    double v_x = v_x_guess_;
    double v_y = v_y_guess_;
    double omega = omega_guess_;

    // Add residuals for each wheel
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<WheelResidual, 2, 1, 1, 1>(
            new WheelResidual(vFR.first, vFR.second, xFR, yFR)),
        nullptr, &v_x, &v_y, &omega);
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<WheelResidual, 2, 1, 1, 1>(
            new WheelResidual(vFL.first, vFL.second, xFL, yFL)),
        nullptr, &v_x, &v_y, &omega);
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<WheelResidual, 2, 1, 1, 1>(
            new WheelResidual(vRL.first, vRL.second, xRL, yRL)),
        nullptr, &v_x, &v_y, &omega);
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<WheelResidual, 2, 1, 1, 1>(
            new WheelResidual(vRR.first, vRR.second, xRR, yRR)),
        nullptr, &v_x, &v_y, &omega);

    // 6. Solve the problem
    ceres::Solver::Options options;
    options.max_num_iterations = 200;
    options.function_tolerance = 1e-8;
    options.gradient_tolerance = 1e-10;
    options.parameter_tolerance = 1e-9;
    options.linear_solver_type = ceres::DENSE_QR;
    options.trust_region_strategy_type = ceres::DOGLEG;
    // options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // (Optional) Log the summary if needed:
    // RCLCPP_INFO(rclcpp::get_logger("quad_rover_kinematics"), "%s", summary.BriefReport().c_str());

    // ---- After solving, store the solution for next iteration ----
    v_x_guess_ = v_x;
    v_y_guess_ = v_y;
    omega_guess_ = omega;

    // 7. Transform and integrate odometry
    double dx_local = v_x * dt;
    double dy_local = v_y * dt;

    double cHeading = std::cos(heading_);
    double sHeading = std::sin(heading_);

    x_ += dx_local * cHeading - dy_local * sHeading;
    y_ += dx_local * sHeading + dy_local * cHeading;

    // 4th-order Runge–Kutta integration for heading (or just heading_ += omega*dt)
    // double k1 = omega * dt;
    // double k2 = (omega + 0.5 * k1) * dt;
    // double k3 = (omega + 0.5 * k2) * dt;
    // double k4 = (omega + k3) * dt;
    // heading_ += (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;

    heading_ += omega * dt;

    // 8. Fill nav_msgs::msg::Odometry
    odometry_.header.stamp = time;
    odometry_.header.frame_id = "odom";
    odometry_.child_frame_id = "base_link";

    odometry_.pose.pose.position.x = x_;
    odometry_.pose.pose.position.y = y_;
    odometry_.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, heading_);
    odometry_.pose.pose.orientation = tf2::toMsg(q);

    odometry_.twist.twist.linear.x = v_x;
    odometry_.twist.twist.linear.y = v_y;
    odometry_.twist.twist.angular.z = omega;

    transformation_.transforms[0].header.stamp = time;
    transformation_.transforms[0].transform.translation.x = x_;
    transformation_.transforms[0].transform.translation.y = y_;
    transformation_.transforms[0].transform.rotation = tf2::toMsg(q);

    return true;
}



