#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <cmath>
#include <algorithm>
#include <vector>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <boost/function.hpp>
#include <boost/bind/bind.hpp>
#include <Eigen/Dense>
#include "rex_interfaces/msg/wheels.hpp"
#include "rex_interfaces/msg/vesc_status.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

using namespace boost::placeholders;

//wheel control modes
#define DUTY_CONTROL    0
#define CURRENT_CONTROL 1
#define RPM_CONTROL     3
#define SET_ORIGIN      5

//steer control modes
#define STEP_MOTOR_VELOCITY_CONTROL 3
#define STEP_MOTOR_CONTROL 4

#define MIN_STEER_RADIOUS 1.0
#define MAX_STEER_RADIOUS 5.0 

#define WHEEL_STEER_ERROR 2.5

class Kinematics
{
public:

    Kinematics();
    Kinematics(double width, double length, std::vector<double> pose_covariance, std::vector<double> twist_covariance);

    double getHeading();
    double getX();
    double getY();
    double getLinearX();
    double getLinearY();
    double getAngular();

    nav_msgs::msg::Odometry getOdom();
    tf2_msgs::msg::TFMessage getTF();

    void resetOdometry();

    void setLengthWidth(const double length, const double width);
    void setPoseCovariance(const std::vector<double> pose_covariance);
    void setTwistCovariance(const std::vector<double> twist_covariance);
    void setSteeringRadius(const double min_radius, const double max_radius);
    void setMinERPM(const double min_erpm);
    void setFrames(const std::string odom_frame_id, const std::string base_frame_id);
    void setWheelRadius(const double radius);
    void setPolesPairsNumber(const int poles);
    void setMotorGearRatio(const double ratio);

    void setOdometryParam();
    void setTFParam();
   
    bool areWheelsInThreshold(const rex_interfaces::msg::Wheels &feedback, float centerAngle, float threshold);

    rex_interfaces::msg::Wheels brake(const rclcpp::Time &time);
    rex_interfaces::msg::Wheels advanceRPMKinematics(const rclcpp::Time &time,const double &radius, const double &drive, const rex_interfaces::msg::Wheels& feedback);
    rex_interfaces::msg::Wheels crabDriveKinematics(const rclcpp::Time &time, const double &vectorX, const double &vectorY, double drive, const rex_interfaces::msg::Wheels& feedback);
    rex_interfaces::msg::Wheels spinDriveKinematics(const rclcpp::Time &time, const double &rot_Z, const rex_interfaces::msg::Wheels& feedback);
     bool updateOdometry(const rclcpp::Time &time, const rex_interfaces::msg::Wheels &feedback);

   

private:

    nav_msgs::msg::Odometry odometry_;
    tf2_msgs::msg::TFMessage transformation_;

    // Current timestamp:
    rclcpp::Time timestamp_;

    double max_steering_radius_;
    double min_steering_radius_;

    double x_;        // [m]
    double y_;        // [m]
    double heading_;  // [rad]

    /// Current velocity:
    double linear_x_;  // [m/s]
    double linear_y_;
    double angular_;  // [rad/s]

    double thetha_spin;
    double radius_a_ratio;
    double radius_b_ratio;
    double min_erpm_;

    double L_;
    double W_;
    double wheel_radius_;
    int poles_pairs_number_;
    double motor_gear_ratio_;

    double v_x_guess_ = 0.0;
    double v_y_guess_ = 0.0;
    double omega_guess_ = 0.0;

    std::string odom_frame_id_;
    std::string base_frame_id_;
    std::vector<double> pose_covariance_diagonal_;
    std::vector<double> twist_covariance_diagonal_;

    struct WheelResidual {
    WheelResidual(double v_xi, double v_yi, double x_i, double y_i)
        : v_xi_(v_xi), v_yi_(v_yi), x_i_(x_i), y_i_(y_i) {}

      template <typename T>
      bool operator()(const T* const v_x, const T* const v_y, const T* const omega, T* residual) const {
          // Compute residuals for the wheel
          residual[0] = v_xi_ - (*v_x - *omega * T(y_i_));
          residual[1] = v_yi_ - (*v_y + *omega * T(x_i_));
          return true;
      }
  
      private:
      const double v_xi_, v_yi_;  // Measured wheel velocities
      const double x_i_, y_i_;    // Wheel positions

    };

    // Helper function to convert radians to degrees
    constexpr double rad2deg(double radians) {
        return radians * (180.0 / M_PI);
    }

    void calculateTrajectoryParams();
    double tangent360(double y, double x);
};
