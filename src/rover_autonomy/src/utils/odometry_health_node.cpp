#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_srvs/srv/empty.hpp>
#include <rtabmap_msgs/srv/reset_pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "rover_autonomy/utils/stream_health_monitor.hpp"

#include <cmath>
#include <memory>
#include <string>

using rover_autonomy::utils::StreamHealthMonitor;

class OdometryHealthNode : public rclcpp::Node {
public:
  OdometryHealthNode() : Node("odometry_health_node"), consecutive_rejections_(0) {
    load_parameters();
    
    health_monitor_ = std::make_unique<StreamHealthMonitor<nav_msgs::msg::Odometry>>(
      config_.expected_hz, config_.timeout_sec);

    // ROS 2 Interfaces
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom_raw", rclcpp::SystemDefaultsQoS(),
      std::bind(&OdometryHealthNode::odom_callback, this, std::placeholders::_1));

    safe_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    
    if (config_.reset_mode == "twist") {
      reset_twist_client_ = this->create_client<std_srvs::srv::Empty>(config_.reset_service_name);
    } else if (config_.reset_mode == "pose") {
      reset_pose_client_ = this->create_client<rtabmap_msgs::srv::ResetPose>(config_.reset_service_name);
    }
    last_reset_time_ = this->now();
  }

private:
  // ==========================================
  // CONFIGURATION & STATE
  // ==========================================
  struct Config {
    double expected_hz;
    double timeout_sec;
    double max_lin_vel;
    double max_ang_vel;
    double max_cov_trace;
    double max_jump_dist;
    int max_consecutive_rejections;
    std::string reset_mode;
    double reset_cooldown_sec;
    std::string reset_service_name;
  } config_;

  enum class HealthState { UNKNOWN, OK, WARN, ERROR };
  HealthState current_health_state_ = HealthState::UNKNOWN;

  std::unique_ptr<StreamHealthMonitor<nav_msgs::msg::Odometry>> health_monitor_;
  int consecutive_rejections_;
  bool has_last_pose_ = false;
  geometry_msgs::msg::Pose last_pose_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr safe_odom_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;

  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_twist_client_;
  rclcpp::Client<rtabmap_msgs::srv::ResetPose>::SharedPtr reset_pose_client_;
  rclcpp::Time last_reset_time_;

  // ==========================================
  // INITIALIZATION
  // ==========================================
  void load_parameters() {
    this->declare_parameter("expected_hz", 30.0);
    this->declare_parameter("timeout_sec", 0.5);
    this->declare_parameter("max_lin_vel", 5.0);
    this->declare_parameter("max_ang_vel", 3.0);
    this->declare_parameter("max_cov_trace", 10.0);
    this->declare_parameter("max_consecutive_rejections", 15);
    this->declare_parameter("max_jump_dist", 1.5);
    this->declare_parameter("reset_mode", "twist");
    this->declare_parameter("reset_cooldown_sec", 2.0);
    this->declare_parameter("reset_service_name", "/camera_00/reset_odom");

    config_.expected_hz = this->get_parameter("expected_hz").as_double();
    config_.timeout_sec = this->get_parameter("timeout_sec").as_double();
    config_.max_lin_vel = this->get_parameter("max_lin_vel").as_double();
    config_.max_ang_vel = this->get_parameter("max_ang_vel").as_double();
    config_.max_cov_trace = this->get_parameter("max_cov_trace").as_double();
    config_.max_jump_dist = this->get_parameter("max_jump_dist").as_double();
    config_.max_consecutive_rejections = this->get_parameter("max_consecutive_rejections").as_int();
    config_.reset_mode = this->get_parameter("reset_mode").as_string();
    config_.reset_cooldown_sec = this->get_parameter("reset_cooldown_sec").as_double();
    config_.reset_service_name = this->get_parameter("reset_service_name").as_string();
  }

  // ==========================================
  // MAIN CALLBACK (The Orchestrator)
  // ==========================================
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // 1. Temporal Check
    if (!process_temporal_health(msg->header.stamp)) {
      return; // Stream is dead, drop message
    }

    // 2. Mathematical Sanity Check
    if (!is_mathematically_sane(msg)) {
      handle_rejection();
      return; // Data corrupted or physically impossible, drop message
    }

    // 3. Transformation & Publishing
    auto safe_msg = std::make_shared<nav_msgs::msg::Odometry>(*msg);
    apply_covariance_scaling(safe_msg);
    publish_safe_odometry(safe_msg);
  }

  // ==========================================
  // PIPELINE STEPS
  // ==========================================
  bool process_temporal_health(const rclcpp::Time& stamp) {
    health_monitor_->tick(stamp, this->now());
    int8_t status = health_monitor_->get_health_status(this->now());

    if (status == diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
      update_diagnostics(HealthState::ERROR, "Odometry stream dead or severely lagging.", status);
      return false;
    } else if (status == diagnostic_msgs::msg::DiagnosticStatus::WARN) {
      update_diagnostics(HealthState::WARN, "Odometry stream lagging.", status);
    }
    return true;
  }

  bool is_mathematically_sane(const nav_msgs::msg::Odometry::SharedPtr msg) const {
    if (has_nans(msg)) return false;
    if (!is_quaternion_valid(msg->pose.pose.orientation)) return false;
    if (!is_kinematically_valid(msg)) return false;
    if (!is_spatially_continuous(msg)) return false;
    if (!is_covariance_sane(msg)) return false;
    
    return true;
  }

  void handle_rejection() {
    consecutive_rejections_++;
    
    if (consecutive_rejections_ > config_.max_consecutive_rejections) {
      update_diagnostics(HealthState::ERROR, "Odometry mathematically insane. Auto-resetting...", 
                         diagnostic_msgs::msg::DiagnosticStatus::ERROR);
                         
      if ((this->now() - last_reset_time_).seconds() > config_.reset_cooldown_sec) {
        
        if (config_.reset_mode == "twist") {
          // --- CAMERA RECOVERY (Twist) ---
          if (reset_twist_client_->service_is_ready()) {
            auto request = std::make_shared<std_srvs::srv::Empty::Request>();
            reset_twist_client_->async_send_request(request);
            RCLCPP_WARN(this->get_logger(), "Sent twist reset to %s", config_.reset_service_name.c_str());
          } else {
            RCLCPP_ERROR(this->get_logger(), "Reset service %s not available!", config_.reset_service_name.c_str());
          }
          
        } else if (config_.reset_mode == "pose") {
          // --- LIDAR RECOVERY (Pose) ---
          if (!has_last_pose_) {
            RCLCPP_ERROR(this->get_logger(), "Cannot reset pose: No valid historical pose exists yet!");
            return; // Abort reset to protect the UKF
          }
          
          if (reset_pose_client_->service_is_ready()) {
            auto request = std::make_shared<rtabmap_msgs::srv::ResetPose::Request>();
            request->x = last_pose_.position.x;
            request->y = last_pose_.position.y;
            request->z = last_pose_.position.z;
            
            tf2::Quaternion q(
                last_pose_.orientation.x,
                last_pose_.orientation.y,
                last_pose_.orientation.z,
                last_pose_.orientation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            
            request->roll = roll;
            request->pitch = pitch;
            request->yaw = yaw;

            reset_pose_client_->async_send_request(request);
            RCLCPP_WARN(this->get_logger(), "Sent pose reset to %s", config_.reset_service_name.c_str());
          } else {
            RCLCPP_ERROR(this->get_logger(), "Reset service %s not available!", config_.reset_service_name.c_str());
          }
        }

        last_reset_time_ = this->now();
        consecutive_rejections_ = 0; // Reset counter to give sensor time to boot
      }
    }
  }

  void publish_safe_odometry(const nav_msgs::msg::Odometry::SharedPtr safe_msg) {
    consecutive_rejections_ = 0;
    update_diagnostics(HealthState::OK, "Odometry tracking OK.", diagnostic_msgs::msg::DiagnosticStatus::OK);
    
    safe_odom_pub_->publish(*safe_msg);
    
    last_pose_ = safe_msg->pose.pose;
    has_last_pose_ = true;
  }

  void apply_covariance_scaling(nav_msgs::msg::Odometry::SharedPtr msg) const {
    double lin_vel = calculate_3d_linear_velocity(msg);

    // Soft reject: If moving erratically near limits, lower EKF trust
    if (lin_vel > (config_.max_lin_vel * 0.75)) {
      for (int i = 0; i < 6; ++i) {
        msg->pose.covariance[i * 6 + i] *= 15.0; 
        msg->twist.covariance[i * 6 + i] *= 15.0;
      }
    }
  }

  // ==========================================
  // VALIDATION SUB-ROUTINES (SRP)
  // ==========================================
  bool has_nans(const nav_msgs::msg::Odometry::SharedPtr msg) const {
    if (std::isnan(msg->pose.pose.position.x) || std::isnan(msg->pose.pose.position.y) || std::isnan(msg->pose.pose.position.z)) return true;
    if (std::isnan(msg->twist.twist.linear.x) || std::isnan(msg->twist.twist.linear.y) || std::isnan(msg->twist.twist.linear.z)) return true;
    for (int i = 0; i < 36; ++i) {
      if (std::isnan(msg->pose.covariance[i]) || std::isnan(msg->twist.covariance[i])) return true;
    }
    return false;
  }

  bool is_quaternion_valid(const geometry_msgs::msg::Quaternion& q) const {
    double q_norm_sq = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    return (q_norm_sq >= 0.01 && std::abs(q_norm_sq - 1.0) <= 0.1);
  }

  bool is_kinematically_valid(const nav_msgs::msg::Odometry::SharedPtr msg) const {
    double lin_vel = calculate_3d_linear_velocity(msg);
    double ang_vel = calculate_3d_angular_velocity(msg);
    return (lin_vel <= config_.max_lin_vel && ang_vel <= config_.max_ang_vel);
  }

  bool is_spatially_continuous(const nav_msgs::msg::Odometry::SharedPtr msg) const {
    if (!has_last_pose_) return true; // Nothing to compare against yet

    double dx = msg->pose.pose.position.x - last_pose_.position.x;
    double dy = msg->pose.pose.position.y - last_pose_.position.y;
    double dz = msg->pose.pose.position.z - last_pose_.position.z;
    double jump_dist = std::sqrt(dx*dx + dy*dy + dz*dz);
    
    return jump_dist <= config_.max_jump_dist;
  }

  bool is_covariance_sane(const nav_msgs::msg::Odometry::SharedPtr msg) const {
    double pose_trace = 0.0;
    for (int i = 0; i < 6; ++i) {
      pose_trace += msg->pose.covariance[i * 6 + i];
    }
    // Reject RTAB-Map "lost" (too high) or infinite confidence (too low)
    return (pose_trace <= config_.max_cov_trace && pose_trace >= 1e-6);
  }

  // ==========================================
  // MATH & DIAGNOSTIC HELPERS
  // ==========================================
  double calculate_3d_linear_velocity(const nav_msgs::msg::Odometry::SharedPtr msg) const {
    return std::sqrt(std::pow(msg->twist.twist.linear.x, 2) + 
                     std::pow(msg->twist.twist.linear.y, 2) + 
                     std::pow(msg->twist.twist.linear.z, 2));
  }

  double calculate_3d_angular_velocity(const nav_msgs::msg::Odometry::SharedPtr msg) const {
    return std::sqrt(std::pow(msg->twist.twist.angular.x, 2) + 
                     std::pow(msg->twist.twist.angular.y, 2) + 
                     std::pow(msg->twist.twist.angular.z, 2));
  }

  void update_diagnostics(HealthState new_state, const std::string& message, int8_t level) {
    // Only publish if the state changed to prevent spamming the network at 30Hz
    if (new_state != current_health_state_) {
      current_health_state_ = new_state;
      
      if (level == diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
        RCLCPP_ERROR(this->get_logger(), "HEALTH FAULT: %s", message.c_str());
      } else if (level == diagnostic_msgs::msg::DiagnosticStatus::WARN) {
        RCLCPP_WARN(this->get_logger(), "HEALTH WARN: %s", message.c_str());
      } else {
        RCLCPP_INFO(this->get_logger(), "HEALTH RECOVERED: %s", message.c_str());
      }
      
      diagnostic_msgs::msg::DiagnosticArray diag_array;
      diag_array.header.stamp = this->now();

      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = "odometry_health_node: vo_lost";
      status.level = level;
      status.message = message;
      status.hardware_id = "odometry";

      diag_array.status.push_back(status);
      diag_pub_->publish(diag_array);
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryHealthNode>());
  rclcpp::shutdown();
  return 0;
}
