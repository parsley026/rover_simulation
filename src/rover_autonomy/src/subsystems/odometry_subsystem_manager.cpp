#include "rover_autonomy/subsystems/odometry_subsystem_manager.hpp"

#include <chrono>
#include <thread>

namespace rover_autonomy
{

// ─────────────────────────────────────────────────────────────────────────────
// Concrete inner process classes — private to this translation unit.
// Each class implements is_healthy() and recover() for its specific process.
// The constructor is inherited directly from ProcessSubsystemManager.
//
// Note: ProcessSubsystemManager::is_healthy() and recover() are pure virtual,
// so the base class cannot be instantiated directly. Subclassing is required.
// ─────────────────────────────────────────────────────────────────────────────

class CameraOdomProcess : public ProcessSubsystemManager
{
public:
  using ProcessSubsystemManager::ProcessSubsystemManager;

  bool is_healthy() const override
  {
    // Healthy = process was launched and is still running (state == ACTIVE).
    return current_state_ == SubsystemState::ACTIVE;
  }

  bool recover() override
  {
    RCLCPP_WARN(parent_node_->get_logger(), "Recovering CameraOdomProcess (%s)...", launch_file_.c_str());
    on_deactivate();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    return on_activate();
  }
};

class CameraOdomHealthProcess : public ProcessSubsystemManager
{
public:
  using ProcessSubsystemManager::ProcessSubsystemManager;

  bool is_healthy() const override
  {
    return current_state_ == SubsystemState::ACTIVE;
  }

  bool recover() override
  {
    RCLCPP_WARN(parent_node_->get_logger(), "Recovering CameraOdomHealthProcess (%s)...", launch_file_.c_str());
    on_deactivate();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    return on_activate();
  }
};

class Camera01OdomProcess : public ProcessSubsystemManager
{
public:
  using ProcessSubsystemManager::ProcessSubsystemManager;

  bool is_healthy() const override
  {
    return current_state_ == SubsystemState::ACTIVE;
  }

  bool recover() override
  {
    RCLCPP_WARN(parent_node_->get_logger(), "Recovering Camera01OdomProcess (%s)...", launch_file_.c_str());
    on_deactivate();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    return on_activate();
  }
};

class Camera01OdomHealthProcess : public ProcessSubsystemManager
{
public:
  using ProcessSubsystemManager::ProcessSubsystemManager;

  bool is_healthy() const override
  {
    return current_state_ == SubsystemState::ACTIVE;
  }

  bool recover() override
  {
    RCLCPP_WARN(parent_node_->get_logger(), "Recovering Camera01OdomHealthProcess (%s)...", launch_file_.c_str());
    on_deactivate();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    return on_activate();
  }
};

class LidarOdomProcess : public ProcessSubsystemManager
{
public:
  using ProcessSubsystemManager::ProcessSubsystemManager;

  bool is_healthy() const override
  {
    // Healthy = process was launched and is still running (state == ACTIVE).
    return current_state_ == SubsystemState::ACTIVE;
  }

  bool recover() override
  {
    RCLCPP_WARN(parent_node_->get_logger(), "Recovering LidarOdomProcess (%s)...", launch_file_.c_str());
    on_deactivate();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    return on_activate();
  }
};

class LidarOdomHealthProcess : public ProcessSubsystemManager
{
public:
  using ProcessSubsystemManager::ProcessSubsystemManager;

  bool is_healthy() const override
  {
    return current_state_ == SubsystemState::ACTIVE;
  }

  bool recover() override
  {
    RCLCPP_WARN(parent_node_->get_logger(), "Recovering LidarOdomHealthProcess (%s)...", launch_file_.c_str());
    on_deactivate();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    return on_activate();
  }
};

class ExtendedKalmanFilterProcess : public ProcessSubsystemManager
{
public:
  using ProcessSubsystemManager::ProcessSubsystemManager;

  bool is_healthy() const override
  {
    // Healthy = process was launched and is still running (state == ACTIVE).
    return current_state_ == SubsystemState::ACTIVE;
  }

  bool recover() override
  {
    RCLCPP_WARN(parent_node_->get_logger(), "Recovering ExtendedKalmanFilterProcess (%s)...", launch_file_.c_str());
    on_deactivate();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    return on_activate();
  }
};

// ─────────────────────────────────────────────────────────────────────────────
// OdometrySubsystemManager — Composite implementation
// Activation order: sensor → watchdog → filter (guarantees pipe is ready)
// ─────────────────────────────────────────────────────────────────────────────

OdometrySubsystemManager::OdometrySubsystemManager(
  rclcpp_lifecycle::LifecycleNode * parent_node,
  const OdometryConfig & config)
: CompositeSubsystemManager(parent_node)
{
  child_processes_.push_back(std::make_unique<CameraOdomProcess>(parent_node,         config.camera_odom));
  child_processes_.push_back(std::make_unique<CameraOdomHealthProcess>(parent_node,   config.camera_odom_health));
  child_processes_.push_back(std::make_unique<Camera01OdomProcess>(parent_node,       config.camera_01_odom));
  child_processes_.push_back(std::make_unique<Camera01OdomHealthProcess>(parent_node, config.camera_01_odom_health));
  child_processes_.push_back(std::make_unique<LidarOdomProcess>(parent_node,          config.lidar_odom));
  child_processes_.push_back(std::make_unique<LidarOdomHealthProcess>(parent_node,    config.lidar_odom_health));
  child_processes_.push_back(std::make_unique<ExtendedKalmanFilterProcess>(parent_node, config.localization));

  RCLCPP_INFO(parent_node_->get_logger(),
    "OdometrySubsystemManager created. "
    "camera_odom=%s (health=%s), camera_01_odom=%s (health=%s), lidar_odom=%s (health=%s), localization=%s",
    config.camera_odom.launch_enabled          ? "enabled" : "disabled",
    config.camera_odom_health.launch_enabled   ? "enabled" : "disabled",
    config.camera_01_odom.launch_enabled       ? "enabled" : "disabled",
    config.camera_01_odom_health.launch_enabled ? "enabled" : "disabled",
    config.lidar_odom.launch_enabled           ? "enabled" : "disabled",
    config.lidar_odom_health.launch_enabled    ? "enabled" : "disabled",
    config.localization.launch_enabled         ? "enabled" : "disabled");
}

}  // namespace rover_autonomy
