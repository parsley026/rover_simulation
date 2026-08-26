#include "rover_autonomy/subsystems/odometry_subsystem_manager.hpp"

#include <chrono>
#include <thread>

namespace rover_autonomy
{

// ─────────────────────────────────────────────────────────────────────────────
// Concrete inner process classes — private to this translation unit.
// Each class implements is_healthy() and recover() for its specific process.
// The constructor is inherited directly from ProcessSubsystemManager.
// ─────────────────────────────────────────────────────────────────────────────

class CameraOdomProcess : public ProcessSubsystemManager
{
public:
  using ProcessSubsystemManager::ProcessSubsystemManager;

  bool is_healthy() const override
  {
    // Healthy = process was launched and is still running (state == ACTIVE).
    // A topic-level heartbeat check (e.g., /camera_00/odom) can be added here later.
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

class LidarOdomProcess : public ProcessSubsystemManager
{
public:
  using ProcessSubsystemManager::ProcessSubsystemManager;

  bool is_healthy() const override
  {
    // Healthy = process was launched and is still running (state == ACTIVE).
    // A topic-level heartbeat check (e.g., /lidar_00/odom) can be added here later.
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

class ExtendedKalmanFilterProcess : public ProcessSubsystemManager
{
public:
  using ProcessSubsystemManager::ProcessSubsystemManager;

  bool is_healthy() const override
  {
    // Healthy = process was launched and is still running (state == ACTIVE).
    // A topic-level heartbeat check (e.g., /odometry/filtered) can be added here later.
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
// ─────────────────────────────────────────────────────────────────────────────

OdometrySubsystemManager::OdometrySubsystemManager(
  rclcpp_lifecycle::LifecycleNode * parent_node,
  const OdometryConfig & config)
: CompositeSubsystemManager(parent_node)
{
  child_processes_.push_back(std::make_unique<CameraOdomProcess>(parent_node, config.camera_odom));
  child_processes_.push_back(std::make_unique<LidarOdomProcess>(parent_node, config.lidar_odom));
  child_processes_.push_back(std::make_unique<ExtendedKalmanFilterProcess>(parent_node, config.localization));

  RCLCPP_INFO(parent_node_->get_logger(),
    "OdometrySubsystemManager created. Children: camera_odom=%s, lidar_odom=%s, localization=%s",
    config.camera_odom.launch_enabled  ? "enabled" : "disabled",
    config.lidar_odom.launch_enabled   ? "enabled" : "disabled",
    config.localization.launch_enabled ? "enabled" : "disabled");
}

}  // namespace rover_autonomy
