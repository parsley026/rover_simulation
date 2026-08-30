#include "rover_autonomy/subsystems/composite_subsystem_manager.hpp"

namespace rover_autonomy
{

CompositeSubsystemManager::CompositeSubsystemManager(rclcpp_lifecycle::LifecycleNode * parent_node)
: parent_node_(parent_node)
{
}

bool CompositeSubsystemManager::on_configure()
{
  for (auto & child : child_processes_) {
    if (!child->on_configure()) {
      RCLCPP_ERROR(parent_node_->get_logger(), "CompositeSubsystemManager: child configure failed.");
      return false;
    }
  }
  return true;
}

bool CompositeSubsystemManager::on_activate()
{
  for (auto & child : child_processes_) {
    if (!child->on_activate()) {
      RCLCPP_ERROR(parent_node_->get_logger(), "CompositeSubsystemManager: child activate failed.");
      return false;
    }
  }
  return true;
}

bool CompositeSubsystemManager::on_deactivate()
{
  bool success = true;
  for (auto & child : child_processes_) {
    if (!child->on_deactivate()) {
      success = false;
    }
  }
  return success;
}

bool CompositeSubsystemManager::on_cleanup()
{
  bool success = true;
  for (auto & child : child_processes_) {
    if (!child->on_cleanup()) {
      success = false;
    }
  }
  return success;
}

SubsystemState CompositeSubsystemManager::get_state() const
{
  if (!is_enabled()) {
    return SubsystemState::INACTIVE;
  }
  for (const auto & child : child_processes_) {
    if (child->is_enabled() && child->get_state() != SubsystemState::ACTIVE) {
      return SubsystemState::INACTIVE;
    }
  }
  return SubsystemState::ACTIVE;
}

bool CompositeSubsystemManager::is_healthy() const
{
  for (const auto & child : child_processes_) {
    if (child->is_enabled() && !child->is_healthy()) {
      return false;
    }
  }
  return true;
}

bool CompositeSubsystemManager::is_enabled() const
{
  for (const auto & child : child_processes_) {
    if (child->is_enabled()) {
      return true;
    }
  }
  return false;
}

void CompositeSubsystemManager::set_enabled(bool enabled)
{
  for (auto & child : child_processes_) {
    child->set_enabled(enabled);
  }
}

bool CompositeSubsystemManager::recover()
{
  bool all_recovered = true;
  for (auto & child : child_processes_) {
    if (child->is_enabled() && !child->is_healthy()) {
      if (!child->recover()) {
        all_recovered = false;
      }
    }
  }
  return all_recovered;
}

} // namespace rover_autonomy
