#include "rover_autonomy/subsystems/localization_and_mapping_subsystem_manager.hpp"
#include <chrono>
#include <thread>

namespace rover_autonomy
{

class MappingProcess : public ProcessSubsystemManager
{
public:
  using ProcessSubsystemManager::ProcessSubsystemManager;

  bool is_healthy() const override
  {
    return current_state_ == SubsystemState::ACTIVE;
  }

  bool recover() override
  {
    RCLCPP_WARN(parent_node_->get_logger(), "Recovering mapping subsystem...");
    on_deactivate();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    return on_activate();
  }
};

class LocalTopographyProcess : public ProcessSubsystemManager
{
public:
  using ProcessSubsystemManager::ProcessSubsystemManager;

  bool is_healthy() const override
  {
    return current_state_ == SubsystemState::ACTIVE;
  }

  bool recover() override
  {
    RCLCPP_WARN(parent_node_->get_logger(), "Recovering local topography subsystem...");
    on_deactivate();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    return on_activate();
  }
};

class GlobalTopographyProcess : public ProcessSubsystemManager
{
public:
  using ProcessSubsystemManager::ProcessSubsystemManager;

  bool is_healthy() const override
  {
    return current_state_ == SubsystemState::ACTIVE;
  }

  bool recover() override
  {
    RCLCPP_WARN(parent_node_->get_logger(), "Recovering global topography subsystem...");
    on_deactivate();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    return on_activate();
  }
};

LocalizationAndMappingSubsystemManager::LocalizationAndMappingSubsystemManager(
  rclcpp_lifecycle::LifecycleNode * parent_node,
  const MappingConfig & config)
: CompositeSubsystemManager(parent_node)
{
  child_processes_.push_back(std::make_unique<MappingProcess>(parent_node, config.mapping));
  child_processes_.push_back(std::make_unique<LocalTopographyProcess>(parent_node, config.local_topography));
  child_processes_.push_back(std::make_unique<GlobalTopographyProcess>(parent_node, config.global_topography));
  
  RCLCPP_INFO(parent_node_->get_logger(),
    "LocalizationAndMappingSubsystemManager created. Children: mapping=%s, local_topography=%s, global_topography=%s",
    config.mapping.launch_enabled ? "enabled" : "disabled",
    config.local_topography.launch_enabled ? "enabled" : "disabled",
    config.global_topography.launch_enabled ? "enabled" : "disabled");
}

SubsystemManager* LocalizationAndMappingSubsystemManager::get_mapping_subsystem()
{
  return child_processes_[0].get();
}

SubsystemManager* LocalizationAndMappingSubsystemManager::get_local_topography_subsystem()
{
  return child_processes_[1].get();
}

SubsystemManager* LocalizationAndMappingSubsystemManager::get_global_topography_subsystem()
{
  return child_processes_[2].get();
}

}  // namespace rover_autonomy
