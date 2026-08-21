#include "rover_autonomy/localization_and_mapping_subsystem_manager.hpp"
#include <chrono>
#include <thread>

namespace rover_autonomy
{

class LocalizationAndMappingProcess : public ProcessSubsystemManager
{
public:
  using ProcessSubsystemManager::ProcessSubsystemManager;

  bool is_healthy() const override
  {
    // For now, healthy means the process is ACTIVE.
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

LocalizationAndMappingSubsystemManager::LocalizationAndMappingSubsystemManager(
  rclcpp_lifecycle::LifecycleNode * parent_node,
  const SubsystemConfig & config)
: CompositeSubsystemManager(parent_node)
{
  child_processes_.push_back(std::make_unique<LocalizationAndMappingProcess>(parent_node, config));
}

}  // namespace rover_autonomy
