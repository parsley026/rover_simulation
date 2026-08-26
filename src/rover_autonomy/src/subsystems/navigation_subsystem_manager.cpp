#include "rover_autonomy/subsystems/navigation_subsystem_manager.hpp"
#include <chrono>
#include <thread>

namespace rover_autonomy
{

class NavigationProcess : public ProcessSubsystemManager
{
public:
  using ProcessSubsystemManager::ProcessSubsystemManager;

  bool is_healthy() const override
  {
    if (current_state_ != SubsystemState::ACTIVE) {
      return false;
    }
    std::string ns = launch_arguments_.count("namespace") ? launch_arguments_.at("namespace") : "";
    std::string topic = ns.empty() ? "/navigate_to_pose/_action/status" : "/" + ns + "/navigate_to_pose/_action/status";
    size_t publishers = parent_node_->count_publishers(topic);
    return publishers > 0;
  }

  bool recover() override
  {
    RCLCPP_WARN(parent_node_->get_logger(), "Recovering navigation subsystem...");
    on_deactivate();
    std::this_thread::sleep_for(std::chrono::seconds(2));
    return on_activate();
  }
};

NavigationSubsystemManager::NavigationSubsystemManager(
  rclcpp_lifecycle::LifecycleNode * parent_node,
  const SubsystemConfig & config)
: CompositeSubsystemManager(parent_node)
{
  child_processes_.push_back(std::make_unique<NavigationProcess>(parent_node, config));
}

}  // namespace rover_autonomy
