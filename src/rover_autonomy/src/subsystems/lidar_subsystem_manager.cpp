#include "rover_autonomy/subsystems/lidar_subsystem_manager.hpp"
#include <chrono>
#include <thread>

namespace rover_autonomy
{

class LidarProcess : public ProcessSubsystemManager
{
public:
  using ProcessSubsystemManager::ProcessSubsystemManager;

  bool is_healthy() const override
  {
    return current_state_ == SubsystemState::ACTIVE;
  }

  bool recover() override
  {
    RCLCPP_WARN(parent_node_->get_logger(), "Recovering lidar subsystem...");
    on_deactivate();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    return on_activate();
  }
};

LidarSubsystemManager::LidarSubsystemManager(
  rclcpp_lifecycle::LifecycleNode * parent_node,
  const SubsystemConfig & config)
: CompositeSubsystemManager(parent_node)
{
  child_processes_.push_back(std::make_unique<LidarProcess>(parent_node, config));
}

}  // namespace rover_autonomy
