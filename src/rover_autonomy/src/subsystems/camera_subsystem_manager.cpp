#include "rover_autonomy/subsystems/camera_subsystem_manager.hpp"
#include <chrono>
#include <thread>

namespace rover_autonomy
{

class CameraProcess : public ProcessSubsystemManager
{
public:
  using ProcessSubsystemManager::ProcessSubsystemManager;

  bool is_healthy() const override
  {
    return current_state_ == SubsystemState::ACTIVE;
  }

  bool recover() override
  {
    RCLCPP_WARN(parent_node_->get_logger(), "Recovering camera subsystem...");
    on_deactivate();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    return on_activate();
  }
};

CameraSubsystemManager::CameraSubsystemManager(
  rclcpp_lifecycle::LifecycleNode * parent_node,
  const CameraConfig & config)
: CompositeSubsystemManager(parent_node)
{
  child_processes_.push_back(std::make_unique<CameraProcess>(parent_node, config.driver));
  child_processes_.push_back(std::make_unique<CameraProcess>(parent_node, config.decompression));
  child_processes_.push_back(std::make_unique<CameraProcess>(parent_node, config.post_processing));
}

}  // namespace rover_autonomy
