#ifndef ROVER_AUTONOMY__COMPOSITE_SUBSYSTEM_MANAGER_HPP_
#define ROVER_AUTONOMY__COMPOSITE_SUBSYSTEM_MANAGER_HPP_

#include "rover_autonomy/subsystems/subsystem_manager.hpp"
#include "rover_autonomy/subsystems/process_subsystem_manager.hpp"
#include <vector>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace rover_autonomy
{

class CompositeSubsystemManager : public SubsystemManager
{
public:
  CompositeSubsystemManager(rclcpp_lifecycle::LifecycleNode * parent_node);
  virtual ~CompositeSubsystemManager() = default;

  bool on_configure() override;
  bool on_activate() override;
  bool on_deactivate() override;
  bool on_cleanup() override;

  SubsystemState get_state() const override;
  bool is_healthy() const override;
  bool is_enabled() const override;
  bool recover() override;

protected:
  rclcpp_lifecycle::LifecycleNode * parent_node_;
  std::vector<std::unique_ptr<ProcessSubsystemManager>> child_processes_;
};

} // namespace rover_autonomy

#endif // ROVER_AUTONOMY__COMPOSITE_SUBSYSTEM_MANAGER_HPP_
