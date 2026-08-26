#ifndef ROVER_AUTONOMY__PROCESS_SUBSYSTEM_MANAGER_HPP_
#define ROVER_AUTONOMY__PROCESS_SUBSYSTEM_MANAGER_HPP_

#include "rover_autonomy/subsystems/subsystem_manager.hpp"
#include "rover_autonomy/parameter_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <map>
#include <string>
#include <sys/types.h>

namespace rover_autonomy
{

class ProcessSubsystemManager : public SubsystemManager
{
public:
  ProcessSubsystemManager(
    rclcpp_lifecycle::LifecycleNode * parent_node,
    const SubsystemConfig & config);

  virtual ~ProcessSubsystemManager();

  void add_launch_argument(const std::string& key, const std::string& value);

  bool on_configure()  override;
  bool on_activate()   override;
  bool on_deactivate() override;
  bool on_cleanup()    override;

  SubsystemState get_state() const override;
  bool is_healthy() const override = 0;
  bool is_enabled() const override;

  bool recover() override = 0;

protected:
  rclcpp_lifecycle::LifecycleNode * parent_node_;
  SubsystemState current_state_{SubsystemState::UNCONFIGURED};

  std::string package_name_;
  std::string launch_file_;
  bool enabled_{true};
  std::map<std::string, std::string> launch_arguments_;

private:
  pid_t launch_pgid_{-1};

  bool start_process();
  bool stop_process();
};

}  // namespace rover_autonomy

#endif  // ROVER_AUTONOMY__PROCESS_SUBSYSTEM_MANAGER_HPP_