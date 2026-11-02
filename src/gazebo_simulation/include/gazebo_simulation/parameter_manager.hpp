#ifndef GAZEBO_SIMULATION__PARAMETER_MANAGER_HPP_
#define GAZEBO_SIMULATION__PARAMETER_MANAGER_HPP_

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <functional>
#include <string>
#include <vector>

namespace gazebo_simulation
{

/**
 * @brief Helper that registers a parameter-change callback on a LifecycleNode.
 *
 * Sensor-level enable/disable toggles have been intentionally removed.
 * Per-sensor activation is now controlled exclusively through lifecycle state
 * transitions (active ↔ inactive), keeping the control plane unified.
 * Only generic node parameters (use_sim_time, robot_name, world_name) are
 * managed here.
 */
class ParameterManager
{
public:
  using ParameterUpdateCallback = std::function<void()>;

  ParameterManager(
    rclcpp_lifecycle::LifecycleNode * node,
    ParameterUpdateCallback update_cb);
  ~ParameterManager() = default;

  void declare_all_parameters();

private:
  rcl_interfaces::msg::SetParametersResult on_parameters_updated(
    const std::vector<rclcpp::Parameter> & parameters);

  rclcpp_lifecycle::LifecycleNode * node_;
  ParameterUpdateCallback update_cb_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

}  // namespace gazebo_simulation

#endif  // GAZEBO_SIMULATION__PARAMETER_MANAGER_HPP_
