#include "gazebo_simulation/parameter_manager.hpp"

namespace gazebo_simulation
{

ParameterManager::ParameterManager(
  rclcpp_lifecycle::LifecycleNode * node,
  ParameterUpdateCallback update_cb)
: node_(node), update_cb_(std::move(update_cb))
{
}

void ParameterManager::declare_all_parameters()
{
  if (!node_->has_parameter("use_sim_time")) {
    node_->declare_parameter<bool>("use_sim_time", false);
  }

  param_callback_handle_ = node_->add_on_set_parameters_callback(
    std::bind(&ParameterManager::on_parameters_updated, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult ParameterManager::on_parameters_updated(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  bool changed = false;
  for (const auto & param : parameters) {
    const std::string & name = param.get_name();
    if (name == "use_sim_time" || name == "robot_name" || name == "world_name") {
      changed = true;
    }
  }

  if (changed && update_cb_) {
    update_cb_();
  }

  return result;
}

}  // namespace gazebo_simulation
