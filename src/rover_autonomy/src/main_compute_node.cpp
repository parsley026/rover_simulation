#include "rover_autonomy/main_compute_node.hpp"

#include <chrono>
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

namespace rover_autonomy
{

MainComputeNode::MainComputeNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("main_compute", options)
{
  RCLCPP_INFO(get_logger(), "MainComputeNode booting...");

  // 1. Boot the COO — wire in recovery lambda before declaring parameters
  param_manager_ = std::make_unique<ParameterManager>(this,
    [this](const std::string & subsystem_name) {
      on_subsystem_config_changed(subsystem_name);
    });
  param_manager_->declare_all_parameters();

  // 2. Fetch configs — called once, result stored; ParameterManager not queried again
  auto camera_config = param_manager_->get_camera_config();
  auto lidar_config = param_manager_->get_lidar_config();
  auto desc_config = param_manager_->get_description_config();
  auto odom_config = param_manager_->get_odometry_config();
  auto mapping_config = param_manager_->get_mapping_config();

  // 3. Instantiate subsystems — ALWAYS created; enabled_ governs behavior, not construction
  camera_subsystem_ = std::make_unique<CameraSubsystemManager>(this, camera_config);
  all_subsystems_.push_back(camera_subsystem_.get());

  lidar_subsystem_ = std::make_unique<LidarSubsystemManager>(this, lidar_config);
  all_subsystems_.push_back(lidar_subsystem_.get());

  description_subsystem_ = std::make_unique<DescriptionSubsystemManager>(this, desc_config);
  all_subsystems_.push_back(description_subsystem_.get());

  odometry_subsystem_ = std::make_unique<OdometrySubsystemManager>(this, odom_config);
  all_subsystems_.push_back(odometry_subsystem_.get());

  mapping_subsystem_ = std::make_unique<LocalizationAndMappingSubsystemManager>(this, mapping_config);
  all_subsystems_.push_back(mapping_subsystem_.get());

  RCLCPP_INFO(get_logger(), "MainComputeNode created. Subsystems registered: %zu", all_subsystems_.size());
}

void MainComputeNode::on_subsystem_config_changed(const std::string & subsystem_name)
{
  RCLCPP_WARN(get_logger(), "Config changed for subsystem '%s'. Triggering recovery...", subsystem_name.c_str());

  // [EXTENSION POINT]: Find the subsystem by name and call recover().
  // Example: if (subsystem_name == "description") { description_subsystem_->recover(); }
}

MainComputeNode::~MainComputeNode()
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MainComputeNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring MainComputeNode...");

  for (auto * sub : all_subsystems_) {
    if (!sub->on_configure()) {
      RCLCPP_ERROR(get_logger(), "Failed to configure a subsystem.");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
  }

  RCLCPP_INFO(get_logger(), "Configuration successful.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MainComputeNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating MainComputeNode...");

  for (auto * sub : all_subsystems_) {
    if (!sub->on_activate()) {
      RCLCPP_ERROR(get_logger(), "Failed to activate a subsystem.");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
  }

  RCLCPP_INFO(get_logger(), "Activation successful.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MainComputeNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating MainComputeNode...");

  for (auto * sub : all_subsystems_) {
    sub->on_deactivate();
  }

  RCLCPP_INFO(get_logger(), "Deactivation successful.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MainComputeNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up MainComputeNode...");

  for (auto * sub : all_subsystems_) {
    sub->on_cleanup();
  }

  RCLCPP_INFO(get_logger(), "Cleanup successful.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MainComputeNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down MainComputeNode...");

  for (auto * sub : all_subsystems_) {
    sub->on_deactivate();
    sub->on_cleanup();
  }

  RCLCPP_INFO(get_logger(), "Shutdown successful.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

RCLCPP_COMPONENTS_REGISTER_NODE(rover_autonomy::MainComputeNode)

}  // namespace rover_autonomy