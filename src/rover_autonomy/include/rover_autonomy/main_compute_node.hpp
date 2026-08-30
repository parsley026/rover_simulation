#ifndef ROVER_AUTONOMY__MAIN_COMPUTE_NODE_HPP_
#define ROVER_AUTONOMY__MAIN_COMPUTE_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "rover_autonomy/parameter_manager.hpp"
#include "rover_autonomy/subsystems/camera_subsystem_manager.hpp"
#include "rover_autonomy/subsystems/lidar_subsystem_manager.hpp"
#include "rover_autonomy/subsystems/description_subsystem_manager.hpp"
#include "rover_autonomy/subsystems/odometry_subsystem_manager.hpp"
#include "rover_autonomy/subsystems/localization_and_mapping_subsystem_manager.hpp"
#include "rover_autonomy/subsystems/navigation_subsystem_manager.hpp"
#include "rover_autonomy/subsystems/subsystem_manager.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include <memory>
#include <vector>

namespace rover_autonomy
{

class MainComputeNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit MainComputeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~MainComputeNode();

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

private:
  // The COO — owns all parameter declaration and resolution
  std::unique_ptr<ParameterManager> param_manager_;

  // Subsystem instances — always created, enabled_ flag governs behavior
  std::unique_ptr<CameraSubsystemManager> camera_subsystem_;
  std::unique_ptr<LidarSubsystemManager> lidar_subsystem_;
  std::unique_ptr<DescriptionSubsystemManager> description_subsystem_;
  std::unique_ptr<OdometrySubsystemManager> odometry_subsystem_;
  std::unique_ptr<LocalizationAndMappingSubsystemManager> mapping_subsystem_;
  std::unique_ptr<NavigationSubsystemManager> navigation_subsystem_;

  // Supervisor loop roster — all subsystems registered here regardless of enabled state
  std::vector<SubsystemManager*> all_subsystems_;

  // Navigation service
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr navigation_service_;
  void on_set_navigation(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  // Mapping & Topography services
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr mapping_service_;
  void on_set_mapping(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr local_topography_service_;
  void on_set_local_topography(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr global_topography_service_;
  void on_set_global_topography(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  // Recovery callback fired by ParameterManager when a config param changes at runtime
  void on_subsystem_config_changed(const std::string & subsystem_name);
};

}  // namespace rover_autonomy

#endif  // ROVER_AUTONOMY__MAIN_COMPUTE_NODE_HPP_
