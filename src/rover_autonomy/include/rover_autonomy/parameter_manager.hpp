#ifndef ROVER_AUTONOMY__PARAMETER_MANAGER_HPP_
#define ROVER_AUTONOMY__PARAMETER_MANAGER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include <functional>
#include <map>
#include <memory>
#include <string>

namespace rover_autonomy
{

// A clean struct to hold everything a Subsystem needs to boot.
// Produced by the ParameterManager (COO) and consumed by the CEO and Subsystems.
struct SubsystemConfig {
  bool        launch_enabled{false};
  std::string package_name;
  std::string launch_script;
  std::map<std::string, std::string> launch_args;
};

// Typed contract handed from COO to Camera subsystem.
struct CameraConfig {
  SubsystemConfig driver;
  SubsystemConfig decompression;
  SubsystemConfig post_processing;
};

// Typed contract handed from COO to Odometry composite subsystem.
struct OdometryConfig {
  SubsystemConfig camera_odom;           // camera_00 rtabmap_odom
  SubsystemConfig camera_odom_health;    // camera_00 health watchdog (optional)
  SubsystemConfig camera_01_odom;        // camera_01 rtabmap_odom    (was unmanaged)
  SubsystemConfig camera_01_odom_health; // camera_01 health watchdog (optional)
  SubsystemConfig lidar_odom;            // lidar_00  rtabmap_odom
  SubsystemConfig lidar_odom_health;     // lidar_00  health watchdog (optional)
  SubsystemConfig localization;
};

// Typed contract handed from COO to Mapping composite subsystem.
struct MappingConfig {
  SubsystemConfig mapping;
  SubsystemConfig local_topography;
  SubsystemConfig global_topography;
};

// Callback the ParameterManager uses to alert the CEO that a config changed.
// The CEO wires in a recovery lambda at construction time.
using ParameterUpdateCallback = std::function<void(const std::string & subsystem_name)>;


class ParameterManager
{
public:
  // node: raw pointer to the parent LifecycleNode (CEO).
  // update_cb: lambda the CEO provides to handle runtime config changes.
  ParameterManager(
    rclcpp_lifecycle::LifecycleNode * node,
    ParameterUpdateCallback update_cb);

  // Declares all parameters on the node's parameter server AND binds the
  // dynamic callback handle. Must be called once at node boot.
  void declare_all_parameters();

  // Returns a fully resolved OdometryConfig (camera odom + lidar odom + localization).
  // [[nodiscard]]: call once and store the result.
  [[nodiscard]] OdometryConfig get_odometry_config() const;

  // Returns a fully resolved CameraConfig for the camera subsystem.
  // [[nodiscard]]: call once and store the result.
  [[nodiscard]] CameraConfig get_camera_config() const;

  // Returns a fully resolved SubsystemConfig for the lidar subsystem.
  // [[nodiscard]]: call once and store the result.
  [[nodiscard]] SubsystemConfig get_lidar_config() const;

  // Returns a fully resolved MappingConfig for the mapping subsystem.
  // [[nodiscard]]: call once and store the result.
  [[nodiscard]] MappingConfig get_mapping_config() const;

  // Returns a fully resolved SubsystemConfig for the navigation subsystem.
  // [[nodiscard]]: call once and store the result.
  [[nodiscard]] SubsystemConfig get_navigation_config() const;

  // Returns a fully resolved SubsystemConfig for the description subsystem.
  // [[nodiscard]]: call once and store the result; never call repeatedly.
  [[nodiscard]] SubsystemConfig get_description_config() const;

private:
  rclcpp_lifecycle::LifecycleNode * node_;
  ParameterUpdateCallback alert_ceo_cb_;

  // Owned here — the CEO never touches this handle.
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // Resolves a package-relative path to an absolute filesystem path using
  // ament_index_cpp. Falls back to relative_path on failure.
  std::string resolve_share_path(
    const std::string & package,
    const std::string & relative_path) const;

  // Receives notifications from the ROS 2 parameter server when a param changes
  // at runtime, then calls alert_ceo_cb_ for subsystems that are affected.
  rcl_interfaces::msg::SetParametersResult on_parameters_updated(
    const std::vector<rclcpp::Parameter> & parameters);
};

}  // namespace rover_autonomy

#endif  // ROVER_AUTONOMY__PARAMETER_MANAGER_HPP_
