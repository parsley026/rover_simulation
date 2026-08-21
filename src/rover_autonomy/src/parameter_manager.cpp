#include "rover_autonomy/parameter_manager.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace rover_autonomy
{

ParameterManager::ParameterManager(
  rclcpp_lifecycle::LifecycleNode * node,
  ParameterUpdateCallback update_cb)
: node_(node), alert_ceo_cb_(std::move(update_cb))
{
}

void ParameterManager::declare_all_parameters()
{
  // --- Mode toggles ---
  // use_sim_time is pre-declared by ROS 2 LifecycleNode — guard against double-declare.
  if (!node_->has_parameter("use_sim_time")) {
    node_->declare_parameter<bool>("use_sim_time", true);
  }

  // --- Subsystem activation toggles ---
  node_->declare_parameter<bool>("launch_description",    true);
  node_->declare_parameter<bool>("launch_camera_00",      true);
  node_->declare_parameter<bool>("launch_lidar_00",       true);
  node_->declare_parameter<bool>("launch_camera_00_odom", true);
  node_->declare_parameter<bool>("launch_lidar_00_odom",  true);
  node_->declare_parameter<bool>("launch_localization",   true);
  node_->declare_parameter<bool>("launch_mapping",        true);

  // --- YAML-driven subsystem identifiers (Option B: fully data-driven) ---
  node_->declare_parameter<std::string>("description_package", "rover_autonomy");
  node_->declare_parameter<std::string>("description_launch",  "description.launch.py");

  // --- Subsystem configuration file paths (relative to package share) ---
  node_->declare_parameter<std::string>("camera_00_config",      "config/sensors/oak_d_pro.yaml");
  node_->declare_parameter<std::string>("lidar_00_config",       "config/sensors/ouster_os.yaml");
  node_->declare_parameter<std::string>("camera_00_odom_config", "config/odometry/rgbd_odom.yaml");
  node_->declare_parameter<std::string>("lidar_00_odom_config",  "config/odometry/icp_odom.yaml");

  node_->declare_parameter<std::string>("mapping_config",        "config/mapping/rtabmap_slam.yaml");
  node_->declare_parameter<std::string>("ekf_config",            "config/localization/ekf_filter_sim.yaml");

  // --- Camera subsystem identifiers ---
  node_->declare_parameter<std::string>("camera_package", "rover_autonomy");
  node_->declare_parameter<std::string>("camera_launch",  "luxonis_camera.launch.py");
  node_->declare_parameter<std::string>("camera_ns",      "camera_00");
  node_->declare_parameter<std::string>("camera_name",    "camera_00");
  node_->declare_parameter<std::string>("camera_parent_frame", "base_link");

  // --- Lidar subsystem identifiers ---
  node_->declare_parameter<std::string>("lidar_package", "rover_autonomy");
  node_->declare_parameter<std::string>("lidar_launch",  "ouster_lidar.launch.py");
  node_->declare_parameter<std::string>("lidar_ns",      "lidar_00");
  node_->declare_parameter<std::string>("lidar_name",    "lidar_00");
  node_->declare_parameter<std::string>("lidar_parent_frame", "base_link");

  // --- Odometry subsystem identifiers (Option B: fully YAML-driven) ---
  node_->declare_parameter<std::string>("camera_odom_package", "rover_autonomy");
  node_->declare_parameter<std::string>("camera_odom_launch",  "rgbd_odometry.launch.py");
  node_->declare_parameter<std::string>("camera_odom_ns",      "camera_00");

  node_->declare_parameter<std::string>("lidar_odom_package",  "rover_autonomy");
  node_->declare_parameter<std::string>("lidar_odom_launch",   "icp_odometry.launch.py");
  node_->declare_parameter<std::string>("lidar_odom_ns",       "lidar_00");

  node_->declare_parameter<std::string>("localization_package", "rover_autonomy");
  node_->declare_parameter<std::string>("localization_launch",  "localization.launch.py");
  node_->declare_parameter<std::string>("localization_type",    "ekf");

  node_->declare_parameter<std::string>("mapping_package",      "rover_autonomy");
  node_->declare_parameter<std::string>("mapping_launch",       "mapping.launch.py");
  node_->declare_parameter<bool>("delete_db_on_start",          true);


  // Bind the dynamic callback handle here — CEO never sees this machinery.
  param_callback_handle_ = node_->add_on_set_parameters_callback(
    std::bind(&ParameterManager::on_parameters_updated, this, std::placeholders::_1));
}

OdometryConfig ParameterManager::get_odometry_config() const
{
  bool use_sim_time = node_->get_parameter("use_sim_time").as_bool();
  std::string sim_time_str = use_sim_time ? "true" : "false";

  // Camera Odometry
  SubsystemConfig camera_odom;
  camera_odom.launch_enabled = node_->get_parameter("launch_camera_00_odom").as_bool();
  camera_odom.package_name   = node_->get_parameter("camera_odom_package").as_string();
  camera_odom.launch_script  = node_->get_parameter("camera_odom_launch").as_string();
  camera_odom.launch_args["use_sim_time"] = sim_time_str;
  camera_odom.launch_args["camera_ns"]    = node_->get_parameter("camera_odom_ns").as_string();
  camera_odom.launch_args["params_file"]  =
    resolve_share_path("rover_autonomy", node_->get_parameter("camera_00_odom_config").as_string());

  // Lidar Odometry
  SubsystemConfig lidar_odom;
  lidar_odom.launch_enabled = node_->get_parameter("launch_lidar_00_odom").as_bool();
  lidar_odom.package_name   = node_->get_parameter("lidar_odom_package").as_string();
  lidar_odom.launch_script  = node_->get_parameter("lidar_odom_launch").as_string();
  lidar_odom.launch_args["use_sim_time"] = sim_time_str;
  lidar_odom.launch_args["lidar_ns"]     = node_->get_parameter("lidar_odom_ns").as_string();
  lidar_odom.launch_args["params_file"]  =
    resolve_share_path("rover_autonomy", node_->get_parameter("lidar_00_odom_config").as_string());

  // Localization (EKF/UKF — type is YAML-driven, Python launch decides)
  SubsystemConfig localization;
  localization.launch_enabled = node_->get_parameter("launch_localization").as_bool();
  localization.package_name   = node_->get_parameter("localization_package").as_string();
  localization.launch_script  = node_->get_parameter("localization_launch").as_string();
  localization.launch_args["use_sim_time"]      = sim_time_str;
  localization.launch_args["localization_type"] = node_->get_parameter("localization_type").as_string();
  localization.launch_args["params_file"]       =
    resolve_share_path("rover_autonomy", node_->get_parameter("ekf_config").as_string());

  return OdometryConfig{camera_odom, lidar_odom, localization};
}

SubsystemConfig ParameterManager::get_camera_config() const
{
  SubsystemConfig config;
  config.launch_enabled = node_->get_parameter("launch_camera_00").as_bool();
  config.package_name   = node_->get_parameter("camera_package").as_string();
  config.launch_script  = node_->get_parameter("camera_launch").as_string();

  bool use_sim_time = node_->get_parameter("use_sim_time").as_bool();
  config.launch_args["use_sim_time"] = use_sim_time ? "true" : "false";

  config.launch_args["camera_ns"]           = node_->get_parameter("camera_ns").as_string();
  config.launch_args["camera_name"]         = node_->get_parameter("camera_name").as_string();
  config.launch_args["camera_parent_frame"] = node_->get_parameter("camera_parent_frame").as_string();

  config.launch_args["params_file"] =
    resolve_share_path("rover_autonomy", node_->get_parameter("camera_00_config").as_string());

  return config;
}

SubsystemConfig ParameterManager::get_lidar_config() const
{
  SubsystemConfig config;
  config.launch_enabled = node_->get_parameter("launch_lidar_00").as_bool();
  config.package_name   = node_->get_parameter("lidar_package").as_string();
  config.launch_script  = node_->get_parameter("lidar_launch").as_string();

  bool use_sim_time = node_->get_parameter("use_sim_time").as_bool();
  config.launch_args["use_sim_time"] = use_sim_time ? "true" : "false";

  config.launch_args["lidar_ns"]           = node_->get_parameter("lidar_ns").as_string();
  config.launch_args["lidar_name"]         = node_->get_parameter("lidar_name").as_string();
  config.launch_args["lidar_parent_frame"] = node_->get_parameter("lidar_parent_frame").as_string();

  config.launch_args["params_file"] =
    resolve_share_path("rover_autonomy", node_->get_parameter("lidar_00_config").as_string());

  return config;
}

SubsystemConfig ParameterManager::get_mapping_config() const
{
  SubsystemConfig config;
  config.launch_enabled = node_->get_parameter("launch_mapping").as_bool();
  config.package_name   = node_->get_parameter("mapping_package").as_string();
  config.launch_script  = node_->get_parameter("mapping_launch").as_string();

  bool use_sim_time = node_->get_parameter("use_sim_time").as_bool();
  config.launch_args["use_sim_time"] = use_sim_time ? "true" : "false";

  bool delete_db_on_start = node_->get_parameter("delete_db_on_start").as_bool();
  config.launch_args["delete_db_on_start"] = delete_db_on_start ? "true" : "false";

  config.launch_args["params_file"] =
    resolve_share_path("rover_autonomy", node_->get_parameter("mapping_config").as_string());

  return config;
}

SubsystemConfig ParameterManager::get_description_config() const
{
  SubsystemConfig config;
  config.launch_enabled = node_->get_parameter("launch_description").as_bool();
  config.package_name   = node_->get_parameter("description_package").as_string();
  config.launch_script  = node_->get_parameter("description_launch").as_string();

  bool use_sim_time = node_->get_parameter("use_sim_time").as_bool();
  config.launch_args["use_sim_time"] = use_sim_time ? "true" : "false";

  return config;
}

std::string ParameterManager::resolve_share_path(
  const std::string & package,
  const std::string & relative_path) const
{
  try {
    std::string pkg_share = ament_index_cpp::get_package_share_directory(package);
    return pkg_share + "/" + relative_path;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Failed to resolve share path for package '%s': %s. Using relative path.",
      package.c_str(), e.what());
    return relative_path;
  }
}

rcl_interfaces::msg::SetParametersResult ParameterManager::on_parameters_updated(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & param : parameters) {
    RCLCPP_INFO(node_->get_logger(), "Parameter updated: %s", param.get_name().c_str());

    // Alert the CEO when description-affecting params change at runtime.
    if (param.get_name() == "use_sim_time" ||
        param.get_name() == "launch_description" ||
        param.get_name() == "description_package" ||
        param.get_name() == "description_launch")
    {
      if (alert_ceo_cb_) {
        alert_ceo_cb_("description");
      }
    }
  }

  return result;
}

}  // namespace rover_autonomy
