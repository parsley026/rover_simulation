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
  node_->declare_parameter<bool>("launch_camera_00_driver",          true);
  node_->declare_parameter<bool>("launch_camera_00_decompression",   true);
  node_->declare_parameter<bool>("launch_camera_00_post_processing", true);
  node_->declare_parameter<bool>("launch_lidar_00",       true);
  node_->declare_parameter<bool>("launch_camera_00_odom", true);
  node_->declare_parameter<bool>("launch_lidar_00_odom",  true);
  node_->declare_parameter<bool>("launch_localization",   true);
  node_->declare_parameter<bool>("launch_mapping",        true);
  node_->declare_parameter<bool>("launch_local_topography",  true);
  node_->declare_parameter<bool>("launch_global_topography", true);
  node_->declare_parameter<bool>("launch_navigation",     true);

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
  node_->declare_parameter<std::string>("navigation_config", "config/navigation/nav2_params.yaml");

  // --- Camera subsystem identifiers ---
  node_->declare_parameter<std::string>("camera_package", "rover_autonomy");
  node_->declare_parameter<std::string>("camera_driver_launch",  "camera_driver.launch.py");
  node_->declare_parameter<std::string>("camera_decompression_launch",  "camera_decompression.launch.py");
  node_->declare_parameter<std::string>("camera_post_processing_launch",  "camera_post_processing.launch.py");
  node_->declare_parameter<bool>("camera_decompress_rgb", true);
  node_->declare_parameter<bool>("camera_decompress_depth", true);
  node_->declare_parameter<std::string>("camera_ns",      "camera_00");
  node_->declare_parameter<std::string>("camera_name",    "camera_00");
  node_->declare_parameter<std::string>("camera_parent_frame", "base_link");

  // --- Lidar subsystem identifiers ---
  node_->declare_parameter<std::string>("lidar_package", "rover_autonomy");
  node_->declare_parameter<std::string>("lidar_driver_launch", "lidar_driver.launch.py");
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
  node_->declare_parameter<std::string>("mapping_ns",           "mapping");
  node_->declare_parameter<std::string>("mapping_db_folder",    "~/.ros/rtabmap");
  node_->declare_parameter<bool>("mapping_load_existing_db",    false);
  node_->declare_parameter<std::string>("mapping_db_file_name", "rtabmap.db");

  node_->declare_parameter<std::string>("local_topography_package", "rover_topography");
  node_->declare_parameter<std::string>("local_topography_launch",  "local_topography.launch.py");
  node_->declare_parameter<std::string>("local_topography_ns",      "");

  node_->declare_parameter<std::string>("global_topography_package", "rover_topography");
  node_->declare_parameter<std::string>("global_topography_launch",  "global_topography.launch.py");
  node_->declare_parameter<std::string>("global_topography_ns",      "");

  node_->declare_parameter<std::string>("navigation_package",      "rover_autonomy");
  node_->declare_parameter<std::string>("navigation_launch",       "navigation.launch.py");
  node_->declare_parameter<std::string>("navigation_ns",           "navigation");
  node_->declare_parameter<bool>("navigation_autostart",           true);
  node_->declare_parameter<bool>("navigation_use_composition",     true);


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

CameraConfig ParameterManager::get_camera_config() const
{
  CameraConfig config;
  bool use_sim_time = node_->get_parameter("use_sim_time").as_bool();
  bool base_enabled = node_->get_parameter("launch_camera_00").as_bool();
  bool driver_enabled = node_->get_parameter("launch_camera_00_driver").as_bool();
  bool decomp_enabled = node_->get_parameter("launch_camera_00_decompression").as_bool();
  bool post_enabled = node_->get_parameter("launch_camera_00_post_processing").as_bool();
  
  std::string sim_time_str = use_sim_time ? "true" : "false";

  // Common args
  std::map<std::string, std::string> common_args;
  common_args["use_sim_time"] = sim_time_str;
  common_args["camera_ns"]    = node_->get_parameter("camera_ns").as_string();

  // 1. Driver
  config.driver.launch_enabled = use_sim_time ? false : (base_enabled && driver_enabled);
  config.driver.package_name   = node_->get_parameter("camera_package").as_string();
  config.driver.launch_script  = node_->get_parameter("camera_driver_launch").as_string();
  config.driver.launch_args = common_args;
  config.driver.launch_args["camera_name"]         = node_->get_parameter("camera_name").as_string();
  config.driver.launch_args["camera_parent_frame"] = node_->get_parameter("camera_parent_frame").as_string();
  config.driver.launch_args["params_file"] = resolve_share_path("rover_autonomy", node_->get_parameter("camera_00_config").as_string());

  // 2. Decompression
  config.decompression.launch_enabled = use_sim_time ? false : (base_enabled && decomp_enabled);
  config.decompression.package_name   = node_->get_parameter("camera_package").as_string();
  config.decompression.launch_script  = node_->get_parameter("camera_decompression_launch").as_string();
  config.decompression.launch_args = common_args;
  config.decompression.launch_args["decompress_rgb"]   = node_->get_parameter("camera_decompress_rgb").as_bool() ? "true" : "false";
  config.decompression.launch_args["decompress_depth"] = node_->get_parameter("camera_decompress_depth").as_bool() ? "true" : "false";

  // 3. Post Processing (sync)
  config.post_processing.launch_enabled = base_enabled && post_enabled; // Always run if camera is "enabled"
  config.post_processing.package_name   = node_->get_parameter("camera_package").as_string();
  config.post_processing.launch_script  = node_->get_parameter("camera_post_processing_launch").as_string();
  config.post_processing.launch_args = common_args;

  return config;
}

SubsystemConfig ParameterManager::get_lidar_config() const
{
  SubsystemConfig config;
  config.launch_enabled = node_->get_parameter("launch_lidar_00").as_bool();
  config.package_name   = node_->get_parameter("lidar_package").as_string();
  config.launch_script  = node_->get_parameter("lidar_driver_launch").as_string();

  bool use_sim_time = node_->get_parameter("use_sim_time").as_bool();
  config.launch_args["use_sim_time"] = use_sim_time ? "true" : "false";

  config.launch_args["lidar_ns"]           = node_->get_parameter("lidar_ns").as_string();
  config.launch_args["lidar_name"]         = node_->get_parameter("lidar_name").as_string();
  config.launch_args["lidar_parent_frame"] = node_->get_parameter("lidar_parent_frame").as_string();

  config.launch_args["params_file"] =
    resolve_share_path("rover_autonomy", node_->get_parameter("lidar_00_config").as_string());

  return config;
}

MappingConfig ParameterManager::get_mapping_config() const
{
  MappingConfig composite;
  bool use_sim_time = node_->get_parameter("use_sim_time").as_bool();

  // 1. Mapping
  composite.mapping.launch_enabled = node_->get_parameter("launch_mapping").as_bool();
  composite.mapping.package_name   = node_->get_parameter("mapping_package").as_string();
  composite.mapping.launch_script  = node_->get_parameter("mapping_launch").as_string();
  composite.mapping.launch_args["use_sim_time"] = use_sim_time ? "true" : "false";
  composite.mapping.launch_args["namespace"] = node_->get_parameter("mapping_ns").as_string();
  composite.mapping.launch_args["mapping_db_folder"] = node_->get_parameter("mapping_db_folder").as_string();
  composite.mapping.launch_args["mapping_load_existing_db"] = node_->get_parameter("mapping_load_existing_db").as_bool() ? "true" : "false";
  composite.mapping.launch_args["mapping_db_file_name"] = node_->get_parameter("mapping_db_file_name").as_string();
  composite.mapping.launch_args["params_file"] =
    resolve_share_path("rover_autonomy", node_->get_parameter("mapping_config").as_string());

  // 2. Local Topography
  composite.local_topography.launch_enabled = node_->get_parameter("launch_local_topography").as_bool();
  composite.local_topography.package_name   = node_->get_parameter("local_topography_package").as_string();
  composite.local_topography.launch_script  = node_->get_parameter("local_topography_launch").as_string();
  composite.local_topography.launch_args["use_sim_time"] = use_sim_time ? "true" : "false";
  composite.local_topography.launch_args["namespace"]    = node_->get_parameter("local_topography_ns").as_string();

  // 3. Global Topography
  composite.global_topography.launch_enabled = node_->get_parameter("launch_global_topography").as_bool();
  composite.global_topography.package_name   = node_->get_parameter("global_topography_package").as_string();
  composite.global_topography.launch_script  = node_->get_parameter("global_topography_launch").as_string();
  composite.global_topography.launch_args["use_sim_time"] = use_sim_time ? "true" : "false";
  composite.global_topography.launch_args["namespace"]    = node_->get_parameter("global_topography_ns").as_string();

  return composite;
}

SubsystemConfig ParameterManager::get_navigation_config() const
{
  SubsystemConfig config;
  config.launch_enabled = node_->get_parameter("launch_navigation").as_bool();
  config.package_name   = node_->get_parameter("navigation_package").as_string();
  config.launch_script  = node_->get_parameter("navigation_launch").as_string();

  bool use_sim_time = node_->get_parameter("use_sim_time").as_bool();
  config.launch_args["use_sim_time"] = use_sim_time ? "true" : "false";
  config.launch_args["namespace"] = node_->get_parameter("navigation_ns").as_string();

  bool autostart = node_->get_parameter("navigation_autostart").as_bool();
  config.launch_args["autostart"] = autostart ? "true" : "false";

  bool use_composition = node_->get_parameter("navigation_use_composition").as_bool();
  config.launch_args["use_composition"] = use_composition ? "True" : "False";

  config.launch_args["params_file"] =
    resolve_share_path("rover_autonomy", node_->get_parameter("navigation_config").as_string());

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
  std::set<std::string> changed_subsystems;

  for (const auto & param : parameters) {
    RCLCPP_INFO(node_->get_logger(), "Parameter updated: %s", param.get_name().c_str());
    std::string name = param.get_name();

    // Alert the CEO when description-affecting params change at runtime.
    if (name == "use_sim_time" ||
        name == "launch_description" ||
        name == "description_package" ||
        name == "description_launch")
    {
      changed_subsystems.insert("description");
    }
    else if (name.find("localization") != std::string::npos) {
      changed_subsystems.insert("localization");
    }
    else if (name.find("mapping") != std::string::npos) {
      changed_subsystems.insert("mapping");
    }
    else if (name.find("navigation") != std::string::npos) {
      changed_subsystems.insert("navigation");
    }
  }

  if (alert_ceo_cb_) {
    for (const auto & subsystem : changed_subsystems) {
      alert_ceo_cb_(subsystem);
    }
  }

  return result;
}

}  // namespace rover_autonomy
