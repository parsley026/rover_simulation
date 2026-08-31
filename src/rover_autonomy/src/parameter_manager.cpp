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
  node_->declare_parameter<std::string>("ukf_config",            "config/localization/ukf_filter_sim.yaml");
  node_->declare_parameter<std::string>("navigation_config", "config/navigation/nav2_params.yaml");

  // --- Camera subsystem identifiers ---
  node_->declare_parameter<std::string>("camera_00_package", "rover_autonomy");
  node_->declare_parameter<std::string>("camera_00_driver_launch",  "camera_driver.launch.py");
  node_->declare_parameter<std::string>("camera_00_decompression_launch",  "camera_decompression.launch.py");
  node_->declare_parameter<std::string>("camera_00_post_processing_launch",  "camera_post_processing.launch.py");
  node_->declare_parameter<bool>("camera_00_decompress_rgb", true);
  node_->declare_parameter<bool>("camera_00_decompress_depth", true);
  node_->declare_parameter<std::string>("camera_00_ns", "camera_00");
  node_->declare_parameter<std::string>("camera_01_ns", "camera_01");
  node_->declare_parameter<std::string>("camera_00_name",    "camera_00");
  node_->declare_parameter<std::string>("camera_00_parent_frame", "base_link");


  // --- Lidar subsystem identifiers ---
  node_->declare_parameter<std::string>("lidar_package", "rover_autonomy");
  node_->declare_parameter<std::string>("lidar_driver_launch", "lidar_driver.launch.py");
  node_->declare_parameter<std::string>("lidar_ns",      "lidar_00");
  node_->declare_parameter<std::string>("lidar_name",    "lidar_00");
  node_->declare_parameter<std::string>("lidar_parent_frame", "base_link");

  // --- Odometry subsystem identifiers (Option B: fully YAML-driven) ---
  node_->declare_parameter<std::string>("camera_00_odom_package", "rover_autonomy");
  node_->declare_parameter<std::string>("camera_00_odom_launch",  "rgbd_odometry.launch.py");
  node_->declare_parameter<std::string>("camera_00_odom_ns",      "camera_00");

  // Camera 01 odometry (previously unmanaged — added as a full subsystem)
  node_->declare_parameter<bool>("launch_camera_01_odom",             false);
  node_->declare_parameter<std::string>("camera_01_odom_launch", "rgbd_odometry.launch.py");
  node_->declare_parameter<std::string>("camera_01_odom_ns",     "camera_01");
  node_->declare_parameter<std::string>("camera_01_odom_config",        "config/odometry/rgbd_odom.yaml");

  node_->declare_parameter<std::string>("lidar_odom_package",  "rover_autonomy");
  node_->declare_parameter<std::string>("lidar_odom_launch",   "icp_odometry.launch.py");
  node_->declare_parameter<std::string>("lidar_odom_ns",       "lidar_00");

  // Odometry health watchdog toggles (all disabled by default)
  node_->declare_parameter<bool>("launch_camera_00_odom_health", false);
  node_->declare_parameter<bool>("launch_camera_01_odom_health", false);
  node_->declare_parameter<bool>("launch_lidar_00_odom_health",  false);

  // Health watchdog config paths
  node_->declare_parameter<std::string>("camera_00_odom_health_config", "config/odometry/health_rgbd.yaml");
  node_->declare_parameter<std::string>("camera_01_odom_health_config", "config/odometry/health_rgbd.yaml");
  node_->declare_parameter<std::string>("lidar_00_odom_health_config",  "config/odometry/health_icp.yaml");

  node_->declare_parameter<std::string>("localization_package", "rover_autonomy");
  node_->declare_parameter<std::string>("localization_launch",  "localization.launch.py");
  node_->declare_parameter<bool>("localization_use_ekf",        true);
  node_->declare_parameter<bool>("localization_use_ukf",        false);
  node_->declare_parameter<std::string>("localization_ns",      "localization");
  node_->declare_parameter<int>("localization_mode",            1);


  node_->declare_parameter<std::string>("mapping_package",      "rover_autonomy");
  node_->declare_parameter<std::string>("mapping_launch",       "mapping.launch.py");
  node_->declare_parameter<std::string>("mapping_ns",           "mapping");
  node_->declare_parameter<int>("mapping_mode",                 2);

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

  // Camera 00 Odometry
  bool cam0_health_on = node_->get_parameter("launch_camera_00_odom_health").as_bool();
  SubsystemConfig camera_odom;
  camera_odom.launch_enabled = node_->get_parameter("launch_camera_00_odom").as_bool();
  camera_odom.package_name   = node_->get_parameter("camera_00_odom_package").as_string();
  camera_odom.launch_script  = node_->get_parameter("camera_00_odom_launch").as_string();
  camera_odom.launch_args["use_sim_time"] = sim_time_str;
  camera_odom.launch_args["camera_ns"]    = node_->get_parameter("camera_00_odom_ns").as_string();
  camera_odom.launch_args["params_file"]  =
    resolve_share_path("rover_autonomy", node_->get_parameter("camera_00_odom_config").as_string());
  camera_odom.launch_args["health_enabled"] = cam0_health_on ? "true" : "false";

  // Camera 00 Health Watchdog
  SubsystemConfig camera_odom_health;
  camera_odom_health.launch_enabled = cam0_health_on;
  camera_odom_health.package_name   = "rover_autonomy";
  camera_odom_health.launch_script  = "odometry_health.launch.py";
  camera_odom_health.launch_args["use_sim_time"] = sim_time_str;
  camera_odom_health.launch_args["sensor_ns"]    = node_->get_parameter("camera_00_odom_ns").as_string();
  camera_odom_health.launch_args["params_file"]  =
    resolve_share_path("rover_autonomy", node_->get_parameter("camera_00_odom_health_config").as_string());

  // Camera 01 Odometry (secondary stereo — was unmanaged)
  bool cam1_health_on = node_->get_parameter("launch_camera_01_odom_health").as_bool();
  SubsystemConfig camera_01_odom;
  camera_01_odom.launch_enabled = node_->get_parameter("launch_camera_01_odom").as_bool();
  camera_01_odom.package_name   = "rover_autonomy";
  camera_01_odom.launch_script  = node_->get_parameter("camera_01_odom_launch").as_string();
  camera_01_odom.launch_args["use_sim_time"] = sim_time_str;
  camera_01_odom.launch_args["camera_ns"]    = node_->get_parameter("camera_01_odom_ns").as_string();
  camera_01_odom.launch_args["params_file"]  =
    resolve_share_path("rover_autonomy", node_->get_parameter("camera_01_odom_config").as_string());
  camera_01_odom.launch_args["health_enabled"] = cam1_health_on ? "true" : "false";

  // Camera 01 Health Watchdog
  SubsystemConfig camera_01_odom_health;
  camera_01_odom_health.launch_enabled = cam1_health_on;
  camera_01_odom_health.package_name   = "rover_autonomy";
  camera_01_odom_health.launch_script  = "odometry_health.launch.py";
  camera_01_odom_health.launch_args["use_sim_time"] = sim_time_str;
  camera_01_odom_health.launch_args["sensor_ns"]    = node_->get_parameter("camera_01_odom_ns").as_string();
  camera_01_odom_health.launch_args["params_file"]  =
    resolve_share_path("rover_autonomy", node_->get_parameter("camera_01_odom_health_config").as_string());

  // Lidar Odometry
  bool lidar_health_on = node_->get_parameter("launch_lidar_00_odom_health").as_bool();
  SubsystemConfig lidar_odom;
  lidar_odom.launch_enabled = node_->get_parameter("launch_lidar_00_odom").as_bool();
  lidar_odom.package_name   = node_->get_parameter("lidar_odom_package").as_string();
  lidar_odom.launch_script  = node_->get_parameter("lidar_odom_launch").as_string();
  lidar_odom.launch_args["use_sim_time"] = sim_time_str;
  lidar_odom.launch_args["lidar_ns"]     = node_->get_parameter("lidar_odom_ns").as_string();
  lidar_odom.launch_args["params_file"]  =
    resolve_share_path("rover_autonomy", node_->get_parameter("lidar_00_odom_config").as_string());
  lidar_odom.launch_args["health_enabled"] = lidar_health_on ? "true" : "false";

  // Lidar Health Watchdog
  SubsystemConfig lidar_odom_health;
  lidar_odom_health.launch_enabled = lidar_health_on;
  lidar_odom_health.package_name   = "rover_autonomy";
  lidar_odom_health.launch_script  = "odometry_health.launch.py";
  lidar_odom_health.launch_args["use_sim_time"] = sim_time_str;
  lidar_odom_health.launch_args["sensor_ns"]    = node_->get_parameter("lidar_odom_ns").as_string();
  lidar_odom_health.launch_args["params_file"]  =
    resolve_share_path("rover_autonomy", node_->get_parameter("lidar_00_odom_health_config").as_string());

  // Localization (EKF/UKF — type is YAML-driven, Python launch decides)
  SubsystemConfig localization;
  bool use_ekf = node_->get_parameter("localization_use_ekf").as_bool();
  bool use_ukf = node_->get_parameter("localization_use_ukf").as_bool();
  
  localization.launch_enabled = node_->get_parameter("launch_localization").as_bool();
  localization.package_name   = node_->get_parameter("localization_package").as_string();
  localization.launch_script  = node_->get_parameter("localization_launch").as_string();
  localization.launch_args["use_sim_time"]      = sim_time_str;
  localization.launch_args["use_ekf"] = use_ekf ? "true" : "false";
  localization.launch_args["use_ukf"] = use_ukf ? "true" : "false";
  
  localization.launch_args["ekf_params_file"] =
    resolve_share_path("rover_autonomy", node_->get_parameter("ekf_config").as_string());
  localization.launch_args["ukf_params_file"] =
    resolve_share_path("rover_autonomy", node_->get_parameter("ukf_config").as_string());
  // Forward sensor namespaces so localization.launch.py can remap EKF inputs dynamically
  localization.launch_args["localization_ns"] = node_->get_parameter("localization_ns").as_string();
  localization.launch_args["localization_mode"] = std::to_string(node_->get_parameter("localization_mode").as_int());
  localization.launch_args["camera_00_ns"] = node_->get_parameter("camera_00_ns").as_string();
  localization.launch_args["camera_01_ns"] = node_->get_parameter("camera_01_ns").as_string();
  localization.launch_args["lidar_ns"]        = node_->get_parameter("lidar_ns").as_string();

  return OdometryConfig{
    camera_odom, camera_odom_health,
    camera_01_odom, camera_01_odom_health,
    lidar_odom, lidar_odom_health,
    localization};
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
  common_args["camera_ns"]    = node_->get_parameter("camera_00_ns").as_string();


  // 1. Driver
  config.driver.launch_enabled = use_sim_time ? false : (base_enabled && driver_enabled);
  config.driver.package_name   = node_->get_parameter("camera_00_package").as_string();
  config.driver.launch_script  = node_->get_parameter("camera_00_driver_launch").as_string();
  config.driver.launch_args = common_args;
  config.driver.launch_args["camera_name"]         = node_->get_parameter("camera_00_name").as_string();
  config.driver.launch_args["camera_parent_frame"] = node_->get_parameter("camera_00_parent_frame").as_string();
  config.driver.launch_args["params_file"] = resolve_share_path("rover_autonomy", node_->get_parameter("camera_00_config").as_string());

  // 2. Decompression
  config.decompression.launch_enabled = use_sim_time ? false : (base_enabled && decomp_enabled);
  config.decompression.package_name   = node_->get_parameter("camera_00_package").as_string();
  config.decompression.launch_script  = node_->get_parameter("camera_00_decompression_launch").as_string();
  config.decompression.launch_args = common_args;
  config.decompression.launch_args["decompress_rgb"]   = node_->get_parameter("camera_00_decompress_rgb").as_bool() ? "true" : "false";
  config.decompression.launch_args["decompress_depth"] = node_->get_parameter("camera_00_decompress_depth").as_bool() ? "true" : "false";

  // 3. Post Processing (sync)
  config.post_processing.launch_enabled = base_enabled && post_enabled; // Always run if camera is "enabled"
  config.post_processing.package_name   = node_->get_parameter("camera_00_package").as_string();
  config.post_processing.launch_script  = node_->get_parameter("camera_00_post_processing_launch").as_string();
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
  // Forward sensor namespaces so mapping.launch.py can build remappings dynamically
  composite.mapping.launch_args["camera_00_ns"] = node_->get_parameter("camera_00_ns").as_string();
  composite.mapping.launch_args["camera_01_ns"] = node_->get_parameter("camera_01_ns").as_string();
  composite.mapping.launch_args["lidar_ns"]            = node_->get_parameter("lidar_ns").as_string();
  composite.mapping.launch_args["localization_ns"]     = node_->get_parameter("localization_ns").as_string();
  composite.mapping.launch_args["mapping_mode"]        = std::to_string(node_->get_parameter("mapping_mode").as_int());



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
