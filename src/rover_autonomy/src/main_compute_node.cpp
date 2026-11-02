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
  auto camera_config  = param_manager_->get_camera_config();
  auto lidar_config   = param_manager_->get_lidar_config();

  auto odom_config    = param_manager_->get_odometry_config();
  auto mapping_config = param_manager_->get_mapping_config();
  auto nav_config     = param_manager_->get_navigation_config();

  // 3. Instantiate subsystems — ALWAYS created; enabled_ governs behavior, not construction
  
  camera_subsystem_ = std::make_unique<CameraSubsystemManager>(this, camera_config);
  all_subsystems_.push_back(camera_subsystem_.get());

  lidar_subsystem_ = std::make_unique<LidarSubsystemManager>(this, lidar_config);
  all_subsystems_.push_back(lidar_subsystem_.get());

  odometry_subsystem_ = std::make_unique<OdometrySubsystemManager>(this, odom_config);
  all_subsystems_.push_back(odometry_subsystem_.get());

  mapping_subsystem_ = std::make_unique<LocalizationAndMappingSubsystemManager>(this, mapping_config);
  all_subsystems_.push_back(mapping_subsystem_.get());

  navigation_subsystem_ = std::make_unique<NavigationSubsystemManager>(this, nav_config);
  all_subsystems_.push_back(navigation_subsystem_.get());

  RCLCPP_INFO(get_logger(), "MainComputeNode created. Subsystems registered: %zu", all_subsystems_.size());
}

void MainComputeNode::on_subsystem_config_changed(const std::string & subsystem_name)
{
  RCLCPP_WARN(get_logger(), "Config changed for subsystem '%s'. Triggering recovery...", subsystem_name.c_str());

  if (subsystem_name == "camera") { camera_subsystem_->recover(); }
  else if (subsystem_name == "lidar") { lidar_subsystem_->recover(); }
  else if (subsystem_name == "odometry") { odometry_subsystem_->recover(); }
  else if (subsystem_name == "mapping") { mapping_subsystem_->recover(); }
  else if (subsystem_name == "navigation") { navigation_subsystem_->recover(); }
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

  // Warn if Full Fusion mode is active but camera_01 odometry is not launched.
  // The EKF will subscribe to a dead /camera_01/odom topic and coast on other sensors.
  int loc_mode = 1;
  if (has_parameter("localization_mode")) {
    loc_mode = get_parameter("localization_mode").as_int();
  }
  bool cam01_odom_enabled = has_parameter("launch_camera_01_odom") &&
                            get_parameter("launch_camera_01_odom").as_bool();
  if (loc_mode == 1 && !cam01_odom_enabled) {
    RCLCPP_WARN(get_logger(),
      "localization_mode=1 (Full Fusion) is set but launch_camera_01_odom=false. "
      "The EKF will subscribe to /camera_01/odom but no publisher will exist. "
      "Set launch_camera_01_odom: true in main_compute.yaml or switch to a different mode.");
  }

  navigation_service_ = this->create_service<std_srvs::srv::SetBool>(
    "~/set_navigation",
    std::bind(&MainComputeNode::on_set_navigation, this, std::placeholders::_1, std::placeholders::_2)
  );

  mapping_service_ = this->create_service<std_srvs::srv::SetBool>(
    "~/set_mapping",
    std::bind(&MainComputeNode::on_set_mapping, this, std::placeholders::_1, std::placeholders::_2)
  );

  local_topography_service_ = this->create_service<std_srvs::srv::SetBool>(
    "~/set_local_topography",
    std::bind(&MainComputeNode::on_set_local_topography, this, std::placeholders::_1, std::placeholders::_2)
  );

  global_topography_service_ = this->create_service<std_srvs::srv::SetBool>(
    "~/set_global_topography",
    std::bind(&MainComputeNode::on_set_global_topography, this, std::placeholders::_1, std::placeholders::_2)
  );

  camera_00_service_ = this->create_service<std_srvs::srv::SetBool>(
    "~/set_camera_00",
    std::bind(&MainComputeNode::on_set_camera_00, this, std::placeholders::_1, std::placeholders::_2)
  );

  camera_01_service_ = this->create_service<std_srvs::srv::SetBool>(
    "~/set_camera_01",
    std::bind(&MainComputeNode::on_set_camera_01, this, std::placeholders::_1, std::placeholders::_2)
  );

  lidar_00_service_ = this->create_service<std_srvs::srv::SetBool>(
    "~/set_lidar_00",
    std::bind(&MainComputeNode::on_set_lidar_00, this, std::placeholders::_1, std::placeholders::_2)
  );

  odom_camera_00_service_ = this->create_service<std_srvs::srv::SetBool>(
    "~/set_odom_camera_00",
    std::bind(&MainComputeNode::on_set_odom_camera_00, this, std::placeholders::_1, std::placeholders::_2)
  );

  odom_camera_01_service_ = this->create_service<std_srvs::srv::SetBool>(
    "~/set_odom_camera_01",
    std::bind(&MainComputeNode::on_set_odom_camera_01, this, std::placeholders::_1, std::placeholders::_2)
  );

  odom_lidar_00_service_ = this->create_service<std_srvs::srv::SetBool>(
    "~/set_odom_lidar_00",
    std::bind(&MainComputeNode::on_set_odom_lidar_00, this, std::placeholders::_1, std::placeholders::_2)
  );

  localization_service_ = this->create_service<std_srvs::srv::SetBool>(
    "~/set_localization",
    std::bind(&MainComputeNode::on_set_localization, this, std::placeholders::_1, std::placeholders::_2)
  );

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

  for (auto it = all_subsystems_.rbegin(); it != all_subsystems_.rend(); ++it) {
    (*it)->on_deactivate();
  }

  RCLCPP_INFO(get_logger(), "Deactivation successful.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MainComputeNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up MainComputeNode...");

  for (auto it = all_subsystems_.rbegin(); it != all_subsystems_.rend(); ++it) {
    (*it)->on_cleanup();
  }

  RCLCPP_INFO(get_logger(), "Cleanup successful.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MainComputeNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down MainComputeNode...");

  for (auto it = all_subsystems_.rbegin(); it != all_subsystems_.rend(); ++it) {
    (*it)->on_deactivate();
    (*it)->on_cleanup();
  }

  RCLCPP_INFO(get_logger(), "Shutdown successful.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void MainComputeNode::on_set_navigation(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  RCLCPP_INFO(get_logger(), "Received request to set navigation: %s", request->data ? "true" : "false");

  if (request->data) {
    if (navigation_subsystem_->get_state() == SubsystemState::ACTIVE) {
      response->success = true;
      response->message = "Navigation is already active.";
    } else {
      navigation_subsystem_->set_enabled(true);
      if (navigation_subsystem_->on_activate()) {
        response->success = true;
        response->message = "Successfully started navigation.";
      } else {
        response->success = false;
        response->message = "Failed to start navigation.";
      }
    }
  } else {
    if (navigation_subsystem_->get_state() != SubsystemState::ACTIVE) {
      response->success = true;
      response->message = "Navigation is already inactive.";
      navigation_subsystem_->set_enabled(false);
    } else {
      if (navigation_subsystem_->on_deactivate()) {
        response->success = true;
        response->message = "Successfully stopped navigation.";
        navigation_subsystem_->set_enabled(false);
      } else {
        response->success = false;
        response->message = "Failed to stop navigation.";
      }
    }
  }
}

void MainComputeNode::on_set_mapping(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  auto sub = mapping_subsystem_->get_mapping_subsystem();
  RCLCPP_INFO(get_logger(), "Received request to set mapping: %s", request->data ? "true" : "false");

  if (request->data) {
    if (sub->get_state() == SubsystemState::ACTIVE) {
      response->success = true;
      response->message = "Mapping is already active.";
    } else {
      sub->set_enabled(true);
      if (sub->on_activate()) {
        response->success = true;
        response->message = "Successfully started mapping.";
      } else {
        response->success = false;
        response->message = "Failed to start mapping.";
      }
    }
  } else {
    if (sub->get_state() != SubsystemState::ACTIVE) {
      response->success = true;
      response->message = "Mapping is already inactive.";
      sub->set_enabled(false);
    } else {
      if (sub->on_deactivate()) {
        response->success = true;
        response->message = "Successfully stopped mapping.";
        sub->set_enabled(false);
      } else {
        response->success = false;
        response->message = "Failed to stop mapping.";
      }
    }
  }
}

void MainComputeNode::on_set_local_topography(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  auto sub = mapping_subsystem_->get_local_topography_subsystem();
  RCLCPP_INFO(get_logger(), "Received request to set local topography: %s", request->data ? "true" : "false");

  if (request->data) {
    if (sub->get_state() == SubsystemState::ACTIVE) {
      response->success = true;
      response->message = "Local topography is already active.";
    } else {
      sub->set_enabled(true);
      if (sub->on_activate()) {
        response->success = true;
        response->message = "Successfully started local topography.";
      } else {
        response->success = false;
        response->message = "Failed to start local topography.";
      }
    }
  } else {
    if (sub->get_state() != SubsystemState::ACTIVE) {
      response->success = true;
      response->message = "Local topography is already inactive.";
      sub->set_enabled(false);
    } else {
      if (sub->on_deactivate()) {
        response->success = true;
        response->message = "Successfully stopped local topography.";
        sub->set_enabled(false);
      } else {
        response->success = false;
        response->message = "Failed to stop local topography.";
      }
    }
  }
}

void MainComputeNode::on_set_global_topography(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  auto sub = mapping_subsystem_->get_global_topography_subsystem();
  RCLCPP_INFO(get_logger(), "Received request to set global topography: %s", request->data ? "true" : "false");

  if (request->data) {
    if (sub->get_state() == SubsystemState::ACTIVE) {
      response->success = true;
      response->message = "Global topography is already active.";
    } else {
      sub->set_enabled(true);
      if (sub->on_activate()) {
        response->success = true;
        response->message = "Successfully started global topography.";
      } else {
        response->success = false;
        response->message = "Failed to start global topography.";
      }
    }
  } else {
    if (sub->get_state() != SubsystemState::ACTIVE) {
      response->success = true;
      response->message = "Global topography is already inactive.";
      sub->set_enabled(false);
    } else {
      if (sub->on_deactivate()) {
        response->success = true;
        response->message = "Successfully stopped global topography.";
        sub->set_enabled(false);
      } else {
        response->success = false;
        response->message = "Failed to stop global topography.";
      }
    }
  }
}

void MainComputeNode::on_set_camera_00(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  auto sub = camera_subsystem_.get();
  RCLCPP_INFO(get_logger(), "Received request to set camera_00: %s", request->data ? "true" : "false");

  if (request->data) {
    if (sub->get_state() == SubsystemState::ACTIVE) {
      response->success = true;
      response->message = "Camera 00 subsystem is already active.";
    } else {
      sub->set_enabled(true);
      if (sub->on_activate()) {
        response->success = true;
        response->message = "Successfully started camera 00 subsystem.";
      } else {
        response->success = false;
        response->message = "Failed to start camera 00 subsystem.";
      }
    }
  } else {
    if (sub->get_state() != SubsystemState::ACTIVE) {
      response->success = true;
      response->message = "Camera 00 subsystem is already inactive.";
      sub->set_enabled(false);
    } else {
      if (sub->on_deactivate()) {
        response->success = true;
        response->message = "Successfully stopped camera 00 subsystem.";
        sub->set_enabled(false);
      } else {
        response->success = false;
        response->message = "Failed to stop camera 00 subsystem.";
      }
    }
  }
}

void MainComputeNode::on_set_camera_01(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  RCLCPP_INFO(get_logger(), "Received request to set camera_01 (placeholder): %s", request->data ? "true" : "false");
  response->success = true;
  response->message = "Camera 01 is a placeholder. No action taken.";
}

void MainComputeNode::on_set_lidar_00(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  auto sub = lidar_subsystem_.get();
  RCLCPP_INFO(get_logger(), "Received request to set lidar_00: %s", request->data ? "true" : "false");

  if (request->data) {
    if (sub->get_state() == SubsystemState::ACTIVE) {
      response->success = true;
      response->message = "Lidar 00 subsystem is already active.";
    } else {
      sub->set_enabled(true);
      if (sub->on_activate()) {
        response->success = true;
        response->message = "Successfully started lidar 00 subsystem.";
      } else {
        response->success = false;
        response->message = "Failed to start lidar 00 subsystem.";
      }
    }
  } else {
    if (sub->get_state() != SubsystemState::ACTIVE) {
      response->success = true;
      response->message = "Lidar 00 subsystem is already inactive.";
      sub->set_enabled(false);
    } else {
      if (sub->on_deactivate()) {
        response->success = true;
        response->message = "Successfully stopped lidar 00 subsystem.";
        sub->set_enabled(false);
      } else {
        response->success = false;
        response->message = "Failed to stop lidar 00 subsystem.";
      }
    }
  }
}

void MainComputeNode::on_set_odom_camera_00(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  auto sub = odometry_subsystem_->get_camera_00_odom_subsystem();
  RCLCPP_INFO(get_logger(), "Received request to set camera 00 odometry: %s", request->data ? "true" : "false");

  if (request->data) {
    if (sub->get_state() == SubsystemState::ACTIVE) {
      response->success = true;
      response->message = "Camera 00 odometry is already active.";
    } else {
      sub->set_enabled(true);
      if (sub->on_activate()) {
        response->success = true;
        response->message = "Successfully started camera 00 odometry.";
      } else {
        response->success = false;
        response->message = "Failed to start camera 00 odometry.";
      }
    }
  } else {
    if (sub->get_state() != SubsystemState::ACTIVE) {
      response->success = true;
      response->message = "Camera 00 odometry is already inactive.";
      sub->set_enabled(false);
    } else {
      if (sub->on_deactivate()) {
        response->success = true;
        response->message = "Successfully stopped camera 00 odometry.";
        sub->set_enabled(false);
      } else {
        response->success = false;
        response->message = "Failed to stop camera 00 odometry.";
      }
    }
  }
}

void MainComputeNode::on_set_odom_camera_01(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  auto sub = odometry_subsystem_->get_camera_01_odom_subsystem();
  RCLCPP_INFO(get_logger(), "Received request to set camera 01 odometry: %s", request->data ? "true" : "false");

  if (request->data) {
    if (sub->get_state() == SubsystemState::ACTIVE) {
      response->success = true;
      response->message = "Camera 01 odometry is already active.";
    } else {
      sub->set_enabled(true);
      if (sub->on_activate()) {
        response->success = true;
        response->message = "Successfully started camera 01 odometry.";
      } else {
        response->success = false;
        response->message = "Failed to start camera 01 odometry.";
      }
    }
  } else {
    if (sub->get_state() != SubsystemState::ACTIVE) {
      response->success = true;
      response->message = "Camera 01 odometry is already inactive.";
      sub->set_enabled(false);
    } else {
      if (sub->on_deactivate()) {
        response->success = true;
        response->message = "Successfully stopped camera 01 odometry.";
        sub->set_enabled(false);
      } else {
        response->success = false;
        response->message = "Failed to stop camera 01 odometry.";
      }
    }
  }
}

void MainComputeNode::on_set_odom_lidar_00(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  auto sub = odometry_subsystem_->get_lidar_00_odom_subsystem();
  RCLCPP_INFO(get_logger(), "Received request to set lidar 00 odometry: %s", request->data ? "true" : "false");

  if (request->data) {
    if (sub->get_state() == SubsystemState::ACTIVE) {
      response->success = true;
      response->message = "Lidar 00 odometry is already active.";
    } else {
      sub->set_enabled(true);
      if (sub->on_activate()) {
        response->success = true;
        response->message = "Successfully started lidar 00 odometry.";
      } else {
        response->success = false;
        response->message = "Failed to start lidar 00 odometry.";
      }
    }
  } else {
    if (sub->get_state() != SubsystemState::ACTIVE) {
      response->success = true;
      response->message = "Lidar 00 odometry is already inactive.";
      sub->set_enabled(false);
    } else {
      if (sub->on_deactivate()) {
        response->success = true;
        response->message = "Successfully stopped lidar 00 odometry.";
        sub->set_enabled(false);
      } else {
        response->success = false;
        response->message = "Failed to stop lidar 00 odometry.";
      }
    }
  }
}

void MainComputeNode::on_set_localization(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  auto sub = odometry_subsystem_->get_localization_subsystem();
  RCLCPP_INFO(get_logger(), "Received request to set localization: %s", request->data ? "true" : "false");

  if (request->data) {
    if (sub->get_state() == SubsystemState::ACTIVE) {
      response->success = true;
      response->message = "Localization is already active.";
    } else {
      sub->set_enabled(true);
      if (sub->on_activate()) {
        response->success = true;
        response->message = "Successfully started localization.";
      } else {
        response->success = false;
        response->message = "Failed to start localization.";
      }
    }
  } else {
    if (sub->get_state() != SubsystemState::ACTIVE) {
      response->success = true;
      response->message = "Localization is already inactive.";
      sub->set_enabled(false);
    } else {
      if (sub->on_deactivate()) {
        response->success = true;
        response->message = "Successfully stopped localization.";
        sub->set_enabled(false);
      } else {
        response->success = false;
        response->message = "Failed to stop localization.";
      }
    }
  }
}


RCLCPP_COMPONENTS_REGISTER_NODE(rover_autonomy::MainComputeNode)

}  // namespace rover_autonomy