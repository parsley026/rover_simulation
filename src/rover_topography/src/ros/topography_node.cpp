#include "rover_topography/ros/topography_node.hpp"
#include "rover_topography/core/elevation_fuser.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_ros/create_timer_ros.h>

namespace rover_topography::ros {

TopographyNode::TopographyNode(const rclcpp::NodeOptions & options)
: Node("topography_node", rclcpp::NodeOptions(options)
    .allow_undeclared_parameters(true)),
  map_({"elevation", "variance"}),
  filter_chain_("grid_map::GridMap")
{
  auto declare_or_get_string = [this](const std::string & name, const std::string & default_value) {
    if (this->has_parameter(name)) return this->get_parameter(name).as_string();
    return this->declare_parameter<std::string>(name, default_value);
  };
  auto declare_or_get_double = [this](const std::string & name, double default_value) {
    if (this->has_parameter(name)) return this->get_parameter(name).as_double();
    return this->declare_parameter<double>(name, default_value);
  };

  resolution_ = declare_or_get_double("resolution", 0.1);
  map_frame_ = declare_or_get_string("map_frame", "map");
  robot_frame_ = declare_or_get_string("robot_frame", "base_link");
  std::string mode = declare_or_get_string("mapping_mode", "local");
  std::string filter_chain_param = declare_or_get_string("filter_chain_parameter_name", "filters");

  std::string costmap_topic = declare_or_get_string("costmap_topic", "/topography_costmap");
  std::string gridmap_topic = declare_or_get_string("gridmap_topic", "/topography/grid_map");
  std::string cloud_topic = declare_or_get_string("pointcloud_topic", "/points");

  map_.setBasicLayers({"elevation"});

  if (mode == "local") {
    bounds_strategy_ = std::make_unique<core::CircularBufferStrategy>();
    map_.setGeometry(grid_map::Length(10.0, 10.0), resolution_);
    map_.setFrameId(map_frame_);
  } else {
    bounds_strategy_ = std::make_unique<core::DynamicBoundingStrategy>();
    map_.setGeometry(grid_map::Length(1.0, 1.0), resolution_); // Initialize resolution
    map_.setFrameId(map_frame_);
  }

  // Initialize Filter Chain dynamically via YAML parameter
  if (!filter_chain_.configure(filter_chain_param, this->get_node_logging_interface(), this->get_node_parameters_interface())) {
    RCLCPP_ERROR(this->get_logger(), "Could not configure the filter chain!");
  }

  // TF Setup
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Message Filters for TF Sync (Eliminates dropped frames)
  pointcloud_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, cloud_topic);
  tf_filter_ = std::make_unique<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
    *pointcloud_sub_, *tf_buffer_, map_frame_, 10, this->get_node_logging_interface(), this->get_node_clock_interface());
  tf_filter_->registerCallback(std::bind(&TopographyNode::pointCloudCallback, this, std::placeholders::_1));

  occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(costmap_topic, rclcpp::QoS(1).transient_local());
  grid_map_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>(gridmap_topic, rclcpp::QoS(10));
  
  RCLCPP_INFO(this->get_logger(), "TopographyNode initialized in %s mode.", mode.c_str());
}

void TopographyNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  sensor_msgs::msg::PointCloud2 transformed_cloud;
  if (msg->header.frame_id == map_frame_) {
    transformed_cloud = *msg;
  } else {
    try {
      geometry_msgs::msg::TransformStamped sensor_to_map = tf_buffer_->lookupTransform(
        map_frame_, msg->header.frame_id, msg->header.stamp);
      tf2::doTransform(*msg, transformed_cloud, sensor_to_map);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "TF error: %s", ex.what());
      return;
    }
  }

  // Get Robot Position for local mapping
  grid_map::Position robot_pos(0.0, 0.0);
  if (dynamic_cast<core::CircularBufferStrategy*>(bounds_strategy_.get())) {
    try {
      auto robot_to_map = tf_buffer_->lookupTransform(map_frame_, robot_frame_, msg->header.stamp);
      robot_pos.x() = robot_to_map.transform.translation.x;
      robot_pos.y() = robot_to_map.transform.translation.y;
    } catch (const tf2::TransformException &) {}
  }

  // 1. Core Engine: Update Boundaries
  bounds_strategy_->updateBounds(map_, robot_pos, transformed_cloud);

  // 2. Core Engine: Fuse Elevation & Variance explicitly
  bool is_global = dynamic_cast<core::DynamicBoundingStrategy*>(bounds_strategy_.get()) != nullptr;
  core::ElevationFuser::fuse(map_, transformed_cloud, is_global);

  // 3. Filter Chain: Compute slope, roughness, cost dynamically via plugins!
  map_.convertToDefaultStartIndex(); // Required for SlidingWindowMathExpressionFilter on rolling window maps
  if (!filter_chain_.update(map_, map_)) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Filter chain failed to update!");
  }

  publishMaps(msg->header.stamp);
}

void TopographyNode::publishMaps(const rclcpp::Time & stamp)
{
  if (map_.getSize()(0) == 0 || map_.getSize()(1) == 0) return;
  map_.setTimestamp(stamp.nanoseconds());

  if (grid_map_pub_->get_subscription_count() > 0) {
    auto grid_map_msg = grid_map::GridMapRosConverter::toMessage(map_);
    grid_map_pub_->publish(std::move(grid_map_msg));
  }

  if (occupancy_grid_pub_->get_subscription_count() > 0 && map_.exists("cost")) {
    nav_msgs::msg::OccupancyGrid occupancy_grid_msg;
    grid_map::GridMapRosConverter::toOccupancyGrid(map_, "cost", 0.0f, 1.0f, occupancy_grid_msg);
    occupancy_grid_pub_->publish(occupancy_grid_msg);
  }
}

} // namespace rover_topography::ros

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rover_topography::ros::TopographyNode)
