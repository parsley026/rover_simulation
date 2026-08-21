#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <message_filters/subscriber.h>
#include <tf2_ros/message_filter.h>
#include <filters/filter_chain.hpp>

#include "rover_topography/core/map_bounds_strategy.hpp"

namespace rover_topography::ros {

class TopographyNode : public rclcpp::Node {
public:
  explicit TopographyNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~TopographyNode() override = default;

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
  void publishMaps(const rclcpp::Time & stamp);

  // Core Mapping Components
  grid_map::GridMap map_;
  std::unique_ptr<core::MapBoundsStrategy> bounds_strategy_;
  
  // Filter Chain Pipeline
  filters::FilterChain<grid_map::GridMap> filter_chain_;

  // ROS parameters
  std::string map_frame_;
  std::string robot_frame_;
  double resolution_;

  // TF Synchronization using message_filters
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> pointcloud_sub_;
  std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>> tf_filter_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_pub_;
};

} // namespace rover_topography::ros
