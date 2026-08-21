#pragma once
#include <grid_map_core/grid_map_core.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace rover_topography::core {

class MapBoundsStrategy {
public:
  virtual ~MapBoundsStrategy() = default;
  virtual void updateBounds(grid_map::GridMap & map, const grid_map::Position & robot_pos, const sensor_msgs::msg::PointCloud2 & cloud) = 0;
};

class CircularBufferStrategy : public MapBoundsStrategy {
public:
  void updateBounds(grid_map::GridMap & map, const grid_map::Position & robot_pos, const sensor_msgs::msg::PointCloud2 & cloud) override;
};

class DynamicBoundingStrategy : public MapBoundsStrategy {
public:
  void updateBounds(grid_map::GridMap & map, const grid_map::Position & robot_pos, const sensor_msgs::msg::PointCloud2 & cloud) override;
};

} // namespace rover_topography::core
