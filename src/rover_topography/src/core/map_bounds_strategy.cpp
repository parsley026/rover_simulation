#include "rover_topography/core/map_bounds_strategy.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <algorithm>
#include <limits>
#include <cmath>

namespace rover_topography::core {

void CircularBufferStrategy::updateBounds(grid_map::GridMap & map, const grid_map::Position & robot_pos, const sensor_msgs::msg::PointCloud2 &) {
  // Constant-time O(1) circular buffer repositioning
  map.move(robot_pos);
}

void DynamicBoundingStrategy::updateBounds(grid_map::GridMap & map, const grid_map::Position &, const sensor_msgs::msg::PointCloud2 & cloud) {
  if (cloud.width == 0 || cloud.height == 0) return;

  float min_x = std::numeric_limits<float>::max();
  float max_x = -std::numeric_limits<float>::max();
  float min_y = std::numeric_limits<float>::max();
  float max_y = -std::numeric_limits<float>::max();
  size_t valid_points = 0;

  try {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

    // Iterate to find the bounding box of the entire global cloud
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    const float x = *iter_x;
    const float y = *iter_y;
    const float z = *iter_z;
    if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
      min_x = std::min(min_x, x);
      max_x = std::max(max_x, x);
      min_y = std::min(min_y, y);
      max_y = std::max(max_y, y);
      ++valid_points;
    }
  }

  if (valid_points == 0 || min_x >= max_x || min_y >= max_y) {
    return;
  }

  const double resolution = map.getResolution();
  const double length_x = static_cast<double>(max_x - min_x) + (2.0 * resolution);
  const double length_y = static_cast<double>(max_y - min_y) + (2.0 * resolution);
  const grid_map::Position center((min_x + max_x) / 2.0, (min_y + max_y) / 2.0);

  // Resize and wipe grid clean to wrap the incoming bounds
  map.setGeometry(grid_map::Length(length_x, length_y), resolution, center);
  } catch (const std::runtime_error&) {
    return;
  }
}

} // namespace rover_topography::core
