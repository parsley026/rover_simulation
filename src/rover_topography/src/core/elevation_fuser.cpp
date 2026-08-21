#include "rover_topography/core/elevation_fuser.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>

namespace rover_topography::core {

void ElevationFuser::fuse(grid_map::GridMap & map, const sensor_msgs::msg::PointCloud2 & cloud, bool is_global) {
  if (cloud.width == 0 || cloud.height == 0) return;

  try {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    const float x = *iter_x;
    const float y = *iter_y;
    const float z = *iter_z;

    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }

    grid_map::Position pos(x, y);
    grid_map::Index index;
    if (!map.getIndex(pos, index)) {
      continue;
    }

    float & elev = map.at("elevation", index);
    float & var = map.at("variance", index);

    if (!std::isfinite(elev)) {
      elev = z;
      var = 0.01f; // Initial variance estimate
    } else {
      if (is_global) {
        // Simple global moving average
        elev = 0.5f * (elev + z);
        var = 0.5f * var;
      } else {
        // Exponential smoothing imitating a simplified Kalman filter
        const float alpha = 0.15f;
        elev = (1.0f - alpha) * elev + alpha * z;
        
        // Track variance (squared error of the estimation)
        const float diff = z - elev;
        var = (1.0f - alpha) * var + alpha * (diff * diff);
      }
    }
  }
  } catch (const std::runtime_error&) {
    return;
  }
}

} // namespace rover_topography::core
