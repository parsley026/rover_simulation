#pragma once
#include <grid_map_core/grid_map_core.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace rover_topography::core {

class ElevationFuser {
public:
  // Ingests point cloud into map. Explicitly tracks variance for Kalman-like smoothing.
  static void fuse(grid_map::GridMap & map, const sensor_msgs::msg::PointCloud2 & cloud, bool is_global);
};

} // namespace rover_topography::core
