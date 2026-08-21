#include <gtest/gtest.h>
#include "rover_topography/core/map_bounds_strategy.hpp"
#include "rover_topography/core/elevation_fuser.hpp"
#include <grid_map_core/grid_map_core.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace rover_topography::core;

TEST(TopographyCoreTest, CircularBufferStrategyShiftsCorrectly) {
  grid_map::GridMap map({"elevation", "variance"});
  map.setGeometry(grid_map::Length(10.0, 10.0), 0.1);
  map.setFrameId("map");

  CircularBufferStrategy strategy;
  sensor_msgs::msg::PointCloud2 empty_cloud;

  // Move robot to 2.5, 3.5
  grid_map::Position robot_pos(2.5, 3.5);
  strategy.updateBounds(map, robot_pos, empty_cloud);

  // The map center should now be aligned near the robot position, discretized by resolution
  EXPECT_NEAR(map.getPosition().x(), 2.5, 0.1);
  EXPECT_NEAR(map.getPosition().y(), 3.5, 0.1);
}

TEST(TopographyCoreTest, ElevationFuserEmptyCloudHandling) {
  grid_map::GridMap map({"elevation", "variance"});
  map.setGeometry(grid_map::Length(2.0, 2.0), 0.1, grid_map::Position(0.0, 0.0));
  
  sensor_msgs::msg::PointCloud2 empty_cloud;
  
  // Verify fusion doesn't crash on empty/invalid point clouds in headless mode
  EXPECT_NO_THROW(ElevationFuser::fuse(map, empty_cloud, false));
  EXPECT_NO_THROW(ElevationFuser::fuse(map, empty_cloud, true));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
