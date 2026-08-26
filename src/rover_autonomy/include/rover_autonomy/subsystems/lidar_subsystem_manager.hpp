#ifndef ROVER_AUTONOMY__LIDAR_SUBSYSTEM_MANAGER_HPP_
#define ROVER_AUTONOMY__LIDAR_SUBSYSTEM_MANAGER_HPP_

#include "rover_autonomy/subsystems/composite_subsystem_manager.hpp"
#include "rover_autonomy/parameter_manager.hpp"

namespace rover_autonomy
{

class LidarSubsystemManager : public CompositeSubsystemManager
{
public:
  LidarSubsystemManager(
    rclcpp_lifecycle::LifecycleNode * parent_node,
    const SubsystemConfig & config);
};

}  // namespace rover_autonomy

#endif  // ROVER_AUTONOMY__LIDAR_SUBSYSTEM_MANAGER_HPP_
