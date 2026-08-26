#ifndef ROVER_AUTONOMY__ODOMETRY_SUBSYSTEM_MANAGER_HPP_
#define ROVER_AUTONOMY__ODOMETRY_SUBSYSTEM_MANAGER_HPP_

#include "rover_autonomy/subsystems/composite_subsystem_manager.hpp"
#include "rover_autonomy/parameter_manager.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace rover_autonomy
{

class OdometrySubsystemManager : public CompositeSubsystemManager
{
public:
  OdometrySubsystemManager(
    rclcpp_lifecycle::LifecycleNode * parent_node,
    const OdometryConfig & config);

  ~OdometrySubsystemManager() override = default;
};

}  // namespace rover_autonomy

#endif  // ROVER_AUTONOMY__ODOMETRY_SUBSYSTEM_MANAGER_HPP_
