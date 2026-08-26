#ifndef ROVER_AUTONOMY__DESCRIPTION_SUBSYSTEM_MANAGER_HPP_
#define ROVER_AUTONOMY__DESCRIPTION_SUBSYSTEM_MANAGER_HPP_

#include "rover_autonomy/subsystems/composite_subsystem_manager.hpp"
#include "rover_autonomy/parameter_manager.hpp"

namespace rover_autonomy
{

class DescriptionSubsystemManager : public CompositeSubsystemManager
{
public:
  DescriptionSubsystemManager(
    rclcpp_lifecycle::LifecycleNode * parent_node,
    const SubsystemConfig & config);
};

}  // namespace rover_autonomy

#endif  // ROVER_AUTONOMY__DESCRIPTION_SUBSYSTEM_MANAGER_HPP_