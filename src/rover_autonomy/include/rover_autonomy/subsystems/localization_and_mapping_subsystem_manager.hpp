#ifndef ROVER_AUTONOMY__MAPPING_SUBSYSTEM_MANAGER_HPP_
#define ROVER_AUTONOMY__MAPPING_SUBSYSTEM_MANAGER_HPP_

#include "rover_autonomy/subsystems/composite_subsystem_manager.hpp"
#include "rover_autonomy/parameter_manager.hpp"

namespace rover_autonomy
{

class LocalizationAndMappingSubsystemManager : public CompositeSubsystemManager
{
public:
  LocalizationAndMappingSubsystemManager(
    rclcpp_lifecycle::LifecycleNode * parent_node,
    const MappingConfig & config);
};

}  // namespace rover_autonomy

#endif  // ROVER_AUTONOMY__MAPPING_SUBSYSTEM_MANAGER_HPP_
