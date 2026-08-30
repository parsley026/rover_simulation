#ifndef ROVER_AUTONOMY__I_SUBSYSTEM_MANAGER_HPP_
#define ROVER_AUTONOMY__I_SUBSYSTEM_MANAGER_HPP_

#include <string>
#include <memory>

namespace rover_autonomy
{

enum class SubsystemState {
  UNCONFIGURED,
  INACTIVE,
    ACTIVE,
  DEGRADED,
  FAILED
};

class SubsystemManager
{
public:
  virtual ~SubsystemManager() = default;

  virtual bool on_configure()  = 0;
  virtual bool on_activate()   = 0;
  virtual bool on_deactivate() = 0;
  virtual bool on_cleanup()    = 0;

  virtual SubsystemState get_state() const = 0;
  virtual bool is_healthy() const = 0;
  virtual bool is_enabled() const = 0;
  virtual void set_enabled(bool enabled) = 0;
  
  virtual bool recover() = 0;
};

}  // namespace rover_autonomy

#endif  // ROVER_AUTONOMY__I_SUBSYSTEM_MANAGER_HPP_