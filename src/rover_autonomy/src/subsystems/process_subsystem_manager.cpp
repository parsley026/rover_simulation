#include "rover_autonomy/subsystems/process_subsystem_manager.hpp"
#include <unistd.h>
#include <signal.h>
#include <sys/wait.h>
#include <thread>
#include <chrono>

namespace rover_autonomy
{

ProcessSubsystemManager::ProcessSubsystemManager(
  rclcpp_lifecycle::LifecycleNode * parent_node,
  const SubsystemConfig & config)
: parent_node_(parent_node),
  package_name_(config.package_name),
  launch_file_(config.launch_script),
  enabled_(config.launch_enabled),
  launch_arguments_(config.launch_args)
{
}

void ProcessSubsystemManager::add_launch_argument(const std::string& key, const std::string& value)
{
  launch_arguments_[key] = value;
}

bool ProcessSubsystemManager::is_enabled() const
{
  return enabled_;
}

void ProcessSubsystemManager::set_enabled(bool enabled)
{
  enabled_ = enabled;
}

ProcessSubsystemManager::~ProcessSubsystemManager()
{
  if (current_state_ == SubsystemState::ACTIVE) {
    on_deactivate();
  }
}

bool ProcessSubsystemManager::on_configure()
{
  if (current_state_ != SubsystemState::UNCONFIGURED) return false;
  
  // [EXTENSION POINT]: Derived classes can override this method to load specific YAML 
  // parameters, prep variables, or initialize ROS 2 subscribers for health monitoring
  // before the process is launched. Call ProcessSubsystemManager::configure() from the override.
  current_state_ = SubsystemState::INACTIVE;
  return true;
}

bool ProcessSubsystemManager::on_activate()
{
  if (current_state_ != SubsystemState::INACTIVE && current_state_ != SubsystemState::FAILED) return false;

  if (!enabled_) {
    RCLCPP_INFO(parent_node_->get_logger(), "Subsystem %s is disabled. Staying in INACTIVE state.", launch_file_.c_str());
    current_state_ = SubsystemState::INACTIVE; // Safely stay inactive
    return true; 
  }

  if (start_process()) {
    current_state_ = SubsystemState::ACTIVE;
    return true;
  }
  
  current_state_ = SubsystemState::FAILED;
  return false;
}

bool ProcessSubsystemManager::on_deactivate()
{
  if (current_state_ != SubsystemState::ACTIVE && current_state_ != SubsystemState::DEGRADED) return false;

  stop_process();
  current_state_ = SubsystemState::INACTIVE;
  return true;
}

bool ProcessSubsystemManager::on_cleanup()
{
  if (current_state_ != SubsystemState::INACTIVE) return false;
  
  current_state_ = SubsystemState::UNCONFIGURED;
  return true;
}

SubsystemState ProcessSubsystemManager::get_state() const
{
  return current_state_;
}

// --- OS-Level Process Management ---

bool ProcessSubsystemManager::start_process()
{
  pid_t pid = fork();

  if (pid == 0) {
    // CHILD PROCESS: Create a new Process Group so we can kill the whole tree later
    setpgid(0, 0);
    
    std::vector<std::string> args;
    args.push_back("ros2");
    args.push_back("launch");
    args.push_back(package_name_);
    args.push_back(launch_file_);

    for (const auto& kv : launch_arguments_) {
      args.push_back(kv.first + ":=" + kv.second);
    }

    std::vector<char*> exec_args;
    for (const auto& arg : args) {
      exec_args.push_back(const_cast<char*>(arg.c_str()));
    }
    exec_args.push_back(nullptr);

    execvp("ros2", exec_args.data());
    
    // If execvp fails, exit the child process
    exit(1);
  } else if (pid > 0) {
    // PARENT PROCESS (Main Brain): Save the Process Group ID
    launch_pgid_ = pid;
    return true;
  }

  return false; // Fork failed
}

bool ProcessSubsystemManager::stop_process()
{
  if (launch_pgid_ <= 0) return true;

  // 1. Polite Request: Send SIGINT to the entire Process Group
  kill(-launch_pgid_, SIGINT);

  // 2. Wait up to 5 seconds for graceful MultiThreadedExecutor shutdown
  auto start_time = std::chrono::steady_clock::now();
  bool safely_closed = false;

  while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(5)) {
    int status;
    pid_t result = waitpid(-launch_pgid_, &status, WNOHANG);
    if (result == -1 || result > 0) {
      safely_closed = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // 3. The Hammer: If it hung, forcefully kill the group
  if (!safely_closed) {
    RCLCPP_ERROR(parent_node_->get_logger(), "Subsystem %s hung. Issuing SIGKILL.", launch_file_.c_str());
    kill(-launch_pgid_, SIGKILL);
    waitpid(-launch_pgid_, nullptr, 0); // Clean up the zombie
  }

  launch_pgid_ = -1;
  return true;
}

}  // namespace rover_autonomy

// --- Derived Class Implementation Note ---
// To use this manager, create a derived class (e.g., CameraSubsystem) that inherits 
// from ProcessSubsystemManager. In your derived class, you MUST implement the 
// is_healthy() method (defined as pure virtual in the header) to define how you 
// monitor this specific process (e.g., subscribing to a topic and tracking the timestamp).