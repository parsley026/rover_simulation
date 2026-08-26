#include "rover_autonomy/subsystems/description_subsystem_manager.hpp"
#include "tf2_msgs/msg/tf_message.hpp" 
#include <chrono>
#include <thread>

namespace rover_autonomy
{

class DescriptionProcess : public ProcessSubsystemManager
{
public:
  using ProcessSubsystemManager::ProcessSubsystemManager;

  bool on_configure() override
  {
    if (!ProcessSubsystemManager::on_configure()) {
      return false;
    }

    last_heartbeat_time_ = parent_node_->get_clock()->now();
    
    tf_sub_ = parent_node_->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_static", 10,
      std::bind(&DescriptionProcess::tf_callback, this, std::placeholders::_1)
    );

    return true;
  }

  bool is_healthy() const override
  {
    if (current_state_ != SubsystemState::ACTIVE) {
      return false;
    }

    auto now = parent_node_->get_clock()->now();
    if ((now - last_heartbeat_time_).seconds() > 2.0) {
      RCLCPP_WARN(parent_node_->get_logger(), 
        "Description subsystem health check failed: No TF data received recently.");
      return false;
    }

    return true;
  }

  bool recover() override
  {
    RCLCPP_WARN(parent_node_->get_logger(), "Recovering subsystem: %s", launch_file_.c_str());
    on_deactivate();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    return on_activate();
  }

private:
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  mutable rclcpp::Time last_heartbeat_time_;

  void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr /*msg*/)
  {
    last_heartbeat_time_ = parent_node_->get_clock()->now();
  }
};

DescriptionSubsystemManager::DescriptionSubsystemManager(
  rclcpp_lifecycle::LifecycleNode * parent_node,
  const SubsystemConfig & config)
: CompositeSubsystemManager(parent_node)
{
  child_processes_.push_back(std::make_unique<DescriptionProcess>(parent_node, config));
}

}  // namespace rover_autonomy