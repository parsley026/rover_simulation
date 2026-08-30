#ifndef ROVER_AUTONOMY_UTILS_STREAM_HEALTH_MONITOR_HPP_
#define ROVER_AUTONOMY_UTILS_STREAM_HEALTH_MONITOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <deque>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

namespace rover_autonomy {
namespace utils {

template <typename MsgT>
class StreamHealthMonitor {
public:
  StreamHealthMonitor(double expected_hz, double timeout_sec)
  : expected_hz_(expected_hz), timeout_sec_(timeout_sec), current_hz_(0.0) {}

  void tick(const rclcpp::Time& msg_stamp, const rclcpp::Time& now) {
    timestamps_.push_back(msg_stamp);
    last_tick_now_ = now;

    // Remove old timestamps outside of a 1-second rolling window for Hz calculation
    while (!timestamps_.empty() && (msg_stamp - timestamps_.front()).seconds() > 1.0) {
      timestamps_.pop_front();
    }

    if (timestamps_.size() > 1) {
      double window_duration = (timestamps_.back() - timestamps_.front()).seconds();
      if (window_duration > 0.0) {
        current_hz_ = static_cast<double>(timestamps_.size() - 1) / window_duration;
      }
    } else {
      current_hz_ = 0.0;
    }
  }

  int8_t get_health_status(const rclcpp::Time& now) const {
    if (timestamps_.empty()) {
      return diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    }

    double time_since_last_msg = (now - last_tick_now_).seconds();
    if (time_since_last_msg > timeout_sec_) {
      return diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    }

    if (expected_hz_ > 0 && current_hz_ < (expected_hz_ * 0.5)) {
      return diagnostic_msgs::msg::DiagnosticStatus::WARN;
    }

    return diagnostic_msgs::msg::DiagnosticStatus::OK;
  }

  double get_current_hz() const {
    return current_hz_;
  }

private:
  double expected_hz_;
  double timeout_sec_;
  double current_hz_;
  std::deque<rclcpp::Time> timestamps_;
  rclcpp::Time last_tick_now_{0, 0, RCL_ROS_TIME};
};

}  // namespace utils
}  // namespace rover_autonomy

#endif  // ROVER_AUTONOMY_UTILS_STREAM_HEALTH_MONITOR_HPP_
