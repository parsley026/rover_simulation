#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/generic_publisher.hpp>
#include <string>
#include <vector>
#include <memory>

struct HeldChannel {
  std::string input_topic;
  std::string output_topic;
  std::string msg_type;
  double max_stale_sec;
  std::shared_ptr<rclcpp::SerializedMessage> last_msg;
  rclcpp::Time last_stamp{0, 0, RCL_ROS_TIME};
  std::shared_ptr<rclcpp::GenericSubscription> sub;
  std::shared_ptr<rclcpp::GenericPublisher> pub;
};

class GenericTemporalSyncNode : public rclcpp::Node {
public:
  GenericTemporalSyncNode() : Node("generic_temporal_sync_node") {
    // 1. Declare & Parse Trigger Topic
    this->declare_parameter<std::string>("trigger_topic.name", "");
    this->declare_parameter<std::string>("trigger_topic.type", "");
    this->declare_parameter<std::string>("trigger_topic.output_topic", "");

    std::string trigger_in = this->get_parameter("trigger_topic.name").as_string();
    std::string trigger_type = this->get_parameter("trigger_topic.type").as_string();
    std::string trigger_out = this->get_parameter("trigger_topic.output_topic").as_string();

    trigger_pub_ = this->create_generic_publisher(trigger_out, trigger_type, 10);
    trigger_sub_ = this->create_generic_subscription(
      trigger_in, trigger_type, rclcpp::SensorDataQoS(),
      std::bind(&GenericTemporalSyncNode::on_trigger, this, std::placeholders::_1));

    // 2. Declare & Parse Held Topics
    this->declare_parameter<std::vector<std::string>>("held_topic_names", std::vector<std::string>());
    auto held_names = this->get_parameter("held_topic_names").as_string_array();

    for (const auto &name : held_names) {
      HeldChannel channel;
      this->declare_parameter<std::string>("held_topics." + name + ".name", "");
      this->declare_parameter<std::string>("held_topics." + name + ".type", "");
      this->declare_parameter<std::string>("held_topics." + name + ".output_topic", "");
      this->declare_parameter<double>("held_topics." + name + ".max_stale_sec", 0.5);

      channel.input_topic = this->get_parameter("held_topics." + name + ".name").as_string();
      channel.msg_type = this->get_parameter("held_topics." + name + ".type").as_string();
      channel.output_topic = this->get_parameter("held_topics." + name + ".output_topic").as_string();
      channel.max_stale_sec = this->get_parameter("held_topics." + name + ".max_stale_sec").as_double();

      channel.pub = this->create_generic_publisher(channel.output_topic, channel.msg_type, 10);
      
      // We must capture the channel index carefully to update the correct last_msg
      auto idx = held_channels_.size();
      
      // Add the channel to the list first so we don't invalidate references
      held_channels_.push_back(channel);
      
      // We need to bind the subscription to update the corresponding element in the vector
      held_channels_.back().sub = this->create_generic_subscription(
        channel.input_topic, channel.msg_type, rclcpp::SensorDataQoS(),
        [this, idx](std::shared_ptr<rclcpp::SerializedMessage> msg) {
          this->held_channels_[idx].last_msg = msg;
          this->held_channels_[idx].last_stamp = this->now();
        });
    }
  }

private:
  void on_trigger(std::shared_ptr<rclcpp::SerializedMessage> trigger_msg) {
    rclcpp::Time now = this->now();

    // Check all held channels for validity and staleness
    for (auto &channel : held_channels_) {
      if (!channel.last_msg) return; // Drop trigger if we are missing any held message
      if ((now - channel.last_stamp).seconds() > channel.max_stale_sec) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "Held topic %s is stale!", channel.input_topic.c_str());
        return;
      }
    }

    // Publish trigger and held messages in lockstep
    trigger_pub_->publish(*trigger_msg);
    for (auto &channel : held_channels_) {
      channel.pub->publish(*channel.last_msg);
    }
  }

  std::shared_ptr<rclcpp::GenericSubscription> trigger_sub_;
  std::shared_ptr<rclcpp::GenericPublisher> trigger_pub_;
  std::vector<HeldChannel> held_channels_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GenericTemporalSyncNode>());
  rclcpp::shutdown();
  return 0;
}
