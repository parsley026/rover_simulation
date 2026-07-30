#include "rover_kinematics_bridge/rover_kinematics_bridge.h"

rover_kinematics_bridge::rover_kinematics_bridge(const rclcpp::NodeOptions& options) : 
Node("rover_kinematics_bridge", options),
publish_period_(rclcpp::Duration::from_seconds(0.05)) {
    
    timer_ = this->create_wall_timer(
        publish_period_.to_chrono<std::chrono::milliseconds>(),
        std::bind(&rover_kinematics_bridge::onUpdate, this));

    kinematics_sub_ = this->create_subscription<rex_interfaces::msg::Wheels>(
        "/CAN/TX/set_motor_vel", 10, std::bind(&rover_kinematics_bridge::kinematicsCallback, this, std::placeholders::_1));

    feedback_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::QoS(10).reliable(),
        std::bind(&rover_kinematics_bridge::feedbackCallback, this, std::placeholders::_1));

    front_left_drive_pub_ = this->create_publisher<std_msgs::msg::Float64>("/wheel/front_left/cmd_vel", 10);
    front_right_drive_pub_ = this->create_publisher<std_msgs::msg::Float64>("/wheel/front_right/cmd_vel", 10);
    rear_right_drive_pub_ = this->create_publisher<std_msgs::msg::Float64>("/wheel/rear_right/cmd_vel", 10);
    rear_left_drive_pub_ = this->create_publisher<std_msgs::msg::Float64>("/wheel/rear_left/cmd_vel", 10);

    front_left_steer_pub_ = this->create_publisher<std_msgs::msg::Float64>("/steer/front_left/cmd_pos", 10);
    front_right_steer_pub_ = this->create_publisher<std_msgs::msg::Float64>("/steer/front_right/cmd_pos", 10);
    rear_right_steer_pub_ = this->create_publisher<std_msgs::msg::Float64>("/steer/rear_right/cmd_pos", 10);
    rear_left_steer_pub_ = this->create_publisher<std_msgs::msg::Float64>("/steer/rear_left/cmd_pos", 10);

    rear_left_drive_feedback_pub_ = this->create_publisher<rex_interfaces::msg::VescStatus>("/CAN/RX/vesc_status", rclcpp::QoS(10).reliable());
    rear_right_drive_feedback_pub_ = this->create_publisher<rex_interfaces::msg::VescStatus>("/CAN/RX/vesc_status", rclcpp::QoS(10).reliable());
    front_left_drive_feedback_pub_ = this->create_publisher<rex_interfaces::msg::VescStatus>("/CAN/RX/vesc_status", rclcpp::QoS(10).reliable());
    front_right_drive_feedback_pub_ = this->create_publisher<rex_interfaces::msg::VescStatus>("/CAN/RX/vesc_status", rclcpp::QoS(10).reliable());
    
    rear_left_turn_feedback_pub_ = this->create_publisher<rex_interfaces::msg::VescStatus>("/CAN/RX/vesc_status", rclcpp::QoS(10).reliable());
    rear_right_turn_feedback_pub_ = this->create_publisher<rex_interfaces::msg::VescStatus>("/CAN/RX/vesc_status", rclcpp::QoS(10).reliable());
    front_left_turn_feedback_pub_ = this->create_publisher<rex_interfaces::msg::VescStatus>("/CAN/RX/vesc_status", rclcpp::QoS(10).reliable());
    front_right_turn_feedback_pub_ = this->create_publisher<rex_interfaces::msg::VescStatus>("/CAN/RX/vesc_status", rclcpp::QoS(10).reliable());
}

void rover_kinematics_bridge::onUpdate() {
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        kinematic_feedback_current_ = kinematic_feedback_buffor_;
        kinematic_data_current_     = kinematic_data_buffor_;
    }

    publishFeedback();

    if (this->count_publishers("/CAN/TX/set_motor_vel") == 0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No publishers found for /CAN/TX/set_motor_vel");
        
        
        kinematic_data_current_.front_left_drive.data  = 0.0;
        kinematic_data_current_.front_right_drive.data = 0.0;
        kinematic_data_current_.rear_right_drive.data  = 0.0;
        kinematic_data_current_.rear_left_drive.data   = 0.0;
    }

    publishWheelData();
}

void rover_kinematics_bridge::kinematicsCallback(const rex_interfaces::msg::Wheels::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    kinematic_data_buffor_.front_left_drive.data  = msg->front_left.drive.set_value * M_PI / 30 / POLE_NUMBERS;
    kinematic_data_buffor_.front_right_drive.data = msg->front_right.drive.set_value * M_PI / 30 / POLE_NUMBERS;
    kinematic_data_buffor_.rear_right_drive.data  = msg->rear_right.drive.set_value * M_PI / 30 / POLE_NUMBERS;
    kinematic_data_buffor_.rear_left_drive.data   = msg->rear_left.drive.set_value * M_PI / 30 / POLE_NUMBERS;

    kinematic_data_buffor_.front_left_steer.data = msg->front_left.turn.set_value * M_PI / 180;
    kinematic_data_buffor_.front_right_steer.data = msg->front_right.turn.set_value * M_PI / 180;
    kinematic_data_buffor_.rear_right_steer.data = msg->rear_right.turn.set_value * M_PI / 180;
    kinematic_data_buffor_.rear_left_steer.data = msg->rear_left.turn.set_value * M_PI / 180;
}

void rover_kinematics_bridge::feedbackCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    static const std::unordered_map<std::string, std::function<void(rover_kinematics_bridge*, double, double)>> joint_map = {
            {"arm_front_left_to_steer_front_left",    [](rover_kinematics_bridge* s, double p, double){ s->kinematic_feedback_buffor_.front_left_steer.data = p; }},
            {"arm_front_right_to_steer_front_right",  [](rover_kinematics_bridge* s, double p, double){ s->kinematic_feedback_buffor_.front_right_steer.data = p; }},
            {"arm_rear_right_to_steer_rear_right",    [](rover_kinematics_bridge* s, double p, double){ s->kinematic_feedback_buffor_.rear_right_steer.data = p; }},
            {"arm_rear_left_to_steer_rear_left",      [](rover_kinematics_bridge* s, double p, double){ s->kinematic_feedback_buffor_.rear_left_steer.data = p; }},
            
            {"steer_front_left_to_wheel_front_left",    [](rover_kinematics_bridge* s, double, double v){ s->kinematic_feedback_buffor_.front_left_drive.data = v; }},
            {"steer_front_right_to_wheel_front_right",  [](rover_kinematics_bridge* s, double, double v){ s->kinematic_feedback_buffor_.front_right_drive.data = v; }},
            {"steer_rear_right_to_wheel_rear_right",    [](rover_kinematics_bridge* s, double, double v){ s->kinematic_feedback_buffor_.rear_right_drive.data = v; }},
            {"steer_rear_left_to_wheel_rear_left",      [](rover_kinematics_bridge* s, double, double v){ s->kinematic_feedback_buffor_.rear_left_drive.data = v; }}
    };

    for (size_t i = 0; i < msg->name.size(); ++i) {
        auto it = joint_map.find(msg->name[i]);
        if (it != joint_map.end()) {
            const double pos = (i < msg->position.size()) ? msg->position[i] : 0.0;
            const double vel = (i < msg->velocity.size()) ? msg->velocity[i] : 0.0;
            it->second(this, pos, vel);
        }
    }
}
    
void rover_kinematics_bridge::publishFeedback() {
    rclcpp::Time current_time = this->get_clock()->now();

    auto publish_feedback = [this, current_time](rclcpp::Publisher<rex_interfaces::msg::VescStatus>::SharedPtr pub, int vesc_id, double erpm, double precise_pos = 0.0) {
        // Optimized: Create the message on the stack. No heap allocation required.
        rex_interfaces::msg::VescStatus feedback_msg;
        feedback_msg.header.stamp = current_time;
        feedback_msg.vesc_id = vesc_id;
        feedback_msg.erpm = erpm;
        feedback_msg.precise_pos = precise_pos;

        pub->publish(feedback_msg);
    };

    publish_feedback(rear_left_drive_feedback_pub_, REAR_LEFT_DRIVE,
                     kinematic_feedback_current_.rear_left_drive.data * POLE_NUMBERS * 30 / M_PI);
    publish_feedback(rear_right_drive_feedback_pub_, REAR_RIGHT_DRIVE,
                     kinematic_feedback_current_.rear_right_drive.data * POLE_NUMBERS * 30 / M_PI);
    publish_feedback(front_left_drive_feedback_pub_, FRONT_LEFT_DRIVE,
                     kinematic_feedback_current_.front_left_drive.data * POLE_NUMBERS * 30 / M_PI);
    publish_feedback(front_right_drive_feedback_pub_, FRONT_RIGHT_DRIVE,
                     kinematic_feedback_current_.front_right_drive.data * POLE_NUMBERS * 30 / M_PI);

    publish_feedback(rear_left_turn_feedback_pub_, REAR_LEFT_TURN,
                     0.0, kinematic_feedback_current_.rear_left_steer.data * 180 / M_PI);
    publish_feedback(rear_right_turn_feedback_pub_, REAR_RIGHT_TURN,
                     0.0, kinematic_feedback_current_.rear_right_steer.data * 180 / M_PI);
    publish_feedback(front_left_turn_feedback_pub_, FRONT_LEFT_TURN,
                     0.0, kinematic_feedback_current_.front_left_steer.data * 180 / M_PI);
    publish_feedback(front_right_turn_feedback_pub_, FRONT_RIGHT_TURN,
                     0.0, kinematic_feedback_current_.front_right_steer.data * 180 / M_PI);
}

void rover_kinematics_bridge::publishWheelData() {
    front_left_drive_pub_->publish(kinematic_data_current_.front_left_drive);
    front_right_drive_pub_->publish(kinematic_data_current_.front_right_drive);
    rear_right_drive_pub_->publish(kinematic_data_current_.rear_right_drive);
    rear_left_drive_pub_->publish(kinematic_data_current_.rear_left_drive);

    front_left_steer_pub_->publish(kinematic_data_current_.front_left_steer);
    front_right_steer_pub_->publish(kinematic_data_current_.front_right_steer);
    rear_right_steer_pub_->publish(kinematic_data_current_.rear_right_steer);
    rear_left_steer_pub_->publish(kinematic_data_current_.rear_left_steer);
}