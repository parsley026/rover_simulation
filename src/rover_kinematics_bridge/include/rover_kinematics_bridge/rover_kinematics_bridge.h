#include "rex_interfaces/msg/wheels.hpp"
#include "rex_interfaces/msg/wheel.hpp"
#include "rex_interfaces/msg/vesc_motor_command.hpp"
#include "rex_interfaces/msg/vesc_status.hpp"
#include "rex_interfaces/msg/rover_control.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <cmath>
#include <algorithm>
#include <memory>

#define FRONT_LEFT_DRIVE 0x50
#define FRONT_RIGHT_DRIVE 0x51
#define REAR_LEFT_DRIVE 0x52
#define REAR_RIGHT_DRIVE 0x53

#define FRONT_LEFT_TURN 0x60
#define FRONT_RIGHT_TURN 0x61
#define REAR_RIGHT_TURN 0x62
#define REAR_LEFT_TURN 0x63

#define ERPM_MODE 1
#define CURRENT_MODE 3

#define POLE_NUMBERS 15
#define WHEEL_RADIUS 0.128


class rover_kinematics_bridge : public rclcpp::Node {
public:
        rover_kinematics_bridge(const rclcpp::NodeOptions& options);
        ~rover_kinematics_bridge() = default;

        void onUpdate();

        rclcpp::TimerBase::SharedPtr timer_;
        

private:

        rclcpp::Duration publish_period_;

        rclcpp::Subscription<rex_interfaces::msg::Wheels>::SharedPtr kinematics_sub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr feedback_sub_;

        rclcpp::Publisher<rex_interfaces::msg::VescStatus>::SharedPtr feedback_pub_;

        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr front_left_drive_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr front_right_drive_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rear_right_drive_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rear_left_drive_pub_;

        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr front_left_steer_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr front_right_steer_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rear_right_steer_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rear_left_steer_pub_;

        rclcpp::Publisher<rex_interfaces::msg::VescStatus>::SharedPtr rear_left_drive_feedback_pub_;
        rclcpp::Publisher<rex_interfaces::msg::VescStatus>::SharedPtr rear_right_drive_feedback_pub_;
        rclcpp::Publisher<rex_interfaces::msg::VescStatus>::SharedPtr front_left_drive_feedback_pub_;
        rclcpp::Publisher<rex_interfaces::msg::VescStatus>::SharedPtr front_right_drive_feedback_pub_;
        rclcpp::Publisher<rex_interfaces::msg::VescStatus>::SharedPtr rear_left_turn_feedback_pub_;
        rclcpp::Publisher<rex_interfaces::msg::VescStatus>::SharedPtr rear_right_turn_feedback_pub_;
        rclcpp::Publisher<rex_interfaces::msg::VescStatus>::SharedPtr front_left_turn_feedback_pub_;
        rclcpp::Publisher<rex_interfaces::msg::VescStatus>::SharedPtr front_right_turn_feedback_pub_;

        void kinematicsCallback(const rex_interfaces::msg::Wheels::SharedPtr msg);
        void feedbackCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
        void roverControlCallback(const rex_interfaces::msg::RoverControl::SharedPtr msg);

        void publishFeedback();
        void publishWheelData();

        struct wheel_data{
                std_msgs::msg::Float64 front_left_drive;
                std_msgs::msg::Float64 front_right_drive;
                std_msgs::msg::Float64 rear_right_drive;
                std_msgs::msg::Float64 rear_left_drive;

                std_msgs::msg::Float64 front_left_steer;
                std_msgs::msg::Float64 front_right_steer;
                std_msgs::msg::Float64 rear_right_steer;
                std_msgs::msg::Float64 rear_left_steer;

        } ;

        std::shared_ptr<wheel_data> kinematic_data_current_;
        std::shared_ptr<wheel_data> kinematic_data_buffor_;

        std::shared_ptr<wheel_data> kinematic_feedback_current_;
        std::shared_ptr<wheel_data> kinematic_feedback_buffor_;

        std::shared_ptr<rex_interfaces::msg::VescStatus> kinematic_feedback_msg_;

        int control_mode_;
};