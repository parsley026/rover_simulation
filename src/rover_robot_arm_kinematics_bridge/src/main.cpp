#include <rclcpp/rclcpp.hpp>
#include "rover_robot_arm_kinematics_bridge/rover_robot_arm_kinematics_bridge.h"
#include <rclcpp/executors/multi_threaded_executor.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rover_robot_arm_kinematics_bridge::rover_robot_arm_kinematics_bridge>(rclcpp::NodeOptions());

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4 /* number of threads */);
    executor.add_node(node);
    executor.spin();

    RCLCPP_INFO(node->get_logger(), "Node shutting down");
    rclcpp::shutdown();
    return 0;
}
