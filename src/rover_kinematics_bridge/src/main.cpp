#include <rclcpp/rclcpp.hpp>
#include "rover_kinematics_bridge/rover_kinematics_bridge.h"
#include <rclcpp/executors/multi_threaded_executor.hpp>

int main(int argc, char** argv) {
    // Initialize the ROS2 node
    rclcpp::init(argc, argv);

    // Create the rover_kinematics_bridge node
    
    auto node = std::make_shared<rover_kinematics_bridge>(rclcpp::NodeOptions());

    // Define the loop rate (frequency in Hz)
    rclcpp::Rate loop_rate(50); // 50 Hz

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4 /* number of threads */);

    // Add your node to the executor
    executor.add_node(node);

    // Spin. This will block until shutdown.
    executor.spin();

    // // Main loop
    // while (rclcpp::ok()) {
    //     node->onUpdate();
    //     rclcpp::spin_some(node);
    //     loop_rate.sleep(); // Sleep to maintain the loop rate
    // }

    RCLCPP_INFO(node->get_logger(), "Node shutting down");

    rclcpp::shutdown();
    return 0;
}