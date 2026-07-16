#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "rover_kinematics/KinematicsNode.hpp"


/**
 * @brief Program entry point for the `rover_kinematics` node.
 *
 * Initializes ROS 2, constructs a `KinematicsNode` configured from the package's
 * `config/rover_config.yaml` file and spins a multi-threaded executor.
 */
int main(int argc, char** argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Get the config file path
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("rover_kinematics");
    std::string cfg_pkg_path = package_share_directory + "/config/rover_config.yaml";

    // Create the Rover node
    auto node = std::make_shared<KinematicsNode>(rclcpp::NodeOptions(), cfg_pkg_path);

    // Create a multi-threaded executor
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4 /* number of threads */);

    // Add your node to the executor
    executor.add_node(node);

    // Spin. This will block until shutdown.
    executor.spin();

    // Cleanup
    rclcpp::shutdown();
    return 0;
}