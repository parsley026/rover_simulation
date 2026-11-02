#include <rclcpp/rclcpp.hpp>
#include "manipulator_simple/ManipulatorSimple.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ManipulatorSimple>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
