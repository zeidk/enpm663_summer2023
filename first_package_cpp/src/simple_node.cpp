#include <rclcpp/rclcpp.hpp>
#include "simple_node.hpp"


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleNode>("simple_cpp_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
}