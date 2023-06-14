#pragma once

#include <rclcpp/rclcpp.hpp>

class SimpleNode : public rclcpp::Node
{
public:
    SimpleNode(std::string node_name) : Node(node_name)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), node_name << " is running");
    }

private:
};