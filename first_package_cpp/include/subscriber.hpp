#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

// timer
class SubscriberNode : public rclcpp::Node
{
public:
    SubscriberNode(std::string node_name) : Node(node_name)
    {
        subscriber_ = this->create_subscription<std_msgs::msg::String>("leia", 10, std::bind(
            &SubscriberNode::leia_callback, 
            this, 
            std::placeholders::_1));
    }

private:
    // attributes
    std_msgs::msg::String msg_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;

    // methods
    void leia_callback(const std_msgs::msg::String::SharedPtr msg);
};