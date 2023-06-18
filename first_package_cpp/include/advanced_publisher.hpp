#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

// timer
class AdvancedPublisherNode : public rclcpp::Node
{
public:
    AdvancedPublisherNode(std::string node_name) : Node(node_name)
    {
        // initialize the message
        msg_ = std_msgs::msg::String();
        // timer callback
        timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(2000.0)),
                                         std::bind(&AdvancedPublisherNode::timer_callback, this));

        // publisher
        publisher_ = this->create_publisher<std_msgs::msg::String>("leia", 10);
    }

private:
    // attributes
    std_msgs::msg::String msg_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    // methods
    void timer_callback();
};