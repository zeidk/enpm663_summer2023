#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

// timer
class QoSSubscriber : public rclcpp::Node
{
public:
    QoSSubscriber(std::string node_name) : Node(node_name)
    {
        initialize_();
    }

private:
    // attributes
    std_msgs::msg::String msg_;
    rclcpp::TimerBase::SharedPtr counter_timer_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
    unsigned int count_;

    // methods
    void leia_callback(const std_msgs::msg::String::SharedPtr msg);
    void counter_timer_cb_();
    void initialize_();
};