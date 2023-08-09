#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

// timer
class SimplePublisher : public rclcpp::Node
{
public:
    SimplePublisher(std::string node_name) : Node(node_name)
    {
        initialize_();
    }

private:
    // attributes
    std_msgs::msg::String msg_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::TimerBase::SharedPtr counter_timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    unsigned int count_;
    // parameters
    std::string qos_reliability_param_;
    std::string qos_durability_param_;

    // methods
    void publisher_timer_cb_();
    void publish_();
    void initialize_();

};