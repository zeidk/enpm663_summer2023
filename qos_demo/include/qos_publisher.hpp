#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

// timer
class QoSPublisher : public rclcpp::Node
{
public:
    QoSPublisher(std::string node_name) : Node(node_name)
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

    // methods
    void publisher_timer_cb_();
    void publish_();
    void initialize_();
    void print_qos(const rclcpp::QoS &qos);
    double rmw_time_to_seconds(const rmw_time_t &time);
};