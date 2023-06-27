#pragma once

#include <rclcpp/rclcpp.hpp>
#include <enpm663_msgs/srv/add_two_ints.hpp>
#include <chrono>
#include <iostream>
#include <memory>

class AddTwoIntsClient : public rclcpp::Node
{
public:
    AddTwoIntsClient() : Node("client_demo_cpp")
    {
        client_ = this->create_client<enpm663_msgs::srv::AddTwoInts>("add_two_ints");

        // timer callback
        timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(2000.0)),
                                         std::bind(&AddTwoIntsClient::call_client_cb, this));
    }

private:
    void call_client(int a, int b);
    rclcpp::TimerBase::SharedPtr timer_;
    void response_callback(rclcpp::Client<enpm663_msgs::srv::AddTwoInts>::SharedFuture future);
    rclcpp::Client<enpm663_msgs::srv::AddTwoInts>::SharedPtr client_;
    void call_client_cb();
};