#pragma once

#include <rclcpp/rclcpp.hpp>
#include <enpm663_msgs/srv/add_two_ints.hpp>
#include <chrono>
#include <iostream>
#include <memory>

/**
 * @brief Class for the client
 *
 */
class AddTwoIntsClient : public rclcpp::Node
{
public:
    AddTwoIntsClient() : Node("client_demo_cpp")
    {
        client_ = this->create_client<enpm663_msgs::srv::AddTwoInts>("add_two_ints");

        // timer callback
        timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(2000.0)),
                                         std::bind(&AddTwoIntsClient::timer_cb, this));
    }

private:
    /**
     * @brief Shared pointer to create_wall_timer object
     *
     */
    rclcpp::TimerBase::SharedPtr timer_;
    /**
     * @brief Shared pointer to create_client object
     *
     */
    rclcpp::Client<enpm663_msgs::srv::AddTwoInts>::SharedPtr client_;
    /**
     * @brief Call the client
     *
     */
    void call_client(int a, int b);
    /**
     * @brief Callback function for the timer
     *
     */
    void timer_cb();
    /**
     * @brief Callback function for the client
     *
     * This function is called when the client receives a response from the server
     * @param future Shared pointer to the future object.
     *  A future is a value that indicates whether the call and response is finished (not the value of the response itself)
     */
    void response_callback(rclcpp::Client<enpm663_msgs::srv::AddTwoInts>::SharedFuture future);
};