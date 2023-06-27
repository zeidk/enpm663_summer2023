#pragma once

#include <rclcpp/rclcpp.hpp>
#include <enpm663_msgs/srv/add_two_ints.hpp>
#include <chrono>
#include <iostream>
#include <memory>

class AddTwoIntsServer : public rclcpp::Node
{
public:
    AddTwoIntsServer()
        : Node("server_demo")
    {
        service_ = this->create_service<enpm663_msgs::srv::AddTwoInts>(
            "add_two_ints",
            std::bind(
                &AddTwoIntsServer::handleAddTwoInts,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "AddTwoInts Server has been started.");
    }

private:
    void handleAddTwoInts(const std::shared_ptr<enpm663_msgs::srv::AddTwoInts::Request> request,
                          std::shared_ptr<enpm663_msgs::srv::AddTwoInts::Response> response);

    rclcpp::Service<enpm663_msgs::srv::AddTwoInts>::SharedPtr service_;
};