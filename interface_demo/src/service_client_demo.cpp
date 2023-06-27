#include <rclcpp/rclcpp.hpp>
#include "service_client_demo.hpp"
#include <chrono>
#include <memory>
#include <iostream>
#include <string>

using namespace std::chrono_literals;

void AddTwoIntsClient::call_client_cb()
{

    // Providing a seed value
    srand((unsigned)time(NULL));

    // Get a random number
    int a = rand() % 10;
    int b = rand() % 10;

    call_client(a, b);
}



void AddTwoIntsClient::response_callback(rclcpp::Client<enpm663_msgs::srv::AddTwoInts>::SharedFuture future)
{
    auto status = future.wait_for(1s);
    if (status == std::future_status::ready)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Result:" << future.get()->sum);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
}
void AddTwoIntsClient::call_client(int a, int b)
{
    // Wait for the service to become available
    while (!client_->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    // Create a request and send it to the server
    auto request = std::make_shared<enpm663_msgs::srv::AddTwoInts::Request>();
    request->a = a;
    request->b = b;

    auto future_result = client_->async_send_request(request, std::bind(&AddTwoIntsClient::response_callback, this, std::placeholders::_1));
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}