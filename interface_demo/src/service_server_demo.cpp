
#include <rclcpp/rclcpp.hpp>
#include "service_server_demo.hpp"

void AddTwoIntsServer::handleAddTwoInts(const std::shared_ptr<enpm663_msgs::srv::AddTwoInts::Request> request,
                                      std::shared_ptr<enpm663_msgs::srv::AddTwoInts::Response> response)
{
    response->sum = request->a + request->b;
    RCLCPP_INFO_STREAM(this->get_logger(), "Received request: " << request->a << " + " << request->b << " = " << response->sum);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsServer>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}