#include <string>
#include "simple_subscriber.hpp"
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;


void SimpleSubscriber::initialize_()
{
    // initialize the message
    msg_ = std_msgs::msg::String();
    subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "leia", 10,
        std::bind(
            &SimpleSubscriber::leia_callback,
            this,
            std::placeholders::_1));
}

void SimpleSubscriber::leia_callback(const std_msgs::msg::String::SharedPtr msg)
{

    RCLCPP_INFO_STREAM(this->get_logger(), "Received: " << msg->data);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto simple_subscriber = std::make_shared<SimpleSubscriber>("simple_subscriber");
    rclcpp::spin(simple_subscriber);
    rclcpp::shutdown();
}