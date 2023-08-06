#include <string>
#include "qos_subscriber.hpp"
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

void QoSSubscriber::initialize_()
{
    // initialize the count
    count_ = 0;
    // initialize the message
    msg_ = std_msgs::msg::String();
    // start counter timer
    counter_timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)(2000.0)),
        std::bind(&QoSSubscriber::counter_timer_cb_, this));
}

void QoSSubscriber::leia_callback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO_STREAM(this->get_logger(), "Received: " << msg->data);
}

void QoSSubscriber::counter_timer_cb_()
{
    // update the count
    count_++;
    if (count_ == 5)
    {
        rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepAll()).best_effort();
        subscriber_ = this->create_subscription<std_msgs::msg::String>("leia", qos_profile, std::bind(&QoSSubscriber::leia_callback, this, std::placeholders::_1));
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto qos_subscriber = std::make_shared<QoSSubscriber>("qos_subscriber");
    rclcpp::spin(qos_subscriber);
    rclcpp::shutdown();
}