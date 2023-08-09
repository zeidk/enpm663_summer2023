#include <rclcpp/rclcpp.hpp>
#include "simple_publisher.hpp"

void SimplePublisher::publisher_timer_cb_()
{
    publish_();
}

void SimplePublisher::publish_()
{
    // update the message using count_
    msg_.data = std::to_string(count_) + ": Help me Obi-Wan Kenobi, you are my only hope.";
    RCLCPP_INFO_STREAM(this->get_logger(), msg_.data);
    publisher_->publish(msg_);

    count_++;
}

void SimplePublisher::initialize_()
{
    count_ = 0;
    // initialize the message
    msg_ = std_msgs::msg::String();
    // publisher timer
    publish_timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(2000.0)), std::bind(&SimplePublisher::publisher_timer_cb_, this));

    // publisher
    publisher_ = this->create_publisher<std_msgs::msg::String>("leia", 10);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto simple_publisher = std::make_shared<SimplePublisher>("simple_publisher");
    rclcpp::spin(simple_publisher);
    rclcpp::shutdown();
}