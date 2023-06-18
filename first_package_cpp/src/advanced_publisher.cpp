#include <rclcpp/rclcpp.hpp>
#include "advanced_publisher.hpp"

void AdvancedPublisherNode::timer_callback()
{
    msg_.data = "Help me Obi-Wan Kenobi, you are my only hope.";
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: " << msg_.data);
    publisher_->publish(msg_);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto publisher_node = std::make_shared<AdvancedPublisherNode>("advanced_publisher");
    rclcpp::spin(publisher_node);
    rclcpp::shutdown();
}