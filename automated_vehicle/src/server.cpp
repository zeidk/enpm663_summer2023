#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "server.hpp"

void ServerNode::left_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    RCLCPP_INFO_STREAM(this->get_logger(), "Received data from Topic /left: " << msg->header.frame_id);
    while (true)
    {
        /* code */
    }
    
}

void ServerNode::right_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    RCLCPP_INFO_STREAM(this->get_logger(), "Received data from Topic /right: " << msg->header.frame_id);
}

void ServerNode::front_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    RCLCPP_INFO_STREAM(this->get_logger(), "Received data from Topic /front: " << msg->header.frame_id);
}

void ServerNode::rear_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    RCLCPP_INFO_STREAM(this->get_logger(), "Received data from Topic /rear: " << msg->header.frame_id);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto server_node = std::make_shared<ServerNode>("server_node");
    rclcpp::spin(server_node);
    rclcpp::shutdown();
}