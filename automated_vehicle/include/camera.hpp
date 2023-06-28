#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

// timer
class GenericCameraNode : public rclcpp::Node
{
public:
    GenericCameraNode(std::string node_name) : Node(node_name)
    {
        // initialize the message
        msg_ = sensor_msgs::msg::Image();

        // publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera", 10);
        // parameters
        this->declare_parameter("frequency", 1.0);
        this->declare_parameter("bandwidth", 2.5);
        this->declare_parameter("name", "camera");
        camera_frequency_ = this->get_parameter("frequency").as_double();
        camera_bandwidth_ = this->get_parameter("bandwidth").as_double();
        camera_name_ = this->get_parameter("name").as_string();
        // timer callback
        timer_ = this->create_wall_timer(std::chrono::milliseconds((int)((1 / camera_frequency_) * 1000)),
                                         std::bind(&GenericCameraNode::timer_callback, this));
    }

private:
    // attributes
    sensor_msgs::msg::Image msg_;
    std::string camera_name_;
    double camera_frequency_;
    double camera_bandwidth_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

    // methods
    void timer_callback();
};