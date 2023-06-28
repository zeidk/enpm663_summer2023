#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "rcl_interfaces/msg/set_parameters_result.hpp"

// timer
class ParameterDemoNode : public rclcpp::Node
{
public:
    ParameterDemoNode(std::string node_name) : Node(node_name)
    {
        // initialize the message
        msg_ = std_msgs::msg::String();
        
        // publisher
        publisher_ = this->create_publisher<std_msgs::msg::String>("leia", 10);
        // parameter
        this->declare_parameter("jedi", "Obi-Wan Kenobi");
        jedi_ = this->get_parameter("jedi").as_string();
        // timer callback
        timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(2000.0)),
                                         std::bind(&ParameterDemoNode::timer_callback, this));

        // callback for parameter change
        callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&ParameterDemoNode::parametersCallback, this, std::placeholders::_1));
    }

private:
    // attributes
    std_msgs::msg::String msg_;
    std::string jedi_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;

    // methods
    void timer_callback();
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);
};