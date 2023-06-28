#include <rclcpp/rclcpp.hpp>
#include "parameter_demo.hpp"

void ParameterDemoNode::timer_callback()
{
    msg_.data = "Help me " + jedi_ + ", you are my only hope.";
    RCLCPP_INFO_STREAM(this->get_logger(), msg_.data);
    publisher_->publish(msg_);
}

rcl_interfaces::msg::SetParametersResult ParameterDemoNode::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    for (const auto &param : parameters)
    {
        if (param.get_name() == "jedi")
        {
            jedi_ = param.as_string();
        }
        else
        {
            result.successful = false;
            result.reason = "parameter name not found";
        }
    }
    return result;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto parameter_demo_node = std::make_shared<ParameterDemoNode>("parameter_demo_node");
    rclcpp::spin(parameter_demo_node);
    rclcpp::shutdown();
}