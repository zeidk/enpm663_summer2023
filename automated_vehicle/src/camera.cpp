#include <rclcpp/rclcpp.hpp>
#include "camera.hpp"

void GenericCameraNode::timer_callback()
{
    msg_.header.stamp = this->get_clock()->now();
    msg_.header.frame_id = camera_name_;
    msg_.height = 480;
    msg_.width = 640;
    msg_.encoding = "rgb8";
    msg_.is_bigendian = false;
    msg_.step = 640 * 3;
    msg_.data = std::vector<uint8_t>(msg_.height * msg_.step, 0);                                                                                                                                                             
    publisher_->publish(msg_);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto camera_node = std::make_shared<GenericCameraNode>("camera_node");
    rclcpp::spin(camera_node);
    rclcpp::shutdown();
}


