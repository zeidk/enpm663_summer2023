#include <broadcaster_demo.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <utils.hpp>

void BroadcasterDemo::run()
{
    // Do not broadcast if the parameter 'broadcast' is false
    if (!param_broadcast_)
    {
        RCLCPP_INFO(this->get_logger(), "Broadcaster demo not started");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Broadcaster demo started");
    publish_static_transform();
}

void BroadcasterDemo::publish_static_transform()
{
    // geometry_msgs::msg::TransformStamped transform_stamped;

    /////////////////////////////////////////////////
    // First frame
    /////////////////////////////////////////////////
    static_transform_stamped_.header.stamp = this->get_clock()->now();
    static_transform_stamped_.header.frame_id = "world";
    static_transform_stamped_.child_frame_id = "first_static_frame";

    static_transform_stamped_.transform.translation.x = 3.5;
    static_transform_stamped_.transform.translation.y = 4.0;
    static_transform_stamped_.transform.translation.z = 5.0;

    geometry_msgs::msg::Quaternion quaternion = utils_ptr_->set_quaternion_from_euler(M_PI / 2, M_PI / 3, M_PI / 4);
    static_transform_stamped_.transform.rotation.x = quaternion.x;
    static_transform_stamped_.transform.rotation.y = quaternion.y;
    static_transform_stamped_.transform.rotation.z = quaternion.z;
    static_transform_stamped_.transform.rotation.w = quaternion.w;
    // Send the transform
    tf_static_broadcaster_->sendTransform(static_transform_stamped_);

    /////////////////////////////////////////////////
    // Second frame
    /////////////////////////////////////////////////
    static_transform_stamped_.header.stamp = this->get_clock()->now();
    static_transform_stamped_.header.frame_id = "world";
    static_transform_stamped_.child_frame_id = "second_static_frame";

    static_transform_stamped_.transform.translation.x = 1.5;
    static_transform_stamped_.transform.translation.y = 2.0;
    static_transform_stamped_.transform.translation.z = 3.0;

    quaternion = utils_ptr_->set_quaternion_from_euler(M_PI / 5, M_PI / 5, M_PI / 5);
    static_transform_stamped_.transform.rotation.x = quaternion.x;
    static_transform_stamped_.transform.rotation.y = quaternion.y;
    static_transform_stamped_.transform.rotation.z = quaternion.z;
    static_transform_stamped_.transform.rotation.w = quaternion.w;
    // Send the transform
    tf_static_broadcaster_->sendTransform(static_transform_stamped_);
}

void BroadcasterDemo::broadcast_timer_cb_()
{
    dynamic_transform_stamped_.header.stamp = this->get_clock()->now();
    dynamic_transform_stamped_.header.frame_id = "world";
    dynamic_transform_stamped_.child_frame_id = "dynamic_frame";

    dynamic_transform_stamped_.transform.translation.x = 5.0;
    dynamic_transform_stamped_.transform.translation.y = 0.4;
    dynamic_transform_stamped_.transform.translation.z = 0.3;

    geometry_msgs::msg::Quaternion quaternion = utils_ptr_->set_quaternion_from_euler(M_PI, M_PI / 2, M_PI / 3);
    dynamic_transform_stamped_.transform.rotation.x = quaternion.x;
    dynamic_transform_stamped_.transform.rotation.y = quaternion.y;
    dynamic_transform_stamped_.transform.rotation.z = quaternion.z;
    dynamic_transform_stamped_.transform.rotation.w = quaternion.w;
    // Send the transform
    tf_broadcaster_->sendTransform(dynamic_transform_stamped_);
}