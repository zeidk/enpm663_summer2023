#pragma once

#include <cmath>
#include <tf2_ros/static_transform_broadcaster.h>
#include <utils.hpp>
#include <geometry_msgs/msg/pose.hpp>
// for static broadcaster
#include "tf2_ros/static_transform_broadcaster.h"
// for dynamic broadcaster
#include "tf2_ros/transform_broadcaster.h"
using namespace std::chrono_literals;

class BroadcasterDemo : public rclcpp::Node
{
public:
    BroadcasterDemo(std::string node_name) : Node(node_name)
    {
        // parameter to decide whether to execute the demo or not
        this->declare_parameter("broadcast", true);
        param_broadcast_ = this->get_parameter("broadcast").as_bool();

        // do not execute the demo if the parameter is false
        if (!param_broadcast_)
        {
            RCLCPP_INFO(this->get_logger(), "Broadcaster demo not started");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Broadcaster demo started");

        // initialize a static transform broadcaster
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // initialize the transform broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Create a utils object to use the utility functions
        utils_ptr_ = std::make_shared<Utils>();

        // timer to publish the transform
        broadcast_timer_ = this->create_wall_timer(
            100ms,
            std::bind(&BroadcasterDemo::broadcast_timer_cb_, this));

        // timer to publish the transform
        static_broadcast_timer_ = this->create_wall_timer(
            10s,
            std::bind(&BroadcasterDemo::static_broadcast_timer_cb_, this));

    }


private:
    /*!< Boolean parameter to whether or not start the broadcaster */
    bool param_broadcast_;
    /*!< Static broadcaster object */
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    /*!< Broadcaster object */
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    /*!< Utils object */
    std::shared_ptr<Utils> utils_ptr_;
    /*!< Wall timer object for the broadcaster*/
    rclcpp::TimerBase::SharedPtr broadcast_timer_;
    /*!< Wall timer object for the static broadcaster*/
    rclcpp::TimerBase::SharedPtr static_broadcast_timer_;

    /**
     * @brief Build and publish a frame
     *
     */
    void publish_static_transform();

    /**
     * @brief Timer to broadcast the transform
     *
     */
    void broadcast_timer_cb_();
    /**
     * @brief Timer to broadcast the transform
     *
     */
    void static_broadcast_timer_cb_();
};
