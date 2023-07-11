#pragma once

#include <cmath>
#include <tf2_ros/static_transform_broadcaster.h>
#include <utils.hpp>
#include <geometry_msgs/msg/pose.hpp>
// for static broadcaster
#include "tf2_ros/static_transform_broadcaster.h"
// for dynamic broadcaster
#include "tf2_ros/transform_broadcaster.h"

class BroadcasterDemo : public rclcpp::Node
{
public:
    BroadcasterDemo(std::string node_name) : Node(node_name)
    {
        // declare a parameter to start the broadcaster
        this->declare_parameter("broadcast", true);
        param_broadcast_ = this->get_parameter("broadcast").as_bool();

        // initialize a static transform broadcaster
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // initialize the transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Create a utils object to use the utility functions
        utils_ptr_ = std::make_shared<Utils>();

            // timer to publish the transform
        broadcast_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),
                                                   std::bind(&BroadcasterDemo::broadcast_timer_cb_, this));
    }

    /**
     * @brief Run the broadcaster demo
     *
     */
    void run();

private:
    /*!< Boolean parameter to whether or not start the broadcaster */
    bool param_broadcast_;
    /*!< Static broadcaster object */
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    /*!< Broadcaster object */
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    /*!< Utils object */
    std::shared_ptr<Utils> utils_ptr_;
    /*!< Wall timer object */
    rclcpp::TimerBase::SharedPtr broadcast_timer_;
    /*!< Transform stamped for the dynamic broadcaster */
    geometry_msgs::msg::TransformStamped dynamic_transform_stamped_;
    /*!< Transform stamped for the static broadcaster */
    geometry_msgs::msg::TransformStamped static_transform_stamped_;

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
};
