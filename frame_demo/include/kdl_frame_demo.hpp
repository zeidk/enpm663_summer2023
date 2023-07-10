#pragma once

#include <cmath>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <utils.hpp>
#include <geometry_msgs/msg/pose.hpp>

class KDLFrameDemo : public rclcpp::Node
{
public:
    KDLFrameDemo(std::string node_name) : Node(node_name)
    {
        // Do not execute the listener if the parameter 'listen' is true
        this->declare_parameter("kdl", false);
        param_kdl_ = this->get_parameter("kdl").as_bool();

        if (!param_kdl_)
        {
            RCLCPP_INFO(this->get_logger(), "KDL demo not started");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "KDL demo started");
    }


    /**
     * @brief Run the KDL demo
     * 
     */
    void run();

private:
    bool param_kdl_;

    /**
     * @brief Hardcode the pose of a camera in the world frame
     * 
     * @return geometry_msgs::msg::Pose Pose of the camera in the world frame
     */
    geometry_msgs::msg::Pose set_camera_pose_in_world();

    /**
     * @brief Hardcode the pose of a part in the camera frame
     * 
     * This pose should be retrieved from a camera subscriber
     * @return geometry_msgs::msg::Pose Pose of the part in the camera frame
     */
    geometry_msgs::msg::Pose set_part_pose_in_camera();
    /**
     * @brief Multiply two poses together and return the result
     *
     * @param p1
     * @param p2
     * @return geometry_msgs::msg::Pose
     */
    geometry_msgs::msg::Pose multiply_kdl_frames(geometry_msgs::msg::Pose pose1, geometry_msgs::msg::Pose pose2);
};
