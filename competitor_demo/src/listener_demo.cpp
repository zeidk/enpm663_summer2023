#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <listener_demo.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"

// needed for the listener
#include <tf2/exceptions.h>
// allows to use, 50ms, etc
using namespace std::chrono_literals;

geometry_msgs::msg::Pose ListenerDemo::listen_transform(const std::string &source_frame, const std::string &target_frame)
{
    geometry_msgs::msg::TransformStamped t_stamped;
    geometry_msgs::msg::Pose pose_out;
    try
    {
        t_stamped = tf_buffer_->lookupTransform(source_frame, target_frame, tf2::TimePointZero, 50ms);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Could not get transform between " << source_frame << " and " << target_frame << ": " << ex.what());
    }

    pose_out.position.x = t_stamped.transform.translation.x;
    pose_out.position.y = t_stamped.transform.translation.y;
    pose_out.position.z = t_stamped.transform.translation.z;
    pose_out.orientation = t_stamped.transform.rotation;

    // RCLCPP_INFO_STREAM(this->get_logger(),
    //                    target_frame << " in " << source_frame << ":\n"
    //                                 << "x: " << pose_out.position.x << "\t"
    //                                 << "y: " << pose_out.position.y << "\t"
    //                                 << "z: " << pose_out.position.z << "\n"
    //                                 << "qx: " << pose_out.orientation.x << "\t"
    //                                 << "qy: " << pose_out.orientation.y << "\t"
    //                                 << "qz: " << pose_out.orientation.z << "\t"
    //                                 << "qw: " << pose_out.orientation.w << "\n");
    return pose_out;
}

void ListenerDemo::listen_timer_cb_()
{
    floor_robot_msgs::msg::FloorRobotPoses floor_robot_poses;
   
    auto end_effector_pose = listen_transform("world", "floor_gripper");
    auto floor_robot_base_pose = listen_transform("world", "robot_base");
    floor_robot_poses.base_pose = floor_robot_base_pose;
    floor_robot_poses.end_effector_pose = end_effector_pose;
    robot_poses_pub_->publish(floor_robot_poses);
}