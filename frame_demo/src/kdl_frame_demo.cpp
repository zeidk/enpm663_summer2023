#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <kdl_frame_demo.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <utils.hpp>

void KDLFrameDemo::run() {

    if (!param_kdl_) {
        return;
    }
    // create a unique pointer to the Utils class
    auto utils_ptr = std::make_unique<Utils>();
    geometry_msgs::msg::Pose camera_pose_in_world = set_camera_pose_in_world();
    geometry_msgs::msg::Pose part_pose_in_camera = set_part_pose_in_camera();
    geometry_msgs::msg::Pose part_pose_in_world = multiply_kdl_frames(camera_pose_in_world, part_pose_in_camera);

    // The quaternion from part_pose_in_world.orientation is of type geometry_msgs::msg::Quaternion
    // get_euler_from_quaternion requires a tf2::Quaternion
    // We need to convert part_pose_in_world.orientation to tf2::Quaternion
    // See http://wiki.ros.org/tf2/Tutorials/Quaternions
    tf2::Quaternion quat_tf;
    tf2::fromMsg(part_pose_in_world.orientation, quat_tf);
    auto rpy = utils_ptr->set_euler_from_quaternion(quat_tf);

    RCLCPP_INFO(this->get_logger(),
                "Part position in world frame:\n x: %f, y: %f, z: %f",
                part_pose_in_world.position.x, part_pose_in_world.position.y, part_pose_in_world.position.z);
    RCLCPP_INFO(this->get_logger(),
                "Part orientation in world frame:\n rpy: %f, %f, %f",
                rpy[0], rpy[1], rpy[2]);
}

geometry_msgs::msg::Pose
KDLFrameDemo::set_camera_pose_in_world()
{
    // create a unique pointer to the Utils class
    auto utils_ptr = std::make_unique<Utils>();

    auto utils = Utils();
    auto pose = geometry_msgs::msg::Pose();
    pose.position.x = -2.286;
    pose.position.y = 2.96;
    pose.position.z = 1.8;
    geometry_msgs::msg::Quaternion quaternion = utils_ptr->set_quaternion_from_euler(M_PI, M_PI / 2, 0);
    pose.orientation.w = quaternion.w;
    pose.orientation.x = quaternion.x;
    pose.orientation.y = quaternion.y;
    pose.orientation.z = quaternion.z;
    return pose;
}

geometry_msgs::msg::Pose KDLFrameDemo::set_part_pose_in_camera()
{
    auto pose = geometry_msgs::msg::Pose();
    pose.position.x = 1.0769784427063858;
    pose.position.y = 0.15500024548461805;
    pose.position.z = -0.5660066794253416;
    pose.orientation.w = 0.7058475442288268;
    pose.orientation.x = -0.0013258319361092675;
    pose.orientation.y = -0.7083612841685344;
    pose.orientation.z = -0.0013332542580505244;
    return pose;
}

geometry_msgs::msg::Pose KDLFrameDemo::multiply_kdl_frames(geometry_msgs::msg::Pose pose1, geometry_msgs::msg::Pose pose2)
{
    KDL::Frame frame1;
    KDL::Frame frame2;

    tf2::fromMsg(pose1, frame1);
    tf2::fromMsg(pose2, frame2);

    KDL::Frame frame3 = frame1 * frame2;

    return tf2::toMsg(frame3);
}