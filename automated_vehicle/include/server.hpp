#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

// timer
class ServerNode : public rclcpp::Node
{
public:
    ServerNode(std::string node_name) : Node(node_name)
    {
        /* These define the callback groups
         * They don't really do much on their own, but they have to exist in order to
         * assign callbacks to them. They're also what the executor looks for when trying to run multiple threads
         */
        callback_group_subscriber1_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_group_subscriber2_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_group_subscriber3_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_group_subscriber4_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        callback_group_subscriber1_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_group_subscriber2_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_group_subscriber3_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_group_subscriber4_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        // Each of these callback groups is basically a thread
        // Everything assigned to one of them gets bundled into the same thread
        auto sub1_opt = rclcpp::SubscriptionOptions();
        sub1_opt.callback_group = callback_group_subscriber1_;
        auto sub2_opt = rclcpp::SubscriptionOptions();
        sub2_opt.callback_group = callback_group_subscriber2_;
        auto sub3_opt = rclcpp::SubscriptionOptions();
        sub3_opt.callback_group = callback_group_subscriber3_;
        auto sub4_opt = rclcpp::SubscriptionOptions();
        sub4_opt.callback_group = callback_group_subscriber4_;

        left_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "left", rclcpp::QoS(10), 
            std::bind(&ServerNode::left_callback, this, std::placeholders::_1),
            sub1_opt);
        right_sub_ = this->create_subscription<sensor_msgs::msg::Image>("right", rclcpp::QoS(10),
                                                                        std::bind(&ServerNode::left_callback, this, std::placeholders::_1),
                                                                        sub2_opt);
        front_sub_ = this->create_subscription<sensor_msgs::msg::Image>("front", rclcpp::QoS(10),
                                                                        std::bind(&ServerNode::left_callback, this, std::placeholders::_1),
                                                                        sub3_opt);
        rear_sub_ = this->create_subscription<sensor_msgs::msg::Image>("rear", rclcpp::QoS(10),
                                                                       std::bind(&ServerNode::left_callback, this, std::placeholders::_1),
                                                                       sub4_opt);
    }

private:
    // attributes
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr front_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rear_sub_;
    // callback groups
    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber1_;
    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber2_;
    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber3_;
    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber4_;
    // methods
    void left_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void right_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void front_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void rear_callback(const sensor_msgs::msg::Image::SharedPtr msg);
};