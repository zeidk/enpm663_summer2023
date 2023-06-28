#pragma once

#include <rclcpp/rclcpp.hpp>
#include <enpm663_msgs/msg/weather_station.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <chrono>


// timer
class Weather : public rclcpp::Node
{
public:
    Weather(std::string node_name) : Node(node_name)
    {
        // initialize the message
        weather_msg_ = enpm663_msgs::msg::WeatherStation();
        // get the current day

        // Get the current ROS time
        rclcpp::Clock clock;
        auto current_time = clock.now();

        // Convert the ROS time to std::chrono::system_clock
        auto tp = std::chrono::system_clock::time_point(std::chrono::nanoseconds(current_time.nanoseconds()));

        // Convert the std::chrono::system_clock to std::tm
        std::time_t t = std::chrono::system_clock::to_time_t(tp);
        std::tm *time_info = std::localtime(&t);

        // Get the day of the week
        current_day_ = time_info->tm_wday;

        // timer callback
        timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(2000.0)),
                                         std::bind(&Weather::pub_timer_cb, this));

        // publisher
        weather_pub_ = this->create_publisher<enpm663_msgs::msg::WeatherStation>("weather", 10);
    }

private:
    // attributes
    enpm663_msgs::msg::WeatherStation weather_msg_;
    rclcpp::TimerBase::SharedPtr timer_;
    // get the current day
    int current_day_;
    rclcpp::Publisher<enpm663_msgs::msg::WeatherStation>::SharedPtr weather_pub_;

    // methods
    void pub_timer_cb();
};