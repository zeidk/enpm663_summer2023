#include <rclcpp/rclcpp.hpp>
#include "message_demo.hpp"

void Weather::pub_timer_cb()
{
    weather_msg_.weather = enpm663_msgs::msg::WeatherStation::CLOUDY;
    weather_msg_.day = current_day_;
    builtin_interfaces::msg::Time time_msg;

    // Get the current ROS time
    auto clock = std::make_shared<rclcpp::Clock>();
    auto current_time = clock->now();
    // Extract seconds and nanoseconds from ROS time
    uint32_t sec = current_time.seconds();
    uint32_t nanosec = current_time.nanoseconds();
    time_msg.sec = sec;
    time_msg.nanosec = nanosec;
    weather_msg_.time = time_msg;
    // RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: " << weather_msg_.weather << " on day " << weather_msg_.day);
    weather_pub_->publish(weather_msg_);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto weather_node = std::make_shared<Weather>("weather_forecast");
    rclcpp::spin(weather_node);
    rclcpp::shutdown();
}