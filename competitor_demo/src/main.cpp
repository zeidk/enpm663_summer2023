#include <rclcpp/rclcpp.hpp>
#include <listener_demo.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto listener_demo_node = std::make_shared<ListenerDemo>("listener_demo");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(listener_demo_node);
    executor.spin();

    rclcpp::shutdown();
}