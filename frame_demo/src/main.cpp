#include <rclcpp/rclcpp.hpp>
#include <kdl_frame_demo.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto kdl_frame_demo_node = std::make_shared<KDLFrameDemo>("kdl_frame_demo");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(kdl_frame_demo_node);
    std::thread([&executor]()
                { executor.spin(); })
        .detach();
    kdl_frame_demo_node->run();
    rclcpp::shutdown();
}