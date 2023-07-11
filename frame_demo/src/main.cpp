#include <rclcpp/rclcpp.hpp>
#include <kdl_frame_demo.hpp>
#include <broadcaster_demo.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto kdl_frame_demo_node = std::make_shared<KDLFrameDemo>("kdl_frame_demo");
    auto broadcaster_demo_node = std::make_shared<BroadcasterDemo>("broadcaster_demo");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(kdl_frame_demo_node);
    executor.add_node(broadcaster_demo_node);
    executor.spin();
    // std::thread([&executor]()
    //             { executor.spin(); })
    //     .detach();
    // kdl_frame_demo_node->run();
    // broadcaster_demo_node->run();
    rclcpp::shutdown();
}