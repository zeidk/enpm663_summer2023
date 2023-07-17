#include "floor_robot.hpp"

// ================================
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto floor_robot = std::make_shared<FloorRobot>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(floor_robot);
    std::thread([&executor]()
                { executor.spin(); })
        .detach();
    // floor_robot->go_home_();
    floor_robot->complete_orders_();

    rclcpp::shutdown();
}