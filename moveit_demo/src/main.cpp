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

    // start the competition
    floor_robot->start_competition_();
    // move the robot to home pose
    floor_robot->go_home_();
    // complete orders
    floor_robot->complete_orders_();
    // end the competition
    floor_robot->end_competition_();

    rclcpp::shutdown();
}