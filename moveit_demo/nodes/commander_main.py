#! /usr/bin/env python3

import rclpy
from moveit_demo.floor_robot_commander import RobotCommanderInterface
from rclpy.executors import MultiThreadedExecutor

def main(args=None):
    '''
    Main function to create a ROS2 node

    Args:
        args (Any, optional): ROS2 arguments. Defaults to None.
    '''
    rclpy.init(args=args)
    node = RobotCommanderInterface()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except rclpy.executors.ExternalShutdownException:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
