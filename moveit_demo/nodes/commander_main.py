#! /usr/bin/env python3

import rclpy
from moveit_demo.floor_robot_commander import FloorRobotCommander


def main(args=None):
    '''
    Main function to create a ROS2 node

    Args:
        args (Any, optional): ROS2 arguments. Defaults to None.
    '''
    rclpy.init(args=args)
    node = FloorRobotCommander('floor_robot_commander')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
