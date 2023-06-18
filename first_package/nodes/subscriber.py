#!/usr/bin/env python3

import rclpy
from first_package.first_package_interface import SubscriberNode


def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode('advanced_subscriber')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
