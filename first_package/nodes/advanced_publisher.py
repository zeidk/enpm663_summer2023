#!/usr/bin/env python3

import rclpy
from first_package.first_package_interface import AdvancedPublisher


def main(args=None):
    rclpy.init(args=args)
    node = AdvancedPublisher('advanced_publisher')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
