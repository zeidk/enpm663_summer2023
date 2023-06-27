#!/usr/bin/env python3

import rclpy
from interface_demo.interface_demo import AddTwoIntsClient


def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClient('client_demo_py')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
