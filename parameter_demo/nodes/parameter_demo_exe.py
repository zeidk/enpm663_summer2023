#!/usr/bin/env python3

import rclpy
from parameter_demo.parameter_demo_interface import ParameterDemoNode


def main(args=None):
    rclpy.init(args=args)
    node = ParameterDemoNode('parameter_demo_node')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
