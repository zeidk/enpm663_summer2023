#!/usr/bin/env python3

import rclpy
from interface_demo.interface_demo import Weather


def main(args=None):
    rclpy.init(args=args)
    node = Weather('weather_forecast')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
