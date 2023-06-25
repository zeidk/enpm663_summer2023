#!/usr/bin/env python3

import rclpy
from automated_vehicle.automated_vehicle_interface import GenericCameraNode


def main(args=None):
    rclpy.init(args=args)
    camera_node = GenericCameraNode('camera_node')
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
