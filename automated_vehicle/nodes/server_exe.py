#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor
from automated_vehicle.automated_vehicle_interface import ServerNode


def main(args=None):
    '''
    Main function showing a single-threaded executor.
    '''
    rclpy.init(args=args)
    server_node = ServerNode('server_node')
    rclpy.spin(server_node)
    server_node.destroy_node()
    rclpy.shutdown()


# def main(args=None):
#     '''
#     Main function showing a multi-threaded executor.
#     '''
#     rclpy.init(args=args)
#     server_node = ServerNode('server_node')
#     executor = MultiThreadedExecutor()
#     executor.add_node(server_node)
#     try:
#         server_node.get_logger().info('Beginning demo, end with CTRL-C')
#         executor.spin()
#     except KeyboardInterrupt:
#         server_node.get_logger().info('KeyboardInterrupt, shutting down.\n')
#     server_node.destroy_node()
#     rclpy.shutdown()


if __name__ == '__main__':
    main()
