#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor
from interface_demo.interface_demo import AddTwoIntsClient

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClient('client_demo_py')
    # create a multi-threaded executor
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt, shutting down.\n')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
