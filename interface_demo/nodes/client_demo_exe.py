#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor
from interface_demo.interface_demo import ServiceClientCallFromTimer, ServiceClientCallFromSubscriber

def main(args=None):
    rclpy.init(args=args)
    timer_node = ServiceClientCallFromTimer('client_timer_py')
    sub_node = ServiceClientCallFromSubscriber('client_sub_py')
    # create a multi-threaded executor
    executor = MultiThreadedExecutor()
    executor.add_node(timer_node)
    executor.add_node(sub_node)
    # Shreejay: You cam add more nodes to the executor here
    # e.g., executor.add_node(node2)

    try:
        executor.spin()
    except KeyboardInterrupt:
        timer_node.get_logger().info('KeyboardInterrupt, shutting down.\n')
    timer_node.destroy_node()
    sub_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
