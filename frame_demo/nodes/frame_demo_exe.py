#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor
from frame_demo.frame_interface import BroadcasterDemo, ListenerDemo, KDLFrameDemo


def main(args=None):
    rclpy.init(args=args)
    broadcaster_node = BroadcasterDemo('broadcaster_demo')
    listener_node = ListenerDemo('listener_demo')
    kdl_node = KDLFrameDemo('kdl_demo')
    # create a multi-threaded executor
    executor = MultiThreadedExecutor()
    executor.add_node(broadcaster_node)
    executor.add_node(listener_node)
    executor.add_node(kdl_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        broadcaster_node.get_logger().info('KeyboardInterrupt, shutting down.\n')
    broadcaster_node.destroy_node()
    listener_node.destroy_node()
    kdl_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
