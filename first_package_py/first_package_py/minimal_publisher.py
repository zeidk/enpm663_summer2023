import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self._publisher = self.create_publisher(String, 'leia', 10)
        self._msg = String()
        self._msg.data = 'Help me Obi-Wan Kenobi, you are my only hope.'
        self._publisher.publish(self._msg)
        self.get_logger().info(f'Publishing: {self._msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher('minimal_publisher')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
