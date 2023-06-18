from rclpy.node import Node
from std_msgs.msg import String


class AdvancedPublisher(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self._publisher = self.create_publisher(String, 'leia', 10)
        # publishing every 2s
        self.get_logger().info('Advanced Publisher Node initialized')
        self._msg = String()
        self._timer = self.create_timer(2, self.timer_callback)
        

    def timer_callback(self):
        self._msg.data = 'Help me Obi-Wan Kenobi, you are my only hope.'
        self._publisher.publish(self._msg)
        self.get_logger().info(f'Publishing: {self._msg.data}')


class SubscriberNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self._subscriber = self.create_subscription(String, 'leia', self.subscriber_callback, 10)
        self.get_logger().info('Subscriber Node initialized')
    def subscriber_callback(self, msg):
        self.get_logger().info(f'Receiving: {msg.data}')
