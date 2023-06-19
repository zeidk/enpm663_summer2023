import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class AdvancedPublisher(Node):
    '''
    _summary_

    Args:
        Node (_type_): _description_
    '''

    def __init__(self, node_name):
        super().__init__(node_name)
        self._publisher = self.create_publisher(String, 'leia', 10)
        # publishing every 2s
        self._timer = self.create_timer(1, self.my_timer_callback)
        self._msg = String()
        self._msg.data = 'Help me Obi-Wan Kenobi, you are my only hope.'

    def my_timer_callback(self):
        '''
        _summary_
        '''
        self._publisher.publish(self._msg)
        self.get_logger().info(f'Publishing: {self._msg.data}', )


def main(args=None):
    '''
    _summary_

    Args:
        args (_type_, optional): _description_. Defaults to None.
    '''    
    rclpy.init(args=args)
    node = AdvancedPublisher('advanced_publisher')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
