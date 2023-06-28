from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class ServerNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        # Callback groups
        subscription_group1 = MutuallyExclusiveCallbackGroup()
        subscription_group2 = MutuallyExclusiveCallbackGroup()
        
        # Subscribers
        # self._left_sub = self.create_subscription(Image, 'left', self._left_sub_callback, 10)
        # self._right_sub = self.create_subscription(Image, 'right', self._right_sub_callback, 10)
        # self._front_sub = self.create_subscription(Image, 'front', self._front_sub_callback, 10)
        # self._rear_sub = self.create_subscription(Image, 'rear', self._rear_sub_callback, 10)
        
        self._left_sub = self.create_subscription(
            Image, 'left', self._left_sub_callback, 10)
        self._right_sub = self.create_subscription(
            Image, 'right', self._right_sub_callback, 10)
        self._front_sub = self.create_subscription(
            Image, 'front', self._front_sub_callback, 10)
        self._rear_sub = self.create_subscription(
            Image, 'rear', self._rear_sub_callback, 10)

    def _left_sub_callback(self, msg):
        '''
        Callback function to the /left topic.

        Args:
            msg (Image): Image message from the left camera.
        '''
        
        self.get_logger().info(f'Receiving data from left camera: {msg.header.frame_id}')
        # while True:
        #     pass

    def _right_sub_callback(self, msg):
        '''
        Callback function to the /right topic.

        Args:
            msg (Image): Image message from the right camera.
        '''
        self.get_logger().info(f'Receiving data from right camera: {msg.header.frame_id}')

    def _front_sub_callback(self, msg):
        '''
        Callback function to the /front topic.

        Args:
            msg (Image): Image message from the left camera.
        '''
        self.get_logger().info(f'Receiving data from front camera: {msg.header.frame_id}')
        
    def _rear_sub_callback(self, msg):
        '''
        Callback function to the /rear topic.

        Args:
            msg (Image): Image message from the rear camera.
        '''
        self.get_logger().info(f'Receiving data from rear camera: {msg.header.frame_id}')
        


class GenericCameraNode(Node):
    '''
    Class for a camera node

    Args:
        Node (Node): Class for creating a ROS node
    '''

    def __init__(self, node_name):
        super().__init__(node_name)
        # Publisher
        self._publisher = self.create_publisher(Image, 'camera', 100)
        # Parameters
        self.declare_parameter('frequency', 1.0)
        self.declare_parameter('bandwidth', 2.5)
        self.declare_parameter('name', 'camera')

        self._camera_frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self._camera_bandwidth = self.get_parameter('bandwidth').get_parameter_value().double_value
        self._camera_name = self.get_parameter('name').get_parameter_value().string_value
        
        self.get_logger().info(f'{node_name}: [frequency: {self._camera_frequency}]')

        self.get_logger().info(f'{node_name} initialized')
        self._timer = self.create_timer(int(1/self._camera_frequency), self.timer_callback)

    def timer_callback(self):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._camera_name
        msg.height = 480
        msg.width = 640
        msg.encoding = 'rgb8'
        msg.is_bigendian = False
        msg.step = 640 * 3
        msg.data = [0] * msg.height * msg.step
        self._publisher.publish(msg)
