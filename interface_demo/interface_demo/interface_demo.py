from datetime import datetime
import random
from rclpy.node import Node
from builtin_interfaces.msg import Time
from enpm663_msgs.msg import WeatherStation
from enpm663_msgs.srv import AddTwoInts
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class Weather(Node):
    '''
    Class to publish weather forecast
    '''

    def __init__(self, node_name):
        super().__init__(node_name)
        self._forecast_pub = self.create_publisher(WeatherStation, 'weather', 100)
        self._pub_timer = self.create_timer(2, self._pub_timer_cb)
        self._current_day = datetime.now().weekday()

    def _pub_timer_cb(self):
        msg = WeatherStation()
        msg.weather = WeatherStation.CLOUDY
        msg.day = self._current_day
        time_msg = Time()
        time_msg.sec = self.get_clock().now().seconds_nanoseconds()[0]
        time_msg.nanosec = self.get_clock().now().seconds_nanoseconds()[1]
        msg.time = time_msg

        self.get_logger().info(f'Publishing: {msg}')

        self._forecast_pub.publish(msg)


class AddTwoIntsClient(Node):
    '''
    Class to call the add_two_ints service
    '''

    def __init__(self, node_name):
        super().__init__(node_name)
        # Create various callback groups to ensure that various events are called in a mutually exclusive manner
        client_cb_group = MutuallyExclusiveCallbackGroup()
        timer_cb_group = MutuallyExclusiveCallbackGroup()

        self._client = self.create_client(AddTwoInts, 'add_two_ints', callback_group=client_cb_group)

        # Wait for the service to be available
        while not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self._logger.info("client created")
        self._request = AddTwoInts.Request()
        self._client_timer = self.create_timer(2, self._client_timer_cb, callback_group=timer_cb_group)

    def _client_timer_cb(self):
        '''
        Callback function for the client timer
        The body of this function is executed every 2 seconds
        '''
        a = random.randint(0, 9)
        b = random.randint(0, 9)
        self.send_request(a, b)

    def send_request(self, a, b):
        '''
        Send a request to the add_two_ints service

        Args:
            a (int): First integer
            b (int): Second integer
        '''
        while not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
            
        self._request.a = a
        self._request.b = b
        response = self._client.call(self._request)
        self.get_logger().info(f'Response: {response.sum}')
