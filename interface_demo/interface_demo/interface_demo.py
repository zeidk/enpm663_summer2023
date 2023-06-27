from datetime import datetime
import random
from rclpy.node import Node
import rclpy
from builtin_interfaces.msg import Time
from enpm663_msgs.msg import WeatherStation
from enpm663_msgs.srv import AddTwoInts


class Weather(Node):
    '''
    Class to publish weather forecast
    '''

    def __init__(self, node_name):
        super().__init__(node_name)
        self._forecast_pub = self.create_publisher(WeatherStation, 'weather_forecast', 100)
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
    def __init__(self, node_name):
        super().__init__(node_name)
        self._client = self.create_client(AddTwoInts, 'add_two_ints')
        while not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self._request = AddTwoInts.Request()
        self._client_timer = self.create_timer(2, self._client_timer_cb)

    def _client_timer_cb(self):
        a = random.randint(0, 9)
        b = random.randint(0, 9)
        self.send_request(a, b)

    def send_request(self, a, b):
        self._request.a = a
        self._request.b = b
        future = self._client.call_async(self._request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
