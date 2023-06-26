from datetime import datetime
from rclpy.node import Node
from builtin_interfaces.msg import Time
from enpm663_msgs.msg import WeatherStation


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
        
        self.get_logger().info(f'Publishing: {mag}')

        self._forecast_pub.publish(msg)
