#! /usr/bin/env python3


## @brief     Functionalities to send commands to the floor robot.
## @details   Commands are sent through service calls.
## @author    Zeid Kootbally
## @author    John Doe (add your teammate's name here)
## @version   0.1
## @date      July 2023
## @warning   Improper use can crash your application
## @copyright GNU Public License.


from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

## @brief Class to send commands to the floor robot through service calls.


class FloorRobotCommander(Node):

    ## @brief Class initializer.
    ## @param node_name (string): ROS2 node name
    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().info(f'{node_name} node initialized')

        self._publisher = self.create_publisher(String, '/moveit_demo/floor_robot/go_home', 10)

        self._floor_robot_client = self.create_client(String, '/moveit_demo/floor_robot/go_home')
        while not self._floor_robot_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self._start_competition_client = self.create_client(String, '/ariac/start_competition')
        while not self._start_competition_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        # start the competition
        request = Trigger.Request()
        future = self._start_competition_client.call_async(request)
        future.add_done_callback(self.future_callback)
        
        # demo with service
        
        
        # # send request to go home
        # request = Trigger.Request()
        # future = self._floor_robot_client.call_async(request)
        # # Add a callback function to be called when the future is complete
        # future.add_done_callback(self.future_callback)

        # demo with publisher
        message = String()
        message.data = 'go_home'
        self._publisher.publish(message)

    ## @brief Callback for message received by the server.
    def future_callback(self, future):
        '''
        Callback function for the future object

        Args:
            future (Future): A future object
        '''
        self.get_logger().info(f'Result: {future.result().success}')
