#! /usr/bin/env python3


# @brief     Functionalities to send commands to the floor robot.
# @details   Commands are sent through service calls.
# @author    Zeid Kootbally
# @author    John Doe (add your teammate's name here)
# @version   0.1
# @date      July 2023
# @warning   Improper use can crash your application
# @copyright GNU Public License.


from rclpy.node import Node
from example_interfaces.msg import String
from rcl_interfaces.msg import SetParametersResult

## @brief Class to send commands to the floor robot through service calls.
class FloorRobotCommander(Node):

    ## @brief Class initializer.
    ## @param node_name (string): ROS2 node name
    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().info(f'{node_name} node running')
