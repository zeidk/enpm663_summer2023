from rclpy.node import Node
from std_msgs.msg import String
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult


class ParameterDemoNode(Node):
    '''
    Class showcasing parameter usage

    Args:
        Node (Node): Class for creating a ROS node
    '''

    def __init__(self, node_name):
        super().__init__(node_name)
        # Publisher
        self._publisher = self.create_publisher(String, 'leia', 10)
        # Parameters
        self.declare_parameter('jedi', 'Obi-Wan Kenobi')
        self._jedi = self.get_parameter('jedi').get_parameter_value().string_value

        # publishing every 2s
        self.get_logger().info(f'{node_name} Node initialized')
        self._msg = String()
        self._timer = self.create_timer(1, self.timer_callback)

        # Callback for parameter changes
        self.add_on_set_parameters_callback(self.parameters_callback)

    def parameters_callback(self, params):
        '''
        Callback for parameter changes

        Args:
            params (list): List of parameters

        Returns:
            SetParametersResult: Object with success flag
        '''
        success = False
        for param in params:
            if param.name == "jedi":
                if param.type_ == Parameter.Type.STRING:
                    success = True
                    self._jedi = param.value
        return SetParametersResult(successful=success)

    def timer_callback(self):
        self._msg.data = f'Help me {self._jedi}, you are my only hope.'
        self._publisher.publish(self._msg)
        self.get_logger().info(f'{self._msg.data}')
