'''
Minimal Python node.
'''

import rclpy
from rclpy.node import Node

####################
# Non OOP style
####################
def main(args=None):
    '''
    Main function for the node.

    Args:
        args (dict, optional): Key-value pair arguments. Defaults to None.
    '''
    rclpy.init(args=args)
    node = Node('minimal_python_node')
    node.get_logger().info('Help me Obi-Wan Kenobi, you are my only hope')
    rclpy.spin(node)
    rclpy.shutdown()

####################
# OOP style
####################


# class SimpleNode(Node):
#     '''
#     Minimal Python node using OOP style.

#     Args:
#         Node (rclpy.Node): Node class from rclpy package.
#     '''

#     def __init__(self, node_name):
#         super().__init__(node_name)
#         # self.get_logger().debug('Help me Obi-Wan Kenobi, you are my only hope')
#         self.get_logger().info('Help me Obi-Wan Kenobi, you are my only hope')
#         # self.get_logger().warn('Help me Obi-Wan Kenobi, you are my only hope')
#         # self.get_logger().error('Help me Obi-Wan Kenobi, you are my only hope')
#         # self.get_logger().fatal('Help me Obi-Wan Kenobi, you are my only hope')


# def main(args=None):
#     '''
#     Main function for the node.

#     Args:
#         args (dict, optional): Key-value pair arguments. Defaults to None.
#     '''
#     rclpy.init(args=args)
#     node = SimpleNode('minimal_python_node')
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()
