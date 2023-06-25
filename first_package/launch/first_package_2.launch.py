from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    node1 = Node(package="first_package",
                 executable="advanced_publisher.py",
                 )
    node2 = Node(
        package="first_package",
        executable="subscriber_exe",
    )
    return LaunchDescription([node1, node2])
