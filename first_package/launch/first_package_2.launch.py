from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="first_package",
            executable="advanced_publisher.py",
        ),
        Node(
            package="first_package",
            executable="subscriber_exe",
        ),
    ])
