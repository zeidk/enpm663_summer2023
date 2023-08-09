# pull in some Python launch modules.
from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():

    publisher_node = Node(
        package="simple_demo",
        executable="simple_publisher_exe"
    )

    subscriber_node = Node(
        package="simple_demo",
        executable="simple_subscriber_exe"
    )

    launch_description = LaunchDescription()
    launch_description.add_action(publisher_node)
    launch_description.add_action(subscriber_node)

    return launch_description
