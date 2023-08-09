# pull in some Python launch modules.
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)


def launch_setup(context, *args, **kwargs):
    '''
    Returns a list of nodes to launch.

    Args:
        context (): The context object for the launch.

    Returns:
        List[Node]: A list of nodes to launch.
    '''

    # initialize arguments passed to the launch file
    reliability_arg = LaunchConfiguration('reliability').perform(context)
    durability_arg = LaunchConfiguration('durability').perform(context)

    publisher_node = Node(
        package="qos_demo",
        executable="qos_publisher_exe",
        parameters=[
            {"reliability": reliability_arg},
            {"durability": durability_arg},
        ]
    )

    subscriber_node = Node(
        package="qos_demo",
        executable="qos_subscriber_exe",
        parameters=[
            {"reliability": reliability_arg},
            {"durability": durability_arg},
        ]
    )

    # all the nodes to launch
    nodes_to_start = [publisher_node, subscriber_node]

    return nodes_to_start


def generate_launch_description():
    
    reliability_arg = DeclareLaunchArgument(
        "reliability",
        default_value='best_effort')
    
    durability_arg = DeclareLaunchArgument(
        "durability",
        default_value='volatile')

    declared_arguments = []
    declared_arguments.append(reliability_arg, durability_arg)

    launch_description = LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

    return launch_description
