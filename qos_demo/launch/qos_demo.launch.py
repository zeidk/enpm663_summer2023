# pull in some Python launch modules.
from launch import LaunchDescription
from launch_ros.actions import Node

# this function is needed


def generate_launch_description():
    # instantiate a Launchdescription object
    publisher_node = Node(                  
        package="qos_demo",            
        executable="qos_publisher_exe"
    )
    
    subscriber_node = Node(
        package="qos_demo",
        executable="qos_subscriber_exe"
    )


    ld = LaunchDescription()
    ld.add_action(publisher_node)   
    ld.add_action(subscriber_node)
    return ld                       
