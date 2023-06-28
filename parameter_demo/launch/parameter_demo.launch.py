# pull in some Python launch modules.
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# this function is needed
def generate_launch_description():
    
    ld = LaunchDescription()
    parameter_individual_demo = Node(
        package="parameter_demo",
        executable="parameter_demo_exe.py",
        parameters=[
            {'jedi': 'Luke Skywalker'},
        ],
    )
    
    # Loading parameters from a file
    node_params = os.path.join(
        get_package_share_directory('parameter_demo'),
        'config',
        'params.yaml'
    )
    parameter_file_demo = Node(
        package="parameter_demo",
        executable="parameter_demo_exe.py",
        parameters=[node_params],
    )

    ld.add_action(parameter_individual_demo)   
    return ld                               