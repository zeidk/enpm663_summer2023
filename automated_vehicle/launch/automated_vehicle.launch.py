# pull in some Python launch modules.
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# this function is needed


def generate_launch_description():

    ld = LaunchDescription()

    # Loading parameters from a file
    node_params = os.path.join(
        get_package_share_directory('automated_vehicle'),
        'config',
        'params.yaml'
    )

    camera1_python = Node(
        package="automated_vehicle",
        executable="camera_exe.py",
        parameters=[node_params],   # parameter file
        name='camera1',             # node remapping
        remappings=[                # topic remapping
            ('/camera', '/left')
        ]
    )

    camera2_python = Node(
        package="automated_vehicle",
        executable="camera_exe.py",
        parameters=[node_params],
        name='camera2',  # node remapping
        remappings=[    # topic remapping
            ('/camera', '/front')
        ]
    )

    camera3_python = Node(
        package="automated_vehicle",
        executable="camera_exe.py",
        parameters=[node_params],
        name='camera3',  # node remapping
        remappings=[    # topic remapping
            ('/camera', '/right')
        ]
    )

    camera4_python = Node(
        package="automated_vehicle",
        executable="camera_exe.py",
        parameters=[node_params],
        name='camera4',  # node remapping
        remappings=[    # topic remapping
            ('/camera', '/rear')
        ]
    )

    camera1_cpp = Node(
        package="automated_vehicle",
        executable="camera_exe",
        parameters=[node_params],   # parameter file
        name='camera1',             # node remapping
        remappings=[                # topic remapping
            ('/camera', '/left')
        ]
    )

    camera2_cpp = Node(
        package="automated_vehicle",
        executable="camera_exe",
        parameters=[node_params],
        name='camera2',  # node remapping
        remappings=[    # topic remapping
            ('/camera', '/front')
        ]
    )

    camera3_cpp = Node(
        package="automated_vehicle",
        executable="camera_exe",
        parameters=[node_params],
        name='camera3',  # node remapping
        remappings=[    # topic remapping
            ('/camera', '/right')
        ]
    )

    camera4_cpp = Node(
        package="automated_vehicle",
        executable="camera_exe",
        parameters=[node_params],
        name='camera4',  # node remapping
        remappings=[    # topic remapping
            ('/camera', '/rear')
        ]
    )

    ld.add_action(camera1_python)
    ld.add_action(camera2_python)
    ld.add_action(camera3_python)
    ld.add_action(camera4_python)
    return ld
