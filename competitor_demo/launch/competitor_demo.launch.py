
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node


from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)


def launch_setup(context, *args, **kwargs):
    # Launch arguments
    trial_name = LaunchConfiguration("trial_name")
    use_rviz = LaunchConfiguration("use_rviz")

    # Moveit
    moveit_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ariac_moveit_config"), "/launch", "/ariac_robots_moveit.launch.py"]
        )
    )

    # ARIAC_environment
    ariac_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ariac_gazebo"), "/launch", "/ariac.launch.py"]
        ),
        launch_arguments={
            'trial_name': trial_name,
            'competitor_pkg': "test_competitor",
            'sensor_config': "sensors"
        }.items()
    )

    # Test competitor
    test_competitor_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("test_competitor"), "/launch", "/competitor.launch.py"]
        ),
        launch_arguments={
            'rviz': use_rviz
        }.items()
    )
    
    # Listener
    listener_cmd = Node(
        package="competitor_demo",
        executable="listener_demo_exe",
        output="screen"
    )


    nodes_to_start = [
        ariac_cmd,
        moveit_cmd,
        test_competitor_cmd,
        listener_cmd
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument("trial_name", default_value="kitting", description="Name of ariac trial")
    )

    declared_arguments.append(
        DeclareLaunchArgument("use_rviz", default_value="false", description="start rviz node?")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
