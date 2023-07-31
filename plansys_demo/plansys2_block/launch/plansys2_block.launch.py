import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory('plansys2_block')
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
          'model_file': example_dir + '/pddl/durative_block_domain.pddl',
          'namespace': namespace
          }.items())

    # Specify the actions
    pickup_cmd = Node(
        package='plansys2_block',
        executable='pickup_action_node',
        name='pickup',
        namespace=namespace,
        output='screen',
        parameters=[])

    putdown_cmd = Node(
        package='plansys2_block',
        executable='putdown_action_node',
        name='putdown',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    stack_cmd = Node(
        package='plansys2_block',
        executable='stack_action_node',
        name='stack',
        namespace=namespace,
        output='screen',
        parameters=[])

    unstack_cmd = Node(
        package='plansys2_block',
        executable='unstack_action_node',
        name='unstack',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    
    launch_description = LaunchDescription()

    launch_description.add_action(declare_namespace_cmd)

    # Declare the launch options
    launch_description.add_action(plansys2_cmd)
    launch_description.add_action(pickup_cmd)
    launch_description.add_action(putdown_cmd)
    launch_description.add_action(stack_cmd)
    launch_description.add_action(unstack_cmd)

    return launch_description
