from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),
        Node(
            package='custom_package',
            executable='fuel_manager',
            name='fuel_manager'
        ),
        Node(
            package='custom_package',
            executable='fuel_visualizer',
            name='fuel_visualizer'
        )
    ])

