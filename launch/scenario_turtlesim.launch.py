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
            package='turtlesim_fuel_simulation',
            executable='scenario_manager',
            name='scenario_manager'
        ),
        Node(
            package='turtlesim_fuel_simulation',
            executable='scenario_listener',
            name='scenario_listener'
        )
    ])
