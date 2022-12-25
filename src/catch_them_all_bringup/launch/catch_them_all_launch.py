#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node'
    )

    turtle_spawner_node = Node(
        package='turtle_spawner',
        executable='turtle_spawner',
        parameters=[
            {'publish_frequency': 2.0},
            {'max_spawn_turtles': 7}
        ]
    )

    turtle_controller_node = Node(
        package='turtle_controller',
        executable='turtle_controller',
        parameters=[
            {'control_turtle_name': 'turtle1'},
            {'control_frequency': 100.0},
        ]
    )

    ld.add_action(turtlesim_node)
    ld.add_action(turtle_spawner_node)
    ld.add_action(turtle_controller_node)

    return ld
