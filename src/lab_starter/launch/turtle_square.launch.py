"""Starts turtlesim and the square-driving node together.

Run with:  ros2 launch lab_starter turtle_square.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='turtlesim', executable='turtlesim_node', name='sim'),
        Node(package='lab_starter', executable='turtle_square', name='turtle_square',
             output='screen'),
    ])
