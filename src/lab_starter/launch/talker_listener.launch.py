"""Starts the talker and listener together.

Run with:  ros2 launch lab_starter talker_listener.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='lab_starter', executable='talker', output='screen'),
        Node(package='lab_starter', executable='listener', output='screen'),
    ])
