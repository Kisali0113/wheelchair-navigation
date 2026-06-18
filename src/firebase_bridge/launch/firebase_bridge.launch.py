#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='firebase_bridge',
            executable='firebase_listener',
            name='firebase_listener',
            output='screen',
        ),
        Node(
            package='firebase_bridge',
            executable='firebase_publisher',
            name='firebase_publisher',
            output='screen',
        ),
        Node(
            package='firebase_bridge',
            executable='goal_bridge',
            name='goal_bridge',
            output='screen',
        ),
    ])
