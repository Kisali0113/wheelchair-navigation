#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arduino_port = LaunchConfiguration('arduino_port')
    room3_x = LaunchConfiguration('room3_x')
    room3_y = LaunchConfiguration('room3_y')
    room3_tol = LaunchConfiguration('room3_tol')

    return LaunchDescription([
        DeclareLaunchArgument(
            'arduino_port',
            default_value='/dev/ttyACM0',
            description='Serial port connected to the Arduino',
        ),
        DeclareLaunchArgument(
            'room3_x',
            default_value='0.0',
            description='Room 3 X coordinate used by the camera trigger',
        ),
        DeclareLaunchArgument(
            'room3_y',
            default_value='0.0',
            description='Room 3 Y coordinate used by the camera trigger',
        ),
        DeclareLaunchArgument(
            'room3_tol',
            default_value='0.5',
            description='Room 3 matching tolerance',
        ),
        Node(
            package='firebase_bridge',
            executable='firebase_listener',
            name='firebase_listener',
            output='screen',
        ),
        # Node(
        #     package='firebase_bridge',
        #     executable='firebase_publisher',
        #     name='firebase_publisher',
        #     output='screen',
        # ),
        Node(
            package='firebase_bridge',
            executable='goal_bridge',
            name='goal_bridge',
            output='screen',
        ),
        # Node(
        #     package='wheelchair_mapping_pkg',
        #     executable='camera_node',
        #     name='camera_trigger',
        #     output='screen',
        #     parameters=[{
        #         'arduino_port': arduino_port,
        #         'room3_x': room3_x,
        #         'room3_y': room3_y,
        #         'room3_tol': room3_tol,
        #         'servo_angle': 180,
        #     }],
        # ),
    ])
