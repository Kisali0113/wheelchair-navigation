#!/usr/bin/env python3
"""
Launch file for chair control system with all nodes.
Launches: gate_controller, speaker_node, odom_localizer
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for chair control system."""

    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # Gate Controller Node
    gate_controller_node = Node(
        package='my_robot_pkg',
        executable='gate_controller',
        name='gate_controller',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('chair_status', '/chair/status'),
            ('speaker_control', '/speaker/control'),
            ('maincontrolling', '/robot/maincontrolling'),
        ]
    )

    # Speaker Node
    speaker_node = Node(
        package='my_robot_pkg',
        executable='speaker_node',
        name='speaker_node',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('speaker_control', '/speaker/control'),
        ]
    )

    # Odometry Localizer Node
    odom_localizer_node = Node(
        package='my_robot_pkg',
        executable='odom_localizer',
        name='odom_localizer',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'serial_port': '/dev/ttyUSB0'},
            {'serial_baudrate': 9600},
        ],
        remappings=[
            ('odom', '/odometry/filtered'),
        ]
    )

    return LaunchDescription([
        use_sim_time,
        gate_controller_node,
        speaker_node,
        odom_localizer_node,
    ])
