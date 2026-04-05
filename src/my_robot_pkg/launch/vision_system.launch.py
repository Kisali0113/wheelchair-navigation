#!/usr/bin/env python3
"""
Launch file for vision-based person detection and tracking.
Launches: human_position_node, person_tracker_node
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for vision system."""

    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    enable_visualization = DeclareLaunchArgument(
        'enable_visualization',
        default_value='true',
        description='Enable OpenCV visualization windows'
    )

    # Human Position Detection Node
    human_position_node = Node(
        package='my_robot_pkg',
        executable='human_position_node',
        name='human_position_detector',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'model_path': 'yolov8n.pt'},
            {'enable_visualization': LaunchConfiguration('enable_visualization')},
        ],
        remappings=[
            ('robotcommand', '/robot/command'),
            ('person_position', '/perception/person_position'),
        ]
    )

    # Person Tracker Node
    person_tracker_node = Node(
        package='my_robot_pkg',
        executable='person_tracker_node',
        name='person_tracker',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'model_path': 'yolov8n.pt'},
            {'enable_visualization': LaunchConfiguration('enable_visualization')},
        ],
        remappings=[
            ('microcontrolling', '/robot/maincontrolling'),
            ('person_position', '/perception/tracked_person_position'),
        ]
    )

    return LaunchDescription([
        use_sim_time,
        enable_visualization,
        human_position_node,
        person_tracker_node,
    ])
