#!/usr/bin/env python3
"""
Master launch file for complete wheelchair/robot system.
Launches all subsystems: chair control, vision, odometry.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Generate master launch description."""

    pkg_share = FindPackageShare(package='my_robot_pkg').find('my_robot_pkg')

    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    enable_chair_control = DeclareLaunchArgument(
        'enable_chair_control',
        default_value='true',
        description='Enable chair control system'
    )

    enable_vision = DeclareLaunchArgument(
        'enable_vision',
        default_value='true',
        description='Enable vision-based person detection and tracking'
    )

    # Include chair control launch file
    chair_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'chair_control.launch.py'])
        ),
        condition=IfCondition(LaunchConfiguration('enable_chair_control')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    # Include vision system launch file
    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'vision_system.launch.py'])
        ),
        condition=IfCondition(LaunchConfiguration('enable_vision')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    return LaunchDescription([
        use_sim_time,
        enable_chair_control,
        enable_vision,
        chair_control_launch,
        vision_launch,
    ])
