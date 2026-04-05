#!/usr/bin/env python3
"""
Launch file for autonomous navigation system.

This launch file starts all the navigation nodes:
- Goal sender node
- Path receiver node
- Pure pursuit controller node
- PWM conversion node

Author: ROS2 Navigation Team
License: BSD-3-Clause
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for navigation system."""

    # Declare launch arguments
    wheelbase = DeclareLaunchArgument(
        'wheelbase',
        default_value='0.42',
        description='Wheelbase of the vehicle in meters'
    )

    lookahead_distance = DeclareLaunchArgument(
        'lookahead_distance',
        default_value='1.0',
        description='Lookahead distance for pure pursuit in meters'
    )

    target_speed = DeclareLaunchArgument(
        'target_speed',
        default_value='0.5',
        description='Target speed in m/s'
    )

    max_steering_angle = DeclareLaunchArgument(
        'max_steering_angle',
        default_value='0.5236',
        description='Maximum steering angle in radians (~30 degrees)'
    )

    # Goal Sender Node
    goal_sender_node = Node(
        package='my_robot_pkg',
        executable='goal_sender_node',
        name='goal_sender',
        output='screen',
        parameters=[
            {'map_frame': 'map'},
            {'robot_frame': 'base_link'},
            {'goal_xy_topic': '/goal_xy'},
            {'goal_pose_topic': '/goal_pose'},
            {'rviz_goal_topic': '/move_base_simple/goal'},
            {'default_orientation': 0.0},
        ]
    )

    # Path Receiver Node
    path_receiver_node = Node(
        package='my_robot_pkg',
        executable='path_receiver_node',
        name='path_receiver',
        output='screen',
        parameters=[
            {'path_topic': '/plan'},
            {'max_path_age': 5.0},
            {'min_path_length': 2},
        ]
    )

    # Pure Pursuit Controller Node
    pure_pursuit_controller_node = Node(
        package='my_robot_pkg',
        executable='pure_pursuit_controller_node',
        name='pure_pursuit_controller',
        output='screen',
        parameters=[
            {'wheelbase': LaunchConfiguration('wheelbase')},
            {'lookahead_distance': LaunchConfiguration('lookahead_distance')},
            {'target_speed': LaunchConfiguration('target_speed')},
            {'max_steering_angle': LaunchConfiguration('max_steering_angle')},
            {'min_lookahead_distance': 0.5},
            {'max_lookahead_distance': 2.0},
            {'speed_reduction_ratio': 0.5},
            {'goal_tolerance': 0.2},
            {'odom_topic': '/odometry/filtered'},
            {'path_topic': '/plan'},
            {'cmd_topic': '/ackermann_cmd'},
            {'map_frame': 'map'},
            {'robot_frame': 'base_link'},
        ]
    )

    # PWM Conversion Node
    pwm_conversion_node = Node(
        package='my_robot_pkg',
        executable='pwm_conversion_node',
        name='pwm_conversion',
        output='screen',
        parameters=[
            {'steering_pwm_min': 1000},
            {'steering_pwm_max': 2000},
            {'steering_pwm_neutral': 1500},
            {'steering_angle_max': LaunchConfiguration('max_steering_angle')},
            {'motor_pwm_min': 0},
            {'motor_pwm_max': 255},
            {'motor_pwm_stop': 127},
            {'max_speed': LaunchConfiguration('target_speed')},
            {'ackermann_topic': '/ackermann_cmd'},
            {'pwm_topic': '/motor_pwm'},
            {'steering_servo_channel': 0},
            {'motor_channel': 1},
        ]
    )

    return LaunchDescription([
        wheelbase,
        lookahead_distance,
        target_speed,
        max_steering_angle,
        goal_sender_node,
        path_receiver_node,
        pure_pursuit_controller_node,
        pwm_conversion_node,
    ])
