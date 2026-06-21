#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():

    return LaunchDescription([

        # Source and launch Realsense workspace
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                'source ~/realsense_ws/install/setup.bash && '
                'ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true'
            ],
            output='screen'
        ),

        # Source and launch Navigation workspace
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                'source ~/fyp_ws/install/setup.bash && '
                'ros2 launch rplidar_ros rplidar_c1_launch.py'  # For RPLIDAR A1
                'ros2 launch wheelchair_mapping_pkg autonomous_navigation.launch.py'
                'ros2 launch firebase_bridge firebase_bridge.launch.py'
            ],
            output='screen'
        ),

        # Source and launch AI workspace
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                'source ~/ai_ws/venv/bin/activate && '
                'python3 /home/kisali/ai_ws/person.py'
            ],
            output='screen'
        ),
    ])