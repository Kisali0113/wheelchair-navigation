#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():

    return LaunchDescription([

        # Realsense
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                'source ~/realsense_ws/install/setup.bash && '
                'ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true'
            ],
            output='screen'
        ),

        # RPLIDAR
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                'source ~/fyp_ws/install/setup.bash && '
                'ros2 launch rplidar_ros rplidar_c1_launch.py'
            ],
            output='screen'
        ),

        # Navigation
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                'source ~/fyp_ws/install/setup.bash && '
                'ros2 launch wheelchair_mapping_pkg autonomous_navigation.launch.py'
            ],
            output='screen'
        ),

        # Firebase Bridge
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                'source ~/fyp_ws/install/setup.bash && '
                'ros2 launch firebase_bridge firebase_bridge.launch.py'
            ],
            output='screen'
        ),

        # Web Video Server
        # ExecuteProcess(
        #     cmd=[
        #         'bash', '-c',
        #         'source /opt/ros/jazzy/setup.bash && '
        #         'ros2 run web_video_server web_video_server'
        #     ],
        #     output='screen'
        # ),

        # AI
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                'source /opt/ros/jazzy/setup.bash && '
                'source ~/fyp_ws/install/setup.bash && '
                'source ~/ai_ws/venv/bin/activate && '
                'python3 /home/kisali/ai_ws/person.py'
            ],
            output='screen' 
),

        ExecuteProcess(
            cmd=[
                'bash', '-c',
                'source ~/fyp_ws/install/setup.bash && '
                'ros2 run wheelchair_mapping_pkg initial_pose_publisher'
            ],
            output='screen' 
        ),
    ])