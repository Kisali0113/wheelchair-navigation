from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_path = get_package_share_directory('robotdesign')
    urdf_file = os.path.join(pkg_path, 'urdf', 'robotdesign.urdf')

    return LaunchDescription([

        # Start Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', 'empty.sdf'],
            output='screen'
        ),

        # Publish URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': open(urdf_file).read()
            }],
            output='screen'
        ),

        # 🔥 Spawn robot CORRECTLY
        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run', 'ros_gz_sim', 'create',
                        '-topic', 'robot_description',
                        '-name', 'robotdesign'
                    ],
                    output='screen'
                )
            ]
        ),
    ])