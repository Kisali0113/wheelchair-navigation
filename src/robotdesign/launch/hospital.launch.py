from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():

    pkg_path = os.path.expanduser("~/fyp_ws/src/robotdesign")

    world = os.path.join(pkg_path, "worlds", "hospital_new2.world")
    urdf = os.path.join(pkg_path, "urdf", "robotdesign.urdf")

    return LaunchDescription([

        # ✅ Proper Gazebo launch (keeps alive)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                '/opt/ros/jazzy/share/ros_gz_sim/launch/gz_sim.launch.py'
            ),
            launch_arguments={'gz_args': world}.items(),
        ),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            arguments=[urdf],
            output='screen'
        ),

        # Spawn robot
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', 'robot_description',
                '-name', 'my_robot',
                '-x', '0',
                '-y', '0',
                '-z', '0.5'
            ],
            output='screen'
        ),
    ])