import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. Dynamically locate both packages
    pkg_robotdesign = FindPackageShare('robotdesign')
    pkg_wheelchair = FindPackageShare('wheelchair_mapping_pkg')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')

    # 2. Define dynamic paths to your specific files
    world_file = PathJoinSubstitution([pkg_robotdesign, 'worlds', 'hospital_new2.world'])
    urdf_file = PathJoinSubstitution([pkg_robotdesign, 'urdf', 'robotdesign.urdf'])
    bridge_config = PathJoinSubstitution([pkg_wheelchair, 'config', 'bridge.yaml'])

    # 3. Force simulation time for all ROS 2 nodes
    sim_time = {'use_sim_time': True}

    return LaunchDescription([
        # --- A. Start Gazebo Harmonic with your custom world ---
        # The '-r' flag tells Gazebo to start playing immediately, rather than starting paused.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
            ),
            launch_arguments={'gz_args': ['-r ', world_file]}.items(),
        ),

        # --- B. Robot State Publisher ---
        # Reads the URDF from the 'robotdesign' package and publishes it to the ROS TF tree.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_file]),
                'use_sim_time': True
            }]
        ),

        # --- C. Spawn the Robot into Gazebo ---
        # Takes the URDF from the robot_description topic and drops it into the physics engine.
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', 'robot_description',
                '-name', 'wheelchair',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.5'
            ],
            output='screen'
        ),

        # --- D. Start the ROS-Gazebo Bridge ---
        # CRITICAL: Translates Gazebo physics data into ROS topics (odom, imu, clock)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{
                'config_file': bridge_config, 
                'use_sim_time': True
            }],
            output='screen'
        ),
    ])