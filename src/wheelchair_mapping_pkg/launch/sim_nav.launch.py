import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. Dynamically locate your package
    pkg_wheelchair = FindPackageShare('wheelchair_mapping_pkg')

    # 2. Define dynamic paths to your config files
    map_yaml = PathJoinSubstitution([pkg_wheelchair, 'maps', 'my_map.yaml'])
    ekf_config = PathJoinSubstitution([pkg_wheelchair, 'config', 'ekf.yaml'])
    nav2_params = PathJoinSubstitution([pkg_wheelchair, 'config', 'nav2_params.yaml'])
    rviz_config = PathJoinSubstitution([pkg_wheelchair, 'config', 'rviz.rviz'])
    
    # CRITICAL: Force simulation time to True for all nodes
    # This ensures Nav2 uses the /clock topic from Gazebo instead of your laptop's CPU clock
    sim_time = {'use_sim_time': True}

    return LaunchDescription([
        # --- A. Static Transform ---
        # Links the base of the robot to where your physical lidar would sit
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_static_transform',
            arguments=['1.08', '0.08', '0.23', '0', '0', '0', 'base_link', 'laser'],
        ),

        # --- B. EKF Node ---
        # Fuses the simulated Gazebo /wheel/odom and /imu/data topics
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config, sim_time],
            remappings=[('/odometry/filtered', '/odom')],
        ),

        # --- C. Nav2 Stack ---
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml}, sim_time],
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_params, sim_time],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params, sim_time],
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params, sim_time],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params, sim_time],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params, sim_time],
        ),

        # --- D. Nav2 Lifecycle Manager ---
        # Boots up all the Nav2 nodes in the mathematically correct order
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': [
                    'map_server', 
                    'amcl', 
                    'planner_server', 
                    'controller_server', 
                    'bt_navigator', 
                    'behavior_server'
                ],
            }],
        ),

        # --- E. RViz 2 ---
        # Automatically opens RViz with your saved configuration
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[sim_time],
            output='screen'
        ),
    ])