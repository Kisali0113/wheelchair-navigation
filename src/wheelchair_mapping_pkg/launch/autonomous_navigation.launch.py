#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_wheelchair = FindPackageShare('wheelchair_mapping_pkg')
    rviz_config = PathJoinSubstitution([pkg_wheelchair, 'config', 'rviz.rviz'])
    sim_time = {'use_sim_time': True}

    serial_port = LaunchConfiguration('serial_port')
    wheel_base = LaunchConfiguration('wheel_base')
    map_yaml = LaunchConfiguration('map_yaml')
    ekf_config = LaunchConfiguration('ekf_config')
    nav2_params = LaunchConfiguration('nav2_params')
    use_sim_time = LaunchConfiguration('use_sim_time')
    venv_site = LaunchConfiguration('venv_site')
    python_executable = LaunchConfiguration('python_executable')
  
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM0',
            description='Serial port connected to Arduino',
        ),
        DeclareLaunchArgument(
            'wheel_base',
            default_value='1.05',
            description='Wheelbase of the robot',
        ),
        DeclareLaunchArgument(
            'map_yaml',
            default_value=PathJoinSubstitution([
                FindPackageShare('wheelchair_mapping_pkg'),
                'maps',
                'wheelchair_map.yaml',
            ]),
            description='Full path to the saved map YAML file',
        ),
        DeclareLaunchArgument(
            'ekf_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('wheelchair_mapping_pkg'),
                'config',
                'ekf.yaml',
            ]),
            description='Path to EKF configuration file',
        ),
        DeclareLaunchArgument(
            'nav2_params',
            default_value=PathJoinSubstitution([
                FindPackageShare('wheelchair_mapping_pkg'),
                'config',
                'nav2_params.yaml',
            ]),
            description='Path to Nav2 parameter file',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time',
        ),
        DeclareLaunchArgument(
            'python_executable',
            default_value='python3',
            description='Python executable to run the HTTP adapter (use venv python)',
        ),
        DeclareLaunchArgument(
            'venv_site',
            default_value='',
            description='Full path to the virtualenv site-packages to prepend to PYTHONPATH',
        ),

        # Prepend virtualenv site-packages to PYTHONPATH when provided
        SetEnvironmentVariable(
            'PYTHONPATH',
            [venv_site, TextSubstitution(text=':'), EnvironmentVariable('PYTHONPATH')],
        ),


        # Arduino serial node
        Node(
            package='wheelchair_mapping_pkg',
            executable='serial_sensor_node',
            name='serial_sensor_node',
            output='screen',
            parameters=[{
                'serial_port': serial_port,
                'baud_rate': 115200,
                'read_rate': 50.0,
                'send_rate': 10.0,
                'cmd_timeout': 0.5,
            }],
        ),

        # Odometry from encoder + IMU + heading
        Node(
            package='wheelchair_mapping_pkg',
            executable='encoder_odom_node',
            name='encoder_odom_node',
            output='screen',
            parameters=[{
                'wheel_base': wheel_base,
                'velocity_topic': '/wheel_speed',
                'steer_angle_topic': '/steer_angle',
                'imu_topic': '/imu/data',
                'heading_topic': '/mag_heading',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'comp_filter_alpha': 0.98,
                'heading_filter_alpha': 0.1,
                'accel_filter_alpha': 0.2,
            }],
        ),

        # Static transform for laser scanner
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_static_transform',
            output='screen',
            arguments=['1.08', '0.08', '0.23', '0', '0', '0', 'base_link', 'laser'],
        ),


        # Static transform for Front Ultrasonic
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='us_front_transform',
            arguments=['1.10', '0.0', '0.15', '0', '0', '0', 'base_link', 'ultrasonic_front_link'],
        ),

        # Static transform for Left Ultrasonic (Angled outward by ~45 degrees or 0.78 rad)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='us_left_transform',
            arguments=['0.45', '0.31', '0.15', '0.78', '0', '0', 'base_link', 'ultrasonic_left_link'],
        ),

        # Static transform for Right Ultrasonic (Angled outward by -45 degrees or -0.78 rad)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='us_right_transform',
            arguments=['0.45', '-0.31', '0.15', '-0.78', '0', '0', 'base_link', 'ultrasonic_right_link'],
        ),

        #EKF for filtered odometry
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': use_sim_time}],
            remappings=[
                ('/odometry/filtered', '/odom')
            ],
        ),

        # Load the saved map
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': map_yaml,
                'use_sim_time': use_sim_time,
            }],
        ),

        # AMCL localization
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        ),

        # Nav2 planner
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        ),

        # Nav2 controller
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        ),

        # Nav2 behavior tree navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        ),

        # Nav2 recoveries
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        ),


        # Lifecycle manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl',
                    'planner_server',
                    'controller_server',
                    'bt_navigator',
                    'behavior_server',
                ],
            }],
        ),

        Node(
            package='wheelchair_mapping_pkg',
            executable='cmd_vel_publisher',
            name='cmd_vel_publisher',
            output='screen',
            parameters=[{
                'wheelbase': wheel_base,
            }],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[sim_time],
            output='screen'
        ),

        Node(
            package='wheelchair_mapping_pkg',
            executable='camera_node',
            name='camera_node',
            output='screen',
        ),

        # Node(
        #     package='wheelchair_mapping_pkg',
        #     executable='initial_pose_publisher',
        #     name='initial_pose_publisher',
        #     output='screen',
        # )
   ])
