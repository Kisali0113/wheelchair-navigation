from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')
    wheel_base = LaunchConfiguration('wheel_base')
    
    pkg_wheelchair = FindPackageShare('wheelchair_mapping_pkg')
    rviz_config = PathJoinSubstitution([pkg_wheelchair, 'config', 'mapping.rviz'])
    sim_time = {'use_sim_time': False}

    ekf_config = PathJoinSubstitution([
        FindPackageShare('wheelchair_mapping_pkg'),
        'config',
        'ekf.yaml',
    ])

    slam_params_file = PathJoinSubstitution([
    FindPackageShare('wheelchair_mapping_pkg'),
    'config',
    'slam_toolbox.yaml',
])

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/ttyACM1',
            description='Serial device path for Arduino',
        ),
        DeclareLaunchArgument(
            'baud_rate', default_value='115200',
            description='Arduino serial baud rate',
        ),
        DeclareLaunchArgument(
            'wheel_base', default_value='1.05',
            description='Wheelbase of the robot in meters',
        ),

        Node(
            package='wheelchair_mapping_pkg',
            executable='serial_sensor_node',
            name='serial_sensor_node',
            output='screen',
            parameters=[{
                'serial_port': serial_port,
                'baud_rate': baud_rate,
                'read_rate': 50.0,
            }],
        ),
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

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config],
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'scan_topic': '/scan',
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'map_frame': 'map',
                'use_sim_time': False,
                'mode': 'mapping',
                'use_lifecycle_manager': True
            }],
        ),
        
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,                 # 🔥 THIS IS KEY
                'node_names': ['slam_toolbox']     # managed nodes
            }]
        ),


        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[sim_time],
            output='screen'
        )

        # RViz workflow hints:
        #  - Set fixed frame to 'map'
        #  - Add display for LaserScan on '/scan'
        #  - Add Map display to visualize the occupancy grid
        #  - Add TF display to see map->odom->base_link->laser_frame
        #  - Add Odometry display for '/odometry/filtered' and optionally '/wheel/odom'
    ])
