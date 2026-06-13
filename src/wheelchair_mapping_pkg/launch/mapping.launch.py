from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')
    wheel_radius = LaunchConfiguration('wheel_radius')
    wheel_base = LaunchConfiguration('wheel_base')
    ticks_per_revolution = LaunchConfiguration('ticks_per_revolution')
    steer_ticks_per_revolution = LaunchConfiguration('steer_ticks_per_revolution')
    steer_max_angle_deg = LaunchConfiguration('steer_max_angle_deg')

    ekf_config = PathJoinSubstitution([
        FindPackageShare('wheelchair_mapping_pkg'),
        'config',
        'ekf.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/ttyACM0',
            description='Serial device path for Arduino',
        ),
        DeclareLaunchArgument(
            'baud_rate', default_value='115200',
            description='Arduino serial baud rate',
        ),
        DeclareLaunchArgument(
            'wheel_radius', default_value='0.220',
            description='Wheel radius in meters',
        ),
        DeclareLaunchArgument(
            'wheel_base', default_value='0.705',
            description='Distance from rear axle to front wheel',
        ),
        DeclareLaunchArgument(
            'ticks_per_revolution', default_value='1024',
            description='Encoder ticks per wheel revolution',
        ),
        DeclareLaunchArgument(
            'steer_ticks_per_revolution', default_value='1024',
            description='Steering encoder ticks per revolution',
        ),
        DeclareLaunchArgument(
            'steer_max_angle_deg', default_value='60.0',
            description='Maximum steering angle for front axle in degrees',
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
                'wheel_radius': wheel_radius,
                'wheel_base': wheel_base,
                'ticks_per_revolution': ticks_per_revolution,
                'steer_ticks_per_revolution': steer_ticks_per_revolution,
                'steer_max_angle_deg': steer_max_angle_deg,
                'steer_center_offset': 0,
            }],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_static_transform',
            output='screen',
            arguments=['1.08', '0.08', '0.23', '0', '0', '0', 'base_link', 'laser'],
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
)
        # RViz workflow hints:
        #  - Set fixed frame to 'map'
        #  - Add display for LaserScan on '/scan'
        #  - Add Map display to visualize the occupancy grid
        #  - Add TF display to see map->odom->base_link->laser_frame
        #  - Add Odometry display for '/odometry/filtered' and optionally '/wheel/odom'
    ])
