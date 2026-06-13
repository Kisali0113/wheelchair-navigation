import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_wheelchair = FindPackageShare('wheelchair_mapping_pkg')
    ekf_config = PathJoinSubstitution([pkg_wheelchair, 'config', 'ekf.yaml'])
    rviz_config = PathJoinSubstitution([pkg_wheelchair, 'config', 'rviz.rviz'])
    
    # Force simulation time
    sim_time = {'use_sim_time': True}

    return LaunchDescription([
        # 1. Static Transform (Tell SLAM where the Lidar is)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_static_transform',
            arguments=['1.05', '0.08', '0.23', '0', '0', '0', 'base_link', 'laser'],
        ),

        # 2. IMU Static Transform (The "Stranger Danger" lock bypass)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_static_transform',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'robotdesign/base_link/wheelchair_imu'],
        ),

        # 3. EKF Node (Tell SLAM how the wheels are moving)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[ekf_config, sim_time],
            remappings=[('/odometry/filtered', '/odom')],
        ),

        # 4. SLAM Toolbox (The actual mapping brain)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[
                sim_time,
                {'odom_frame': 'odom',
                 'map_frame': 'map',
                 'mode': 'mapping',
                 'base_frame': 'base_link',
                 'scan_topic': '/scan',
                 'use_lifecycle_manager': True
                }
            ],
            output='screen'
        ),

        # 5. RViz 2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[sim_time],
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
    ])
