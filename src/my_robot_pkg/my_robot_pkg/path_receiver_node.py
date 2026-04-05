#!/usr/bin/env python3
"""
Path Receiver Node for ROS2 Navigation

This node subscribes to the planned path from Nav2 and extracts waypoints
for use by the controller node. It stores the current path and provides
access to path points for lookahead calculations.

Author: ROS2 Navigation Team
License: BSD-3-Clause
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math


class PathReceiverNode(Node):
    """
    ROS2 node that receives and stores planned paths from Nav2.

    This node subscribes to the global plan topic and maintains a list
    of waypoints that can be accessed by the controller for path following.
    """

    def __init__(self):
        super().__init__('path_receiver_node')

        # Declare parameters
        self.declare_parameter('path_topic', '/plan')
        self.declare_parameter('max_path_age', 5.0)  # seconds
        self.declare_parameter('min_path_length', 2)  # minimum waypoints

        # Get parameters
        path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        self.max_path_age = self.get_parameter('max_path_age').get_parameter_value().double_value
        self.min_path_length = self.get_parameter('min_path_length').get_parameter_value().integer_value

        # Path storage
        self.current_path = []
        self.path_timestamp = None
        self.path_frame_id = None

        # Create subscription for planned path
        self.path_subscription = self.create_subscription(
            Path,
            path_topic,
            self.path_callback,
            10
        )

        # Create publisher for current path (for debugging/visualization)
        self.current_path_publisher = self.create_publisher(
            Path,
            '/current_path',
            10
        )

        # Create timer for path age checking
        self.create_timer(1.0, self.check_path_age)

        self.get_logger().info('Path Receiver Node initialized')
        self.get_logger().info(f'Subscribing to path topic: {path_topic}')

    def path_callback(self, msg: Path):
        """
        Callback for planned path messages.

        Args:
            msg: Path message containing planned waypoints
        """
        if len(msg.poses) < self.min_path_length:
            self.get_logger().warn(f'Path too short: {len(msg.poses)} waypoints (minimum: {self.min_path_length})')
            return

        # Store path information
        self.current_path = msg.poses
        self.path_timestamp = self.get_clock().now()
        self.path_frame_id = msg.header.frame_id

        self.get_logger().info(f'Received path with {len(msg.poses)} waypoints')

        # Publish current path for visualization
        self.publish_current_path()

    def check_path_age(self):
        """
        Timer callback to check if current path is too old.
        Clears path if it's older than max_path_age.
        """
        if self.path_timestamp is None:
            return

        age = (self.get_clock().now() - self.path_timestamp).nanoseconds / 1e9

        if age > self.max_path_age:
            self.get_logger().warn('.1f')
            self.clear_path()

    def clear_path(self):
        """Clear the current stored path."""
        self.current_path = []
        self.path_timestamp = None
        self.path_frame_id = None

        # Publish empty path
        empty_path = Path()
        empty_path.header.stamp = self.get_clock().now().to_msg()
        empty_path.header.frame_id = 'map'
        self.current_path_publisher.publish(empty_path)

    def publish_current_path(self):
        """Publish the current stored path for visualization."""
        if not self.current_path:
            return

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.path_frame_id or 'map'
        path_msg.poses = self.current_path

        self.current_path_publisher.publish(path_msg)

    def get_path_points(self):
        """
        Get the current path waypoints.

        Returns:
            list: List of PoseStamped messages representing path waypoints
        """
        return self.current_path.copy()

    def get_closest_waypoint_index(self, robot_x, robot_y):
        """
        Find the index of the closest waypoint to the robot position.

        Args:
            robot_x: Robot x position
            robot_y: Robot y position

        Returns:
            int: Index of closest waypoint, or -1 if no path
        """
        if not self.current_path:
            return -1

        min_distance = float('inf')
        closest_index = -1

        for i, pose_stamped in enumerate(self.current_path):
            dx = pose_stamped.pose.position.x - robot_x
            dy = pose_stamped.pose.position.y - robot_y
            distance = math.sqrt(dx*dx + dy*dy)

            if distance < min_distance:
                min_distance = distance
                closest_index = i

        return closest_index

    def get_lookahead_point(self, current_index, lookahead_distance):
        """
        Get the lookahead point on the path.

        Args:
            current_index: Current waypoint index
            lookahead_distance: Distance to look ahead

        Returns:
            tuple: (x, y) coordinates of lookahead point, or None
        """
        if not self.current_path or current_index < 0:
            return None

        # Start from current index and accumulate distance
        accumulated_distance = 0.0
        current_point = self.current_path[current_index]

        for i in range(current_index, len(self.current_path) - 1):
            next_point = self.current_path[i + 1]

            # Calculate distance to next point
            dx = next_point.pose.position.x - current_point.pose.position.x
            dy = next_point.pose.position.y - current_point.pose.position.y
            segment_distance = math.sqrt(dx*dx + dy*dy)

            if accumulated_distance + segment_distance >= lookahead_distance:
                # Interpolate point along this segment
                remaining_distance = lookahead_distance - accumulated_distance
                ratio = remaining_distance / segment_distance

                x = current_point.pose.position.x + dx * ratio
                y = current_point.pose.position.y + dy * ratio

                return (x, y)

            accumulated_distance += segment_distance
            current_point = next_point

        # If we reach the end of the path, return the last point
        if self.current_path:
            last_point = self.current_path[-1]
            return (last_point.pose.position.x, last_point.pose.position.y)

        return None

    def is_path_valid(self):
        """
        Check if the current path is valid and not too old.

        Returns:
            bool: True if path is valid
        """
        if not self.current_path or self.path_timestamp is None:
            return False

        age = (self.get_clock().now() - self.path_timestamp).nanoseconds / 1e9
        return age <= self.max_path_age


def main(args=None):
    """Main function to run the path receiver node."""
    rclpy.init(args=args)
    node = PathReceiverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Path Receiver Node shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()