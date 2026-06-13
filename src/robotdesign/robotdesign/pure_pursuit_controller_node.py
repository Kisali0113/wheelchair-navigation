#!/usr/bin/env python3
"""
Pure Pursuit Bicycle Controller Node for ROS2 Navigation

This node implements the pure pursuit algorithm for Ackermann steering vehicles.
It subscribes to odometry and planned path, computes steering angle using the
bicycle model, and publishes Ackermann drive commands.

Pure Pursuit Formula:
delta = atan((2 * L * sin(alpha)) / Ld)

Where:
- delta: steering angle
- L: wheelbase
- alpha: heading error to lookahead point
- Ld: lookahead distance

Author: ROS2 Navigation Team
License: BSD-3-Clause
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, PointStamped
import math
import tf2_geometry_msgs
from tf2_ros import Buffer, TransformListener


class PurePursuitControllerNode(Node):
    """
    ROS2 node implementing pure pursuit control for Ackermann vehicles.

    This controller computes steering angles based on the pure pursuit algorithm
    and maintains constant velocity control for path following.
    """

    def __init__(self):
        super().__init__('pure_pursuit_controller_node')

        # Declare parameters
        self.declare_parameter('wheelbase', 0.42)  # meters
        self.declare_parameter('lookahead_distance', 1.0)  # meters
        self.declare_parameter('target_speed', 0.5)  # m/s
        self.declare_parameter('max_steering_angle', 0.5236)  # radians (~30 degrees)
        self.declare_parameter('min_lookahead_distance', 0.5)  # meters
        self.declare_parameter('max_lookahead_distance', 2.0)  # meters
        self.declare_parameter('speed_reduction_ratio', 0.5)  # reduce speed in turns
        self.declare_parameter('goal_tolerance', 0.2)  # meters
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('path_topic', '/plan')
        self.declare_parameter('cmd_topic', '/ackermann_cmd')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')

        # Get parameters
        self.wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.lookahead_distance = self.get_parameter('lookahead_distance').get_parameter_value().double_value
        self.target_speed = self.get_parameter('target_speed').get_parameter_value().double_value
        self.max_steering_angle = self.get_parameter('max_steering_angle').get_parameter_value().double_value
        self.min_lookahead_distance = self.get_parameter('min_lookahead_distance').get_parameter_value().double_value
        self.max_lookahead_distance = self.get_parameter('max_lookahead_distance').get_parameter_value().double_value
        self.speed_reduction_ratio = self.get_parameter('speed_reduction_ratio').get_parameter_value().double_value
        self.goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.robot_frame = self.get_parameter('robot_frame').get_parameter_value().string_value

        # State variables
        self.current_path = []
        self.robot_pose = None
        self.current_speed = 0.0
        self.at_goal = False

        # TF2 buffer for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create subscriptions
        self.odom_subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )

        self.path_subscription = self.create_subscription(
            Path,
            path_topic,
            self.path_callback,
            10
        )

        # Create publishers
        self.cmd_publisher = self.create_publisher(
            AckermannDriveStamped,
            cmd_topic,
            10
        )

        # Debug publishers
        self.lookahead_publisher = self.create_publisher(
            PointStamped,
            '/lookahead_point',
            10
        )

        # Control timer (50 Hz)
        self.create_timer(0.02, self.control_loop)

        self.get_logger().info('Pure Pursuit Controller Node initialized')
        self.get_logger().info('.2f')
        self.get_logger().info('.2f')
        self.get_logger().info('.2f')

    def odom_callback(self, msg: Odometry):
        """
        Callback for odometry messages.

        Args:
            msg: Odometry message with robot pose and velocity
        """
        self.robot_pose = msg.pose.pose
        self.current_speed = msg.twist.twist.linear.x

    def path_callback(self, msg: Path):
        """
        Callback for planned path messages.

        Args:
            msg: Path message with planned waypoints
        """
        if len(msg.poses) < 2:
            self.get_logger().warn('Path too short for navigation')
            return

        self.current_path = msg.poses
        self.at_goal = False

        self.get_logger().info(f'Received path with {len(msg.poses)} waypoints')

    def control_loop(self):
        """
        Main control loop executed at 50 Hz.
        Computes and publishes steering commands.
        """
        if not self.robot_pose or not self.current_path:
            # No pose or path available, stop the robot
            self.publish_stop_command()
            return

        if self.at_goal:
            self.publish_stop_command()
            return

        # Get current robot position in map frame
        try:
            robot_position = self.get_robot_position_in_map()
            if robot_position is None:
                self.get_logger().warn('Could not transform robot position to map frame')
                self.publish_stop_command()
                return
        except Exception as e:
            self.get_logger().error(f'TF transform error: {e}')
            self.publish_stop_command()
            return

        # Find closest waypoint
        closest_index = self.find_closest_waypoint(robot_position)
        if closest_index == -1:
            self.get_logger().warn('No valid waypoint found')
            self.publish_stop_command()
            return

        # Check if we're at the goal
        if self.is_at_goal(robot_position):
            self.at_goal = True
            self.get_logger().info('Reached navigation goal!')
            self.publish_stop_command()
            return

        # Get lookahead point
        lookahead_point = self.get_lookahead_point(closest_index, robot_position)
        if lookahead_point is None:
            self.get_logger().warn('No lookahead point found')
            self.publish_stop_command()
            return

        # Publish lookahead point for debugging
        self.publish_lookahead_point(lookahead_point)

        # Compute steering angle using pure pursuit
        steering_angle = self.compute_pure_pursuit_steering(robot_position, lookahead_point)

        # Compute target speed (reduce in turns)
        target_speed = self.compute_target_speed(steering_angle)

        # Create and publish Ackermann command
        self.publish_ackermann_command(steering_angle, target_speed)

    def get_robot_position_in_map(self):
        """
        Get current robot position in map frame.

        Returns:
            tuple: (x, y, yaw) in map frame, or None if transform fails
        """
        try:
            # Create pose stamped for transformation
            robot_pose_stamped = PoseStamped()
            robot_pose_stamped.header.frame_id = self.robot_frame
            robot_pose_stamped.header.stamp = self.get_clock().now().to_msg()
            robot_pose_stamped.pose = self.robot_pose

            # Transform to map frame
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                rclpy.time.Time()
            )

            transformed_pose = tf2_geometry_msgs.do_transform_pose_stamped(
                robot_pose_stamped,
                transform
            )

            # Extract position and orientation
            x = transformed_pose.pose.position.x
            y = transformed_pose.pose.position.y

            # Convert quaternion to yaw
            orientation = transformed_pose.pose.orientation
            yaw = math.atan2(
                2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
            )

            return (x, y, yaw)

        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None

    def find_closest_waypoint(self, robot_position):
        """
        Find the index of the closest waypoint to the robot.

        Args:
            robot_position: tuple (x, y, yaw) of robot position

        Returns:
            int: Index of closest waypoint
        """
        if not self.current_path:
            return -1

        robot_x, robot_y, _ = robot_position
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

    def is_at_goal(self, robot_position):
        """
        Check if robot is within tolerance of the goal.

        Args:
            robot_position: tuple (x, y, yaw) of robot position

        Returns:
            bool: True if at goal
        """
        if not self.current_path:
            return True

        robot_x, robot_y, _ = robot_position
        goal_pose = self.current_path[-1]

        dx = goal_pose.pose.position.x - robot_x
        dy = goal_pose.pose.position.y - robot_y
        distance = math.sqrt(dx*dx + dy*dy)

        return distance <= self.goal_tolerance

    def get_lookahead_point(self, closest_index, robot_position):
        """
        Get the lookahead point on the path.

        Args:
            closest_index: Index of closest waypoint
            robot_position: tuple (x, y, yaw) of robot position

        Returns:
            tuple: (x, y) of lookahead point
        """
        robot_x, robot_y, _ = robot_position

        # Start from closest index and accumulate distance
        accumulated_distance = 0.0
        current_point = self.current_path[closest_index]

        for i in range(closest_index, len(self.current_path) - 1):
            next_point = self.current_path[i + 1]

            # Calculate distance to next point
            dx = next_point.pose.position.x - current_point.pose.position.x
            dy = next_point.pose.position.y - current_point.pose.position.y
            segment_distance = math.sqrt(dx*dx + dy*dy)

            if accumulated_distance + segment_distance >= self.lookahead_distance:
                # Interpolate point along this segment
                remaining_distance = self.lookahead_distance - accumulated_distance
                ratio = remaining_distance / segment_distance

                x = current_point.pose.position.x + dx * ratio
                y = current_point.pose.position.y + dy * ratio

                return (x, y)

            accumulated_distance += segment_distance
            current_point = next_point

        # If we reach the end, return the last point
        if self.current_path:
            last_point = self.current_path[-1]
            return (last_point.pose.position.x, last_point.pose.position.y)

        return None

    def compute_pure_pursuit_steering(self, robot_position, lookahead_point):
        """
        Compute steering angle using pure pursuit algorithm.

        Args:
            robot_position: tuple (x, y, yaw) of robot position
            lookahead_point: tuple (x, y) of lookahead point

        Returns:
            float: Steering angle in radians
        """
        robot_x, robot_y, robot_yaw = robot_position
        target_x, target_y = lookahead_point

        # Transform lookahead point to robot frame
        dx = target_x - robot_x
        dy = target_y - robot_y

        # Rotate to robot heading
        local_x = dx * math.cos(robot_yaw) + dy * math.sin(robot_yaw)
        local_y = -dx * math.sin(robot_yaw) + dy * math.cos(robot_yaw)

        # Compute heading error (alpha)
        alpha = math.atan2(local_y, local_x)

        # Pure pursuit formula
        # delta = atan((2 * L * sin(alpha)) / Ld)
        if abs(alpha) < 1e-6:
            steering_angle = 0.0
        else:
            steering_angle = math.atan((2.0 * self.wheelbase * math.sin(alpha)) / self.lookahead_distance)

        # Apply steering limits
        steering_angle = max(-self.max_steering_angle, min(self.max_steering_angle, steering_angle))

        return steering_angle

    def compute_target_speed(self, steering_angle):
        """
        Compute target speed based on steering angle.

        Args:
            steering_angle: Current steering angle in radians

        Returns:
            float: Target speed in m/s
        """
        # Reduce speed in sharp turns
        steering_ratio = abs(steering_angle) / self.max_steering_angle
        speed_factor = 1.0 - (steering_ratio * self.speed_reduction_ratio)

        return self.target_speed * max(0.1, speed_factor)  # Minimum speed

    def publish_ackermann_command(self, steering_angle, speed):
        """
        Publish Ackermann drive command.

        Args:
            steering_angle: Steering angle in radians
            speed: Linear speed in m/s
        """
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self.robot_frame

        cmd.drive.steering_angle = steering_angle
        cmd.drive.speed = speed

        self.cmd_publisher.publish(cmd)

        # Debug logging (throttled)
        if self.get_clock().now().nanoseconds % 1000000000 < 20000000:  # ~1 Hz
            self.get_logger().debug('.2f')

    def publish_stop_command(self):
        """Publish zero velocity command to stop the robot."""
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self.robot_frame

        cmd.drive.steering_angle = 0.0
        cmd.drive.speed = 0.0

        self.cmd_publisher.publish(cmd)

    def publish_lookahead_point(self, point):
        """
        Publish lookahead point for debugging/visualization.

        Args:
            point: tuple (x, y) of lookahead point
        """
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        msg.point.x = point[0]
        msg.point.y = point[1]
        msg.point.z = 0.0

        self.lookahead_publisher.publish(msg)


def main(args=None):
    """Main function to run the pure pursuit controller node."""
    rclpy.init(args=args)
    node = PurePursuitControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Pure Pursuit Controller Node shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
