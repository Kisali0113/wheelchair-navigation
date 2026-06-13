#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from geometry_msgs.msg import Quaternion, TransformStamped
import tf2_ros


class EncoderOdomNode(Node):
    """Convert wheel encoder ticks into planar odometry with a simple tricycle model."""

    def __init__(self):
        super().__init__('encoder_odom_node')

        self.declare_parameter('wheel_radius', 0.220)
        self.declare_parameter('wheel_base', 0.705)
        self.declare_parameter('ticks_per_revolution', 1024)
        self.declare_parameter('steer_ticks_per_revolution', 1024)
        self.declare_parameter('steer_max_angle_deg', 60.0)
        self.declare_parameter('steer_center_offset',598)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.ticks_per_revolution = self.get_parameter('ticks_per_revolution').value
        self.steer_ticks_per_revolution = self.get_parameter('steer_ticks_per_revolution').value
        self.steer_max_angle = math.radians(self.get_parameter('steer_max_angle_deg').value)
        self.steer_center_offset = self.get_parameter('steer_center_offset').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.steer_angle = 0.0
        self.last_wheel_ticks = None
        self.last_time = self.get_clock().now()

        self.wheel_sub = self.create_subscription(Int32, '/wheelticks', self.wheel_ticks_callback, 10)
        self.steer_sub = self.create_subscription(Int32, '/steerticks', self.steer_ticks_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/wheel/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info('Encoder odometry node started')

    def wheel_ticks_callback(self, msg: Int32) -> None:
        current_time = self.get_clock().now()
        if self.last_wheel_ticks is None:
            self.last_wheel_ticks = msg.data
            self.last_time = current_time
            return

        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            self.get_logger().warn('Non-positive dt in odometry update, skipping')
            return

        delta_ticks = msg.data - self.last_wheel_ticks
        delta_distance = self.ticks_to_distance(delta_ticks)
        delta_yaw = self.compute_heading_change(delta_distance)

        self.x += delta_distance * math.cos(self.yaw + delta_yaw / 2.0)
        self.y += delta_distance * math.sin(self.yaw + delta_yaw / 2.0)
        self.yaw = self.normalize_angle(self.yaw + delta_yaw)

        linear_velocity = delta_distance / dt
        angular_velocity = delta_yaw / dt

        self.publish_odometry(current_time, linear_velocity, angular_velocity)

        self.last_wheel_ticks = msg.data
        self.last_time = current_time

    def steer_ticks_callback(self, msg: Int32) -> None:
        self.steer_angle = self.steer_ticks_to_angle(msg.data)

    def ticks_to_distance(self, ticks: int) -> float:
        return (ticks / float(self.ticks_per_revolution)) * 2.0 * math.pi * self.wheel_radius

    def steer_ticks_to_angle(self, ticks: int) -> float:
        normalized = float(ticks - self.steer_center_offset) / float(self.steer_ticks_per_revolution)
        angle = normalized * 2.0 * math.pi
        return max(min(angle, self.steer_max_angle), -self.steer_max_angle)

    def compute_heading_change(self, distance: float) -> float:
        if abs(self.steer_angle) < 1e-6 or self.wheel_base <= 0.0:
            return 0.0
        return math.tan(self.steer_angle) * distance / self.wheel_base

    def publish_odometry(self, stamp, linear_velocity: float, angular_velocity: float) -> None:
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self.yaw_to_quaternion(self.yaw)

        odom.pose.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.2,
        ]

        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = angular_velocity

        odom.twist.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.2, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.2, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.4,
        ]

        self.odom_pub.publish(odom)
        self.publish_tf(stamp, odom.pose.pose)

    def publish_tf(self, stamp, pose) -> None:
        transform = TransformStamped()
        transform.header.stamp = stamp.to_msg()
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.translation.z = pose.position.z
        transform.transform.rotation = pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    @staticmethod
    def yaw_to_quaternion(yaw: float) -> Quaternion:
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
