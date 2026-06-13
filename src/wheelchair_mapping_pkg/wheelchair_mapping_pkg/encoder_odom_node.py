#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import Quaternion, TransformStamped
import tf2_ros


class EncoderOdomNode(Node):
    """Odometry using encoder based velocity + steering (Ackermann model)"""

    def __init__(self):
        super().__init__('encoder_odom_node')

        # Parameters
        self.declare_parameter('wheel_base', 1.05)
        self.declare_parameter('velocity_topic', '/wheel_speed')
        self.declare_parameter('steer_angle_topic', '/steer_angle')
        self.declare_parameter('heading_topic', '/mag_heading')
        self.declare_parameter('heading_filter_alpha', 0.1)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.wheel_base = self.get_parameter('wheel_base').value
        self.velocity_topic = self.get_parameter('velocity_topic').value
        self.steer_angle_topic = self.get_parameter('steer_angle_topic').value
        self.heading_topic = self.get_parameter('heading_topic').value
        self.heading_filter_alpha = self.get_parameter('heading_filter_alpha').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # State
        self.heading = 0.0  # Initialize heading for filtering
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.velocity = 0.0
        self.steer_angle = 0.0
        self.heading_filter_alpha = 0.1  # Low-pass filter coefficient (0.0 = no filter, 1.0 = no smoothing)

        self.last_time = self.get_clock().now()

        # Subscribers
        self.vel_sub = self.create_subscription(
            Float32, self.velocity_topic, self.velocity_callback, 10)

        self.steer_sub = self.create_subscription(
            Float32, self.steer_angle_topic, self.steer_callback, 10)

        self.heading_sub = self.create_subscription(
            Float32, self.heading_topic, self.heading_callback, 10)

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/wheel/odom', 10)

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Timer for integration (20 Hz)
        self.timer = self.create_timer(0.05, self.update_odometry)

        self.get_logger().info(
            f'Velocity-based odometry node started: velocity_topic={self.velocity_topic}, steer_angle_topic={self.steer_angle_topic}, heading_topic={self.heading_topic}')

    # ===============================
    # Callbacks
    # ===============================

    def velocity_callback(self, msg: Float32):
        self.velocity = msg.data  # m/s
        self.get_logger().info(f'Received velocity: {self.velocity:.3f}')

    def steer_callback(self, msg: Float32):
        raw_angle = msg.data
        if abs(raw_angle) > 2.0 * math.pi:
            self.steer_angle = math.radians(raw_angle)
            units = 'deg'
        else:
            self.steer_angle = raw_angle
            units = 'rad'
        self.get_logger().info(
            f'Received steer angle: {raw_angle:.3f} ({units}) -> {self.steer_angle:.3f} rad')

    def heading_callback(self, msg: Float32):
        raw_heading = msg.data
        if abs(raw_heading) > 2.0 * math.pi:
            raw_heading_rad = math.radians(raw_heading)
            units = 'deg'
        else:
            raw_heading_rad = raw_heading
            units = 'rad'
        
        # Apply low-pass filter to smooth heading
        self.heading = self.heading_filter_alpha * raw_heading_rad + (1.0 - self.heading_filter_alpha) * self.heading
        self.yaw = self.heading
        
        self.get_logger().info(
            f'Received magnetometer heading: {raw_heading:.3f} ({units}) -> filtered yaw: {self.yaw:.3f} rad')

    # ===============================
    # Main update loop
    # ===============================

    def update_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        if dt <= 0.0:
            return

        # Distance traveled
        delta_distance = self.velocity * dt

        # Use direct magnetometer heading
        self.yaw = self.heading

        # Update pose using current yaw
        self.x += delta_distance * math.cos(self.yaw)
        self.y += delta_distance * math.sin(self.yaw)

        # Velocities
        linear_velocity = self.velocity
        angular_velocity = (self.velocity / self.wheel_base) * math.tan(self.steer_angle)

        # Publish
        self.publish_odometry(current_time, linear_velocity, angular_velocity)

        self.last_time = current_time

    # ===============================
    # Publish
    # ===============================

    def publish_odometry(self, stamp, linear_velocity, angular_velocity):
        odom = Odometry()

        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        # Pose
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self.yaw_to_quaternion(self.yaw)

        # Pose Covariance (Required by EKF)
        odom.pose.covariance = [
            0.01, 0.0,  0.0,  0.0, 0.0, 0.0,
            0.0,  0.01, 0.0,  0.0, 0.0, 0.0,
            0.0,  0.0,  0.01, 0.0, 0.0, 0.0,
            0.0,  0.0,  0.0,  0.1, 0.0, 0.0,
            0.0,  0.0,  0.0,  0.0, 0.1, 0.0,
            0.0,  0.0,  0.0,  0.0, 0.0, 0.1
        ]

        # Twist
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.angular.z = angular_velocity

        # Twist Covariance (Required by EKF)
        odom.twist.covariance = [
            0.01, 0.0,  0.0,  0.0, 0.0, 0.0,
            0.0,  0.01, 0.0,  0.0, 0.0, 0.0,
            0.0,  0.0,  0.01, 0.0, 0.0, 0.0,
            0.0,  0.0,  0.0,  0.1, 0.0, 0.0,
            0.0,  0.0,  0.0,  0.0, 0.1, 0.0,
            0.0,  0.0,  0.0,  0.0, 0.0, 0.1
        ]

        self.odom_pub.publish(odom)

        # TF
        self.publish_tf(stamp, odom.pose.pose)

    def publish_tf(self, stamp, pose):
        transform = TransformStamped()

        transform.header.stamp = stamp.to_msg()
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame

        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.translation.z = pose.position.z
        transform.transform.rotation = pose.orientation

        self.tf_broadcaster.sendTransform(transform)

    # ===============================
    # Utilities
    # ===============================

    @staticmethod
    def yaw_to_quaternion(yaw: float) -> Quaternion:
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()