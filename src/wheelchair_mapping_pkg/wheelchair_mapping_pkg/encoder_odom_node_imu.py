#!/usr/bin/env python3

import math
from collections import deque

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, TransformStamped
import tf2_ros


class EncoderOdomNode(Node):
    """Odometry using IMU acceleration + magnetometer heading"""

    def __init__(self):
        super().__init__('encoder_odom_node')

        # Parameters
        self.declare_parameter('wheel_base', 0.705)
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('steer_angle_topic', '/steer_angle')
        self.declare_parameter('heading_topic', '/mag_heading')
        self.declare_parameter('heading_filter_alpha', 0.1)
        self.declare_parameter('accel_filter_window', 5)
        self.declare_parameter('accel_bias_samples', 100)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.wheel_base = self.get_parameter('wheel_base').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.steer_angle_topic = self.get_parameter('steer_angle_topic').value
        self.heading_topic = self.get_parameter('heading_topic').value
        self.heading_filter_alpha = self.get_parameter('heading_filter_alpha').value
        self.accel_filter_window = self.get_parameter('accel_filter_window').value
        self.accel_bias_samples = self.get_parameter('accel_bias_samples').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # State
        self.heading = 0.0  # Initialize heading for filtering
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_x_history = deque(maxlen=self.accel_filter_window)
        self.accel_y_history = deque(maxlen=self.accel_filter_window)
        self.accel_bias_x = 0.0
        self.accel_bias_y = 0.0
        self.accel_bias_count = 0
        self.accel_bias_ready = False
        self.vel_x_body = 0.0
        self.vel_y_body = 0.0
        self.steer_angle = 0.0

        self.last_time = self.get_clock().now()

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, self.imu_topic, self.imu_callback, 10)

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
            f'IMU-based odometry node started: imu_topic={self.imu_topic}, steer_angle_topic={self.steer_angle_topic}, heading_topic={self.heading_topic}')

    # ===============================
    # Callbacks
    # ===============================

    def imu_callback(self, msg: Imu):
        raw_x = msg.linear_acceleration.x
        raw_y = msg.linear_acceleration.y

        if not self.accel_bias_ready:
            self.accel_bias_x += raw_x
            self.accel_bias_y += raw_y
            self.accel_bias_count += 1

            if self.accel_bias_count >= self.accel_bias_samples:
                self.accel_bias_x /= float(self.accel_bias_samples)
                self.accel_bias_y /= float(self.accel_bias_samples)
                self.accel_bias_ready = True
                self.get_logger().info(
                    f'Accel bias calibrated: bx={self.accel_bias_x:.3f}, by={self.accel_bias_y:.3f}')
            else:
                self.get_logger().info(
                    f'Calibrating accel bias: {self.accel_bias_count}/{self.accel_bias_samples}')
            return

        corrected_x = raw_x - self.accel_bias_x
        corrected_y = raw_y - self.accel_bias_y

        self.accel_x_history.append(corrected_x)
        self.accel_y_history.append(corrected_y)

        self.accel_x = sum(self.accel_x_history) / len(self.accel_x_history)
        self.accel_y = sum(self.accel_y_history) / len(self.accel_y_history)

        self.get_logger().info(
            f'Received IMU accel raw: x={raw_x:.3f}, y={raw_y:.3f}; bias-corrected: x={corrected_x:.3f}, y={corrected_y:.3f}; filtered: x={self.accel_x:.3f}, y={self.accel_y:.3f}')

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

        if dt <= 0.0 or not self.accel_bias_ready:
            self.last_time = current_time
            return

        # Integrate acceleration to get velocity in body frame
        self.vel_x_body += self.accel_x * dt
        self.vel_y_body += self.accel_y * dt

        self.get_logger().info(f'Velocity in body frame: vx={self.vel_x_body:.3f}, vy={self.vel_y_body:.3f}')

        # Transform velocity to world frame using current yaw
        vel_x_world = self.vel_x_body * math.cos(self.yaw) - self.vel_y_body * math.sin(self.yaw)
        vel_y_world = self.vel_x_body * math.sin(self.yaw) + self.vel_y_body * math.cos(self.yaw)

        self.get_logger().info(f'Velocity in world frame: vx={vel_x_world:.3f}, vy={vel_y_world:.3f}')

        # Update pose
        self.x += vel_x_world * dt
        self.y += vel_y_world * dt

        # Linear velocity magnitude
        linear_velocity = math.sqrt(vel_x_world**2 + vel_y_world**2)
        angular_velocity = 0.0

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

        # Twist
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.angular.z = angular_velocity

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
