#!/usr/bin/env python3
"""
PWM Conversion Node for ROS2 Ackermann Control

This node converts Ackermann drive commands (steering angle and velocity)
into PWM values for motor and servo control. Designed for Arduino Mega
with servo steering and DC motor control.

PWM Ranges:
- Steering Servo: 1000-2000 μs (neutral at 1500 μs)
- Drive Motor: 0-255 (0 = full reverse, 127 = stop, 255 = full forward)

Author: ROS2 Navigation Team
License: BSD-3-Clause
"""

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import UInt16MultiArray
import math


class PwmConversionNode(Node):
    """
    ROS2 node that converts Ackermann commands to PWM values.

    This node subscribes to AckermannDriveStamped messages and converts
    steering angles and velocities into PWM signals for servo and motor control.
    """

    def __init__(self):
        super().__init__('pwm_conversion_node')

        # Declare parameters
        self.declare_parameter('steering_pwm_min', 1000)  # microseconds
        self.declare_parameter('steering_pwm_max', 2000)  # microseconds
        self.declare_parameter('steering_pwm_neutral', 1500)  # microseconds
        self.declare_parameter('steering_angle_max', 0.5236)  # radians (~30 degrees)
        self.declare_parameter('motor_pwm_min', 0)  # 0-255
        self.declare_parameter('motor_pwm_max', 255)  # 0-255
        self.declare_parameter('motor_pwm_stop', 127)  # neutral/stop value
        self.declare_parameter('max_speed', 1.0)  # m/s
        self.declare_parameter('ackermann_topic', '/ackermann_cmd')
        self.declare_parameter('pwm_topic', '/motor_pwm')
        self.declare_parameter('steering_servo_channel', 0)  # PWM channel for steering
        self.declare_parameter('motor_channel', 1)  # PWM channel for motor

        # Get parameters
        self.steering_pwm_min = self.get_parameter('steering_pwm_min').get_parameter_value().integer_value
        self.steering_pwm_max = self.get_parameter('steering_pwm_max').get_parameter_value().integer_value
        self.steering_pwm_neutral = self.get_parameter('steering_pwm_neutral').get_parameter_value().integer_value
        self.steering_angle_max = self.get_parameter('steering_angle_max').get_parameter_value().double_value
        self.motor_pwm_min = self.get_parameter('motor_pwm_min').get_parameter_value().integer_value
        self.motor_pwm_max = self.get_parameter('motor_pwm_max').get_parameter_value().integer_value
        self.motor_pwm_stop = self.get_parameter('motor_pwm_stop').get_parameter_value().integer_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        ackermann_topic = self.get_parameter('ackermann_topic').get_parameter_value().string_value
        pwm_topic = self.get_parameter('pwm_topic').get_parameter_value().string_value
        self.steering_channel = self.get_parameter('steering_servo_channel').get_parameter_value().integer_value
        self.motor_channel = self.get_parameter('motor_channel').get_parameter_value().integer_value

        # Create subscription for Ackermann commands
        self.ackermann_subscription = self.create_subscription(
            AckermannDriveStamped,
            ackermann_topic,
            self.ackermann_callback,
            10
        )

        # Create publisher for PWM commands
        self.pwm_publisher = self.create_publisher(
            UInt16MultiArray,
            pwm_topic,
            10
        )

        # Initialize PWM array (for multiple channels)
        self.pwm_array = UInt16MultiArray()
        self.pwm_array.data = [0] * 8  # Support up to 8 PWM channels

        # Set initial values to neutral/stop
        self.pwm_array.data[self.steering_channel] = self.steering_pwm_neutral
        self.pwm_array.data[self.motor_channel] = self.motor_pwm_stop

        self.get_logger().info('PWM Conversion Node initialized')
        self.get_logger().info(f'Subscribing to: {ackermann_topic}')
        self.get_logger().info(f'Publishing to: {pwm_topic}')
        self.get_logger().info(f'Steering PWM range: {self.steering_pwm_min}-{self.steering_pwm_max}')
        self.get_logger().info(f'Motor PWM range: {self.motor_pwm_min}-{self.motor_pwm_max}')

    def ackermann_callback(self, msg: AckermannDriveStamped):
        """
        Callback for Ackermann drive commands.

        Args:
            msg: AckermannDriveStamped message with steering and speed
        """
        steering_angle = msg.drive.steering_angle  # radians
        speed = msg.drive.speed  # m/s

        # Convert steering angle to PWM
        steering_pwm = self.steering_angle_to_pwm(steering_angle)

        # Convert speed to PWM
        motor_pwm = self.speed_to_pwm(speed)

        # Update PWM array
        self.pwm_array.data[self.steering_channel] = steering_pwm
        self.pwm_array.data[self.motor_channel] = motor_pwm

        # Publish PWM commands
        self.pwm_publisher.publish(self.pwm_array)

        # Debug logging (throttled to ~2 Hz)
        if self.get_clock().now().nanoseconds % 500000000 < 20000000:
            self.get_logger().debug('.2f')

    def steering_angle_to_pwm(self, angle_rad):
        """
        Convert steering angle in radians to PWM microseconds.

        Args:
            angle_rad: Steering angle in radians

        Returns:
            int: PWM value in microseconds
        """
        # Clamp angle to valid range
        angle_rad = max(-self.steering_angle_max, min(self.steering_angle_max, angle_rad))

        # Convert to normalized value (-1 to 1)
        normalized_angle = angle_rad / self.steering_angle_max

        # Convert to PWM range
        if normalized_angle >= 0:
            # Positive angle (right turn)
            pwm_range = self.steering_pwm_max - self.steering_pwm_neutral
            pwm_value = self.steering_pwm_neutral + (normalized_angle * pwm_range)
        else:
            # Negative angle (left turn)
            pwm_range = self.steering_pwm_neutral - self.steering_pwm_min
            pwm_value = self.steering_pwm_neutral + (normalized_angle * pwm_range)

        # Ensure PWM is within bounds and return as integer
        pwm_value = max(self.steering_pwm_min, min(self.steering_pwm_max, pwm_value))

        return int(round(pwm_value))

    def speed_to_pwm(self, speed_mps):
        """
        Convert speed in m/s to PWM value (0-255).

        Args:
            speed_mps: Speed in meters per second

        Returns:
            int: PWM value (0-255)
        """
        # Clamp speed to valid range
        speed_mps = max(-self.max_speed, min(self.max_speed, speed_mps))

        # Convert to normalized value (-1 to 1)
        normalized_speed = speed_mps / self.max_speed

        # Convert to PWM range
        if normalized_speed >= 0:
            # Forward motion
            pwm_range = self.motor_pwm_max - self.motor_pwm_stop
            pwm_value = self.motor_pwm_stop + (normalized_speed * pwm_range)
        else:
            # Reverse motion
            pwm_range = self.motor_pwm_stop - self.motor_pwm_min
            pwm_value = self.motor_pwm_stop + (normalized_speed * pwm_range)

        # Ensure PWM is within bounds and return as integer
        pwm_value = max(self.motor_pwm_min, min(self.motor_pwm_max, pwm_value))

        return int(round(pwm_value))

    def publish_neutral_command(self):
        """Publish neutral/stop PWM values."""
        self.pwm_array.data[self.steering_channel] = self.steering_pwm_neutral
        self.pwm_array.data[self.motor_channel] = self.motor_pwm_stop
        self.pwm_publisher.publish(self.pwm_array)
        self.get_logger().info('Published neutral PWM command')

    def get_pwm_values(self):
        """
        Get current PWM values for debugging.

        Returns:
            dict: Current PWM values
        """
        return {
            'steering_pwm': self.pwm_array.data[self.steering_channel],
            'motor_pwm': self.pwm_array.data[self.motor_channel],
            'steering_channel': self.steering_channel,
            'motor_channel': self.motor_channel
        }


def main(args=None):
    """Main function to run the PWM conversion node."""
    rclpy.init(args=args)
    node = PwmConversionNode()

    try:
        # Publish initial neutral command
        node.publish_neutral_command()

        rclpy.spin(node)
    except KeyboardInterrupt:
        # Publish neutral command on shutdown
        node.publish_neutral_command()
        node.get_logger().info('PWM Conversion Node shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()