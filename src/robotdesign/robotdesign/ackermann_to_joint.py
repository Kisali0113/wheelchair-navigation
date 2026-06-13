#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64


class AckermannToJoint(Node):

    def __init__(self):
        super().__init__('ackermann_to_joint')

        # Parameters
        self.declare_parameter('wheel_radius', 0.1)

        self.wheel_radius = self.get_parameter(
            'wheel_radius').get_parameter_value().double_value

        # Subscriber
        self.sub = self.create_subscription(
            AckermannDriveStamped,
            '/ackermann_cmd',
            self.callback,
            10
        )

        # Publishers
        self.steering_pub = self.create_publisher(
            Float64,
            '/joint/steering_joint/cmd_pos',
            10
        )

        self.wheel_pub = self.create_publisher(
            Float64,
            '/joint/front_wheel_joint/cmd_vel',
            10
        )

        self.get_logger().info("Ackermann → Joint converter started")

    def callback(self, msg):

        steering = msg.drive.steering_angle
        speed = msg.drive.speed

        # Convert linear speed → angular velocity
        wheel_velocity = speed / self.wheel_radius

        # Publish steering
        steering_msg = Float64()
        steering_msg.data = steering

        # Publish wheel velocity
        wheel_msg = Float64()
        wheel_msg.data = wheel_velocity

        self.steering_pub.publish(steering_msg)
        self.wheel_pub.publish(wheel_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AckermannToJoint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
