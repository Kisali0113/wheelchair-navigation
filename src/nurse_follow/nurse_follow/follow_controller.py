#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist


class FollowController(Node):

    def __init__(self):
        super().__init__('follow_controller')

        self.sub = self.create_subscription(
            PointStamped,
            '/person_target',
            self.target_callback,
            10
        )

        self.pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.desired_distance = 1.5

    def target_callback(self, msg):

        target_x = msg.point.x
        target_z = msg.point.z

        distance_error = target_z - self.desired_distance

        linear = 0.5 * distance_error
        angular = -1.2 * target_x

        linear = max(-0.3, min(0.3, linear))
        angular = max(-0.8, min(0.8, angular))

        if target_z < 0.8:
            linear = 0.0

        cmd = Twist()

        cmd.linear.x = linear
        cmd.angular.z = angular

        self.pub.publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    node = FollowController()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()