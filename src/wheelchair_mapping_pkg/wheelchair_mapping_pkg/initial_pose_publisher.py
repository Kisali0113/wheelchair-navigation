#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class InitialPosePublisher(Node):

    def __init__(self):
        super().__init__('initial_pose_publisher')

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        self.timer = self.create_timer(
            3.0,
            self.publish_pose
        )

        self.sent = False

    def publish_pose(self):

        if self.sent:
            return

        msg = PoseWithCovarianceStamped()

        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = 1.64867830276489
        msg.pose.pose.position.y = -6.20749378204346

        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0

        self.publisher.publish(msg)

        self.get_logger().info(
            'Docking pose published'
        )

        self.sent = True


def main(args=None):

    rclpy.init(args=args)

    node = InitialPosePublisher()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()