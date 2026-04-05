#!/usr/bin/env python3
"""
Navigation Test Script

This script demonstrates how to send goals to the navigation system
and monitor the robot's progress.

Usage:
    python3 navigation_test.py [x] [y] [orientation]

Example:
    python3 navigation_test.py 2.0 3.0 0.0  # Go to (2, 3) facing forward

Author: ROS2 Navigation Team
License: BSD-3-Clause
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import sys
import time


class NavigationTester(Node):
    """Simple node to test navigation by sending goals."""

    def __init__(self):
        super().__init__('navigation_tester')

        # Create publisher for goal coordinates
        self.goal_publisher = self.create_publisher(
            Float32MultiArray,
            '/goal_xy',
            10
        )

        self.get_logger().info('Navigation Tester initialized')

    def send_goal(self, x, y, orientation=0.0):
        """
        Send a navigation goal.

        Args:
            x: Goal x coordinate in map frame
            y: Goal y coordinate in map frame
            orientation: Goal orientation in radians (optional)
        """
        # Create goal message
        goal_msg = Float32MultiArray()
        goal_msg.data = [float(x), float(y), float(orientation)]

        # Publish goal
        self.goal_publisher.publish(goal_msg)

        self.get_logger().info('.2f')


def main(args=None):
    """Main function."""
    rclpy.init(args=args)

    if len(sys.argv) < 3:
        print("Usage: python3 navigation_test.py <x> <y> [orientation]")
        print("Example: python3 navigation_test.py 2.0 3.0 0.0")
        return

    # Parse arguments
    x = float(sys.argv[1])
    y = float(sys.argv[2])
    orientation = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0

    # Create tester node
    tester = NavigationTester()

    try:
        # Send goal
        tester.send_goal(x, y, orientation)

        # Keep node alive briefly to ensure message is sent
        time.sleep(1.0)

    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()