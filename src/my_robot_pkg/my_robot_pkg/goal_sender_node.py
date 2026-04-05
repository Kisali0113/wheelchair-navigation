#!/usr/bin/env python3
"""
Goal Sender Node for ROS2 Navigation

This node accepts goal coordinates (x, y) and publishes them as PoseStamped
messages to the Nav2 action server for path planning and execution.

Author: ROS2 Navigation Team
License: BSD-3-Clause
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Float32MultiArray
import math


class GoalSenderNode(Node):
    """
    ROS2 node that sends navigation goals to Nav2.

    This node subscribes to a goal topic and converts simple x,y coordinates
    into PoseStamped messages for Nav2 navigation.
    """

    def __init__(self):
        super().__init__('goal_sender_node')

        # Declare parameters
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('goal_xy_topic', '/goal_xy')
        self.declare_parameter('goal_pose_topic', '/goal_pose')
        self.declare_parameter('rviz_goal_topic', '/move_base_simple/goal')
        self.declare_parameter('default_orientation', 0.0)  # radians, forward facing

        # Get parameters
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.robot_frame = self.get_parameter('robot_frame').get_parameter_value().string_value
        goal_xy_topic = self.get_parameter('goal_xy_topic').get_parameter_value().string_value
        goal_pose_topic = self.get_parameter('goal_pose_topic').get_parameter_value().string_value
        rviz_goal_topic = self.get_parameter('rviz_goal_topic').get_parameter_value().string_value
        self.default_orientation = self.get_parameter('default_orientation').get_parameter_value().double_value

        # Create subscription for goal coordinates (x, y format)
        self.goal_xy_subscription = self.create_subscription(
            Float32MultiArray,
            goal_xy_topic,
            self.goal_xy_callback,
            10
        )

        # Create subscription for PoseStamped goals (from other nodes)
        self.goal_pose_subscription = self.create_subscription(
            PoseStamped,
            goal_pose_topic,
            self.goal_pose_callback,
            10
        )

        # Create subscription for RViz goals
        self.rviz_goal_subscription = self.create_subscription(
            PoseStamped,
            rviz_goal_topic,
            self.rviz_goal_callback,
            10
        )

        # Create publisher for PoseStamped goals (for visualization/debugging)
        self.goal_pose_publisher = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )

        # Create Nav2 action client
        self.nav_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        self.get_logger().info('Goal Sender Node initialized')
        self.get_logger().info(f'Subscribing to goal_xy topic: {goal_xy_topic}')
        self.get_logger().info(f'Subscribing to goal_pose topic: {goal_pose_topic}')
        self.get_logger().info(f'Subscribing to RViz goal topic: {rviz_goal_topic}')
        self.get_logger().info(f'Publishing to Nav2 action: navigate_to_pose')

    def goal_xy_callback(self, msg: Float32MultiArray):
        """
        Callback for goal coordinates in x,y format.

        Expects Float32MultiArray with at least 2 elements: [x, y]
        Optional 3rd element: orientation in radians
        """
        if len(msg.data) < 2:
            self.get_logger().error('Goal message must contain at least x and y coordinates')
            return

        x = msg.data[0]
        y = msg.data[1]

        # Use default orientation if not provided
        orientation_rad = self.default_orientation
        if len(msg.data) >= 3:
            orientation_rad = msg.data[2]

        self.get_logger().info('.2f')

        # Create PoseStamped message
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.map_frame
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        # Set position
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0

        # Set orientation (convert radians to quaternion)
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = math.sin(orientation_rad / 2.0)
        goal_pose.pose.orientation.w = math.cos(orientation_rad / 2.0)

        # Send goal to Nav2
        self.send_goal_to_nav2(goal_pose)

        # Publish PoseStamped for visualization
        self.goal_pose_publisher.publish(goal_pose)

    def goal_pose_callback(self, msg: PoseStamped):
        """
        Callback for PoseStamped goals (from other nodes).

        Args:
            msg: PoseStamped message with goal position and orientation
        """
        self.get_logger().info('.2f')

        # Ensure the goal is in the correct frame
        if msg.header.frame_id != self.map_frame:
            self.get_logger().warn(f'Goal frame {msg.header.frame_id} differs from map frame {self.map_frame}')
            # For now, assume it's already in the right frame
            # In production, you might want to transform it

        # Send goal to Nav2
        self.send_goal_to_nav2(msg)

        # Republish for visualization
        self.goal_pose_publisher.publish(msg)

    def rviz_goal_callback(self, msg: PoseStamped):
        """
        Callback for RViz goals.

        Args:
            msg: PoseStamped message from RViz goal tool
        """
        self.get_logger().info('.2f')

        # RViz goals are typically in map frame, but let's ensure
        goal_pose = PoseStamped()
        goal_pose.header = msg.header
        goal_pose.pose = msg.pose

        # Ensure frame_id is set to map
        if not goal_pose.header.frame_id:
            goal_pose.header.frame_id = self.map_frame

        # Send goal to Nav2
        self.send_goal_to_nav2(goal_pose)

        # Publish for visualization
        self.goal_pose_publisher.publish(goal_pose)

    def send_goal_to_nav2(self, goal_pose: PoseStamped):
        """
        Send navigation goal to Nav2 action server.

        Args:
            goal_pose: PoseStamped message with goal position and orientation
        """
        # Wait for action server to be available
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available')
            return

        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.get_logger().info('Sending goal to Nav2...')

        # Send goal asynchronously
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        # Add callback for goal response
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        Callback for goal acceptance/rejection.

        Args:
            future: Future object containing goal handle
        """
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2')
            return

        self.get_logger().info('Goal accepted by Nav2')

        # Get result asynchronously
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """
        Callback for navigation feedback.

        Args:
            feedback_msg: Navigation feedback message
        """
        # Optional: Log progress or publish feedback
        pass

    def get_result_callback(self, future):
        """
        Callback for navigation result.

        Args:
            future: Future object containing navigation result
        """
        result = future.result().result
        status = future.result().status

        if status == 4:  # SUCCEEDED
            self.get_logger().info('Navigation goal reached successfully!')
        elif status == 6:  # CANCELED
            self.get_logger().info('Navigation goal was canceled')
        elif status == 5:  # ABORTED
            self.get_logger().error('Navigation goal aborted')
        else:
            self.get_logger().warn(f'Navigation completed with status: {status}')


def main(args=None):
    """Main function to run the goal sender node."""
    rclpy.init(args=args)
    node = GoalSenderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Goal Sender Node shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()