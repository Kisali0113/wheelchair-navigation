#!/usr/bin/env python3
"""Simple bridge node: subscribes to a PoseStamped and sends it to Nav2.

Subscribes to: `/target_person/pose` (geometry_msgs/PoseStamped)
Uses: `nav2_simple_commander.BasicNavigator` to send goals to Nav2

Adjust `input_topic` parameter if your topic differs.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import threading
import time
import tf2_ros
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_pose

try:
    from nav2_simple_commander.robot_navigator import BasicNavigator
except Exception:
    BasicNavigator = None


class GoalBridge(Node):
    def __init__(self):
        super().__init__('goal_bridge')

        self.declare_parameter('input_topic', '/target_person/pose')
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value

        if BasicNavigator is None:
            self.get_logger().error('nav2_simple_commander not available. Install nav2_simple_commander.')
            raise RuntimeError('nav2_simple_commander not available')

        self.navigator = BasicNavigator()

        # TF buffer for transforming incoming poses to 'map' frame
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(
            PoseStamped,
            input_topic,
            self.callback,
            10
        )

        # Publish navigation result status when goal completes
        self.status_pub = self.create_publisher(String, 'goal_status', 10)
        # Publish the transformed goal when it's sent (for external bridges)
        self.goal_sent_pub = self.create_publisher(PoseStamped, 'goal_sent', 10)

        self.get_logger().info(f'GoalBridge listening on "{input_topic}"')

    def callback(self, msg: PoseStamped):
        self.get_logger().info('Received pose; transforming to map frame')

        # Transform incoming pose into 'map' frame expected by Nav2
        target_frame = 'map'
        transformed_goal = None
        try:
            # Try transform at the pose timestamp first
            tf_stamped = self.tf_buffer.lookup_transform(
                target_frame,
                msg.header.frame_id,
                msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            transformed = do_transform_pose(msg, tf_stamped)
            transformed_goal = transformed
        except TransformException as e:
            # Fallback to latest transform
            self.get_logger().warn(f'TF lookup at stamp failed: {e}; trying latest')
            try:
                tf_stamped = self.tf_buffer.lookup_transform(
                    target_frame,
                    msg.header.frame_id,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5)
                )
                transformed = do_transform_pose(msg, tf_stamped)
                transformed_goal = transformed
            except TransformException as e2:
                self.get_logger().error(f'Failed to transform pose to {target_frame}: {e2}')
                return

        # BasicNavigator.goToPose accepts a PoseStamped
        try:
            # Publish the transformed goal to Nav2
            self.navigator.goToPose(transformed_goal)
            self.get_logger().info('Transformed goal sent to Nav2')
            # publish the transformed pose for external consumers (e.g., MQTT bridge)
            try:
                self.goal_sent_pub.publish(transformed_goal)
            except Exception:
                pass

            # Start a watcher thread to wait for completion and publish status
            def watch():
                # Wait until navigator reports completion if API available
                try:
                    while hasattr(self.navigator, 'isTaskComplete') and not self.navigator.isTaskComplete():
                        time.sleep(0.5)

                    result = None
                    if hasattr(self.navigator, 'getResult'):
                        result = self.navigator.getResult()

                    status_msg = String()
                    if result is None:
                        status_msg.data = 'sent'
                    else:
                        # Commonly 0==succeeded in nav2_simple_commander; otherwise publish raw result
                        try:
                            status_msg.data = 'succeeded' if int(result) == 0 else f'failed:{result}'
                        except Exception:
                            status_msg.data = str(result)

                    self.status_pub.publish(status_msg)
                    self.get_logger().info(f'Published navigation status: {status_msg.data}')
                except Exception as e:
                    self.get_logger().error(f'Watcher thread error: {e}')

            threading.Thread(target=watch, daemon=True).start()
        except Exception as e:
            self.get_logger().error(f'Failed to send goal: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GoalBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()