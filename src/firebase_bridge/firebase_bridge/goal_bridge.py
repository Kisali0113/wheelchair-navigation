#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped

from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import String

import firebase_admin
from firebase_admin import credentials
from firebase_admin import firestore


class GoalBridge(Node):

    def __init__(self):

        super().__init__('goal_bridge')

        cred = credentials.Certificate(
            '/home/kisali/fyp_ws/src/firebase_bridge/config/serviceAccountKey.json'
        )

        if not firebase_admin._apps:
            firebase_admin.initialize_app(cred)

        self.db = firestore.client()

        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
        self.goal_sent_pub = self.create_publisher(PoseStamped, 'goal_sent', 10)
        self.status_pub = self.create_publisher(String, 'goal_status', 10)
        self.current_request_id = None
        self.current_status = None

        self.request_sub = self.create_subscription(
            String,
            '/active_request_id',
            self.request_callback,
            10
        )

        self.status_sub = self.create_subscription(
            String,
            '/active_request_status',
            self.status_callback,
            10
        )

        self.get_logger().info("Goal Bridge Started")

    def goal_callback(self, msg):

        self.get_logger().info(
            f"Received goal: "
            f"({msg.pose.position.x}, "
            f"{msg.pose.position.y})"
        )

        if not self.nav_client.wait_for_server(
            timeout_sec=5.0
        ):

            self.get_logger().error(
                "Nav2 server not available"
            )

            return

        goal_msg = NavigateToPose.Goal()

        goal_msg.pose = msg

        self.goal_sent_pub.publish(msg)

        future = self.nav_client.send_goal_async(
            goal_msg
        )

        future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(
        self,
        future
    ):

        goal_handle = future.result()

        if not goal_handle.accepted:

            self.get_logger().error(
                "Goal rejected"
            )

            return

        self.get_logger().info(
            "Goal accepted"
        )

        result_future = (
            goal_handle.get_result_async()
        )

        result_future.add_done_callback(
            self.result_callback
        )

    def result_callback(self, future):

        result = future.result()

        self.get_logger().info(f"Navigation finished status={result.status}")

        status_msg = String()
        status_msg.data = 'succeeded' if result.status == 4 else f'failed:{result.status}'
        self.status_pub.publish(status_msg)

        # STATUS_SUCCEEDED = 4
        if result.status != 4:

            self.get_logger().warn("Navigation failed")

            return

        self.get_logger().info("Goal reached!")

        if self.current_request_id is None:

            self.get_logger().warn("No request id available")

            return

        next_status = None

        if self.current_status == "pickup":
            next_status = "boarding"

        elif self.current_status == "in_transit":
            next_status = "arrived"

        elif self.current_status == "returning":
            next_status = "docked"

        if next_status is None:
            return

        self.db.collection(
            "requests"
        ).document(
            self.current_request_id
        ).update({
            "status": next_status
        })

        self.get_logger().info(
            f"Updated request "
            f"{self.current_request_id} "
            f"to {next_status}"
        )


    def request_callback(self, msg):

        self.current_request_id = msg.data

        self.get_logger().info(
            f"Current request = {msg.data}"
        )

    def status_callback(self, msg):

        self.current_status = msg.data

        self.get_logger().info(
            f"Current request status = {msg.data}"
        )

def main(args=None):

    rclpy.init(args=args)

    node = GoalBridge()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()

# """Simple bridge node: subscribes to a PoseStamped and sends it to Nav2.

# Subscribes to: `/target_person/pose` (geometry_msgs/PoseStamped)
# Uses: `nav2_simple_commander.BasicNavigator` to send goals to Nav2

# Adjust `input_topic` parameter if your topic differs.
# """
# import rclpy
# from rclpy.node import Node

# from geometry_msgs.msg import PoseStamped

# from std_msgs.msg import String
# import threading
# import time
# import tf2_ros
# from tf2_ros import TransformException
# from tf2_geometry_msgs import do_transform_pose

# try:
#     from nav2_simple_commander.robot_navigator import BasicNavigator
# except Exception:
#     BasicNavigator = None


# class GoalBridge(Node):
#     def __init__(self):
#         super().__init__('goal_bridge')

#         self.declare_parameter('input_topic', '/target_person/pose')
#         input_topic = self.get_parameter('input_topic').get_parameter_value().string_value

#         if BasicNavigator is None:
#             self.get_logger().error('nav2_simple_commander not available. Install nav2_simple_commander.')
#             raise RuntimeError('nav2_simple_commander not available')

#         self.navigator = BasicNavigator()

#         # TF buffer for transforming incoming poses to 'map' frame
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

#         self.sub = self.create_subscription(
#             PoseStamped,
#             input_topic,
#             self.callback,
#             10
#         )

#         # Publish navigation result status when goal completes
#         self.status_pub = self.create_publisher(String, 'goal_status', 10)
#         # Publish the transformed goal when it's sent (for external bridges)
#         self.goal_sent_pub = self.create_publisher(PoseStamped, 'goal_sent', 10)

#         self.get_logger().info(f'GoalBridge listening on "{input_topic}"')

#     def callback(self, msg: PoseStamped):
#         self.get_logger().info('Received pose; transforming to map frame')

#         # Transform incoming pose into 'map' frame expected by Nav2
#         target_frame = 'map'
#         transformed_goal = None
#         try:
#             # Try transform at the pose timestamp first
#             tf_stamped = self.tf_buffer.lookup_transform(
#                 target_frame,
#                 msg.header.frame_id,
#                 msg.header.stamp,
#                 timeout=rclpy.duration.Duration(seconds=0.5)
#             )
#             transformed = do_transform_pose(msg, tf_stamped)
#             transformed_goal = transformed
#         except TransformException as e:
#             # Fallback to latest transform
#             self.get_logger().warn(f'TF lookup at stamp failed: {e}; trying latest')
#             try:
#                 tf_stamped = self.tf_buffer.lookup_transform(
#                     target_frame,
#                     msg.header.frame_id,
#                     rclpy.time.Time(),
#                     timeout=rclpy.duration.Duration(seconds=0.5)
#                 )
#                 transformed = do_transform_pose(msg, tf_stamped)
#                 transformed_goal = transformed
#             except TransformException as e2:
#                 self.get_logger().error(f'Failed to transform pose to {target_frame}: {e2}')
#                 return

#         # BasicNavigator.goToPose accepts a PoseStamped
#         try:
#             # Publish the transformed goal to Nav2
#             self.navigator.goToPose(transformed_goal)
#             self.get_logger().info('Transformed goal sent to Nav2')
#             # publish the transformed pose for external consumers (e.g., MQTT bridge)
#             try:
#                 self.goal_sent_pub.publish(transformed_goal)
#             except Exception:
#                 pass

#             # Start a watcher thread to wait for completion and publish status
#             def watch():
#                 # Wait until navigator reports completion if API available
#                 try:
#                     while hasattr(self.navigator, 'isTaskComplete') and not self.navigator.isTaskComplete():
#                         # time.sleep(0.5)

#                     result = None
#                     if hasattr(self.navigator, 'getResult'):
#                         result = self.navigator.getResult()

#                     status_msg = String()
#                     if result is None:
#                         status_msg.data = 'sent'
#                     else:
#                         # Commonly 0==succeeded in nav2_simple_commander; otherwise publish raw result
#                         try:
#                             status_msg.data = 'succeeded' if int(result) == 0 else f'failed:{result}'
#                         except Exception:
#                             status_msg.data = str(result)

#                     self.status_pub.publish(status_msg)
#                     self.get_logger().info(f'Published navigation status: {status_msg.data}')
#                 except Exception as e:
#                     self.get_logger().error(f'Watcher thread error: {e}')

#             threading.Thread(target=watch, daemon=True).start()
#         except Exception as e:
#             self.get_logger().error(f'Failed to send goal: {e}')


# def main(args=None):
#     rclpy.init(args=args)
#     node = None
#     try:
#         node = GoalBridge()
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         if node is not None:
#             node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()