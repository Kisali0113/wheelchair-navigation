#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped

import sys

print("PYTHON =", sys.executable)
print("PATHS =")
for p in sys.path:
    print(p)

import firebase_admin
from firebase_admin import credentials, firestore

class FirebaseListener(Node):

    def __init__(self):
        super().__init__('firebase_listener')

        self.last_request_id = None

        cred = credentials.Certificate(
            '/home/kisali/fyp_ws/src/firebase_bridge/config/serviceAccountKey.json'
        )

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        if not firebase_admin._apps:
            firebase_admin.initialize_app(cred)

        self.db = firestore.client()

        self.watch_requests()

    def watch_requests(self):

        requests_ref = self.db.collection('requests')

        requests_ref.on_snapshot(self.on_snapshot)

    def on_snapshot(self, docs, changes, read_time):

        for doc in docs:
            data = doc.to_dict()

            self.get_logger().info(
            f"Request {doc.id} status={data.get('status')}"
        )

            if data.get("status") != "in_transit":
                continue

            destination = data.get("destination")

            self.get_logger().info(f"Destination = {destination}")

            if destination is None:
                continue
            
            x = destination["x"]
            y = destination["y"]

            # Prevent duplicate goals
            if self.last_request_id == doc.id:
                continue

            self.last_request_id = doc.id
            self.current_request_id = doc.id

            self.get_logger().info(f"Navigate to ({x}, {y})")

            self.get_logger().info(f"Destination = {destination}")

            self.send_nav_goal(x, y)

    def send_nav_goal(self, x, y):
        
        self.get_logger().info("Inside send_nav_goal")
        goal_msg = NavigateToPose.Goal()

        goal_msg.pose.header.frame_id = "map"

        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)

        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info("Waiting for Nav2 server")
        self.nav_client.wait_for_server()
        self.get_logger().info("Nav2 server found")

        goal_future = self.nav_client.send_goal_async(goal_msg)

        goal_future.add_done_callback(
            self.goal_response_callback
        )
        self.get_logger().info("Calling send_nav_goal")
        self.get_logger().info(
            f"Goal sent ({x}, {y})"
    )
        
    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return

        self.get_logger().info("Goal accepted")

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            self.result_callback
        )

    def result_callback(self, future):

        result = future.result()

        self.get_logger().info(
            f"Nav2 result status: {result.status}"
        )

        # STATUS_SUCCEEDED = 4
        if result.status == 4:

            self.get_logger().info(
                "Destination reached!"
            )

            self.db.collection("requests").document(
                self.current_request_id
            ).update({
                "status": "arrived"
            })
        else:

            self.get_logger().warn(
                "Navigation failed"
            )

def main(args=None):

    rclpy.init(args=args)

    node = FirebaseListener()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()