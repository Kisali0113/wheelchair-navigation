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

        self.goal_pub = self.create_publisher(
            PoseStamped,
            'goal_pose',
            10
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

            self.publish_goal(x, y)

    def publish_goal(self, x, y):

        goal = PoseStamped()

        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"

        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.position.z = 0.0

        goal.pose.orientation.w = 1.0

        self.goal_pub.publish(goal)

        self.get_logger().info(
            f"Published goal ({x}, {y})"
        )
        

def main(args=None):

    rclpy.init(args=args)

    node = FirebaseListener()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()