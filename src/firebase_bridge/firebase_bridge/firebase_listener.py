#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import subprocess
import signal

# from nav2_msgs.action import NavigateToPose
# from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

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
        self.camera_process = None
        self.get_logger().info("Firebase listener node started")

        cred = credentials.Certificate(
            '/home/kisali/fyp_ws/src/firebase_bridge/config/serviceAccountKey.json'
        )

        self.goal_pub = self.create_publisher(
            PoseStamped,
            'goal_pose',
            10
        )

        self.request_pub = self.create_publisher(
            String,
            '/active_request_id',
            10
        )

        self.follow_pub = self.create_publisher(
            String,
            "/caregiver_following",
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/active_request_status',
            10
        )

        if not firebase_admin._apps:
            firebase_admin.initialize_app(cred)
    # def watch_camera_control(self):

    #     control_ref = (
    #         self.db
    #         .collection("system")
    #         .document("control")
    #     )
        self.db = firestore.client()

        self.watch_requests()
        # self.watch_camera_control()

    def watch_requests(self):
        self.get_logger().info("inside watch_requests()")

        requests_ref = self.db.collection('requests')

        requests_ref.on_snapshot(self.on_snapshot)
    
    # def watch_camera_control(self):

    #     control_ref = (
    #         self.db
    #         .collection("system")
    #         .document("control")
    #     )

        # control_ref.on_snapshot(
        #     self.on_camera_control
        # )

        # self.get_logger().info(
        #     "Camera control listener started"
        # )

    def on_snapshot(self, docs, changes, read_time):
        self.get_logger().info("inside on snapshot()")

        for doc in docs:
            data = doc.to_dict()

            # camera_active = data.get("camera_active")

            # if camera_active is True:
            #     self.start_camera_server()

            # elif camera_active is False:
            #     self.stop_camera_server()

            status = data.get("status")

            self.get_logger().info(f"Request {doc.id} status={status}")
            
            # ---------- PICKUP ----------
            if status == "assigned" or status == "pickup_in_transit":

                pickup_name = data.get("location")

                self.get_logger().info(f"Pickup = {pickup_name}")

                if pickup_name is None:
                    continue

                room_docs = (
                    self.db.collection("rooms")
                    .where("name", "==", pickup_name)
                    .limit(1)
                    .stream()
                )
                room = next(room_docs, None)

                if room is None:
                    self.get_logger().error(
                        f"Room not found: {pickup_name}"
                    )
                    continue

                room_data = room.to_dict()

                x = room_data["x"]
                y = room_data["y"]

            # ---------- DESTINATION ----------
            elif status == "destination_in_transit":
                
                destination = data.get("destination")

                self.get_logger().info(f"Destination = {destination}")

                if destination is None:
                    continue
                
                x = destination["x"]
                y = destination["y"]

             # ---------- RETURN TO DOCK ----------
            elif status == "completed":

                x = 1.64867830276489
                y = -6.20749378204346

            else:
                continue

            # Avoid duplicate goal publication
            goal_key = f"{doc.id}_{status}"

            if self.last_request_id == goal_key:
                continue

            self.last_request_id = goal_key

            self.current_request_id = doc.id
            self.current_status = status

            self.get_logger().info(f"Navigate to ({x}, {y})")
         
            request_msg = String()
            request_msg.data = doc.id

            status_msg = String()
            status_msg.data = status

            self.request_pub.publish(request_msg)

            self.status_pub.publish(status_msg)

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
        
    # def on_camera_control(self, docs, changes, read_time):

    #     self.get_logger().info("Camera listener triggered")

    #     for doc in docs:

    #         data = doc.to_dict()

    #         self.get_logger().info(f"Data = {data}")

    #         camera_active = data.get("camera_active", False)
    #         follow_enabled = data.get("caregiver_following", False)

    #         if camera_active:
    #             self.get_logger().info("Starting camera server")
    #             self.start_camera_server()
    #         else:
    #             self.get_logger().info("Stopping camera server")
    #             self.stop_camera_server()

    #         if follow_enabled:
    #             self.get_logger().info("Caregiver following enabled")
    #             msg = String()
    #             msg.data = "START"
    #             self.status_pub.publish(msg)
    #         else:
    #             self.get_logger().info("Caregiver following disabled")
    #             msg = String()
    #             msg.data = "STOP"
    #             self.status_pub.publish(msg)
    
    # def start_camera_server(self):

    #     if self.camera_process is not None:
    #         self.get_logger().info(
    #             "web_video_server already running"
    #         )
    #         return

    #     try:

    #         self.camera_process = subprocess.Popen(
    #             [
    #                 "bash",
    #                 "-c",
    #                 "source /opt/ros/jazzy/setup.bash && ros2 run web_video_server web_video_server"
    #             ]
    #         )

    #         self.get_logger().info(
    #             "Started web_video_server"
    #         )

    #     except Exception as e:

    #         self.get_logger().error(
    #             f"Failed to start web_video_server: {e}"
    #         )

    # def stop_camera_server(self):

    #     if self.camera_process is None:
    #         return

    #     self.camera_process.terminate()
    #     self.camera_process.wait()

    #     self.camera_process = None

    #     self.get_logger().info(
    #         "Stopped web_video_server"
    #     )

def main(args=None):

    rclpy.init(args=args)

    node = FirebaseListener()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()