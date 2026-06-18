#!/usr/bin/env python3
"""Node that converts web commands into navigation goals.

Subscribes to: `controlweb` (std_msgs/String) -- commands from web server
Publishes to: `goal_pose` (geometry_msgs/PoseStamped) -- goal for navigation

Supported commands (case-sensitive):
- "dock" -> Docking position
- "room1" -> Room 1
- "room2" -> Room 2
- "room3" -> Room 3
- "toilet" -> Toilet

Adjust the coordinates in `LOCATION_POSES` for your map.
"""
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import subprocess
import sys
import os

import firebase_admin
from firebase_admin import credentials, firestore

def make_pose_stamped(x, y, yaw=0.0, frame_id='map'):
    """Create a PoseStamped with position (x,y) and heading `yaw` (radians).

    Yaw is rotation around the Z axis; quaternion is computed from yaw.
    """
    from math import sin, cos

    ps = PoseStamped()
    ps.header.frame_id = frame_id
    ps.pose = Pose()
    ps.pose.position = Point(x=x, y=y, z=0.0)
    half = yaw * 0.5
    qz = sin(half)
    qw = cos(half)
    ps.pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)
    return ps


class GoalFromWeb(Node):
    def __init__(self):
        super().__init__('goal_from_web')

        # Name of the input and output topics can be changed via parameters later
        self.declare_parameter('input_topic', 'controlweb')
        self.declare_parameter('output_topic', 'goal_pose')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        cred = credentials.Certificate(
            "/home/kisali/fyp_ws/src/firebase_bridge/config/serviceAccountKey.json"
        )

        if not firebase_admin._apps:
            firebase_admin.initialize_app(cred)

        self.db = firestore.client()
        
        self.subscription = self.create_subscription(
            String,
            input_topic,
            self.control_callback,
            10)

        self.publisher = self.create_publisher(PoseStamped, output_topic, 10)

        # Process handle for person tracker when started via 'follow'
        self.person_proc = None

        self.get_logger().info(f'Listening on "{input_topic}", publishing goals to "{output_topic}"')

    def control_callback(self, msg: String):
        cmd = msg.data.strip()
        # Expect exact commands: 'dock', 'room1', 'room2', 'room3', 'toilet'
        key = cmd.lower()

        # Commands that map to locations
        location_commands = {'dock', 'room1', 'room2', 'room3', 'toilet'}
        stop_commands = {'stop_follow', 'stopfollow', 'stop'}

        # If person tracker is running, ignore all commands except stop commands
        if self.person_proc is not None and self.person_proc.poll() is None:
            if key in stop_commands:
                # allow stop to proceed
                pass
            elif key == 'follow':
                self.get_logger().info('person_tracker_node already running')
                return
            else:
                self.get_logger().info(f'person_tracker_node running; ignoring command "{msg.data}"')
                return

        # Special command to start the person tracker
        if key == 'follow':
            # Start person_tracker_node.py as a separate process to avoid rclpy conflicts
            if self.person_proc is not None and self.person_proc.poll() is None:
                self.get_logger().info('person_tracker_node already running')
                return

            node_path = os.path.join(os.path.dirname(__file__), 'person_tracker_node.py')
            if not os.path.exists(node_path):
                self.get_logger().error(f'person_tracker_node.py not found at {node_path}')
                return

            try:
                # Launch using the same Python interpreter
                self.person_proc = subprocess.Popen([sys.executable, node_path])
                self.get_logger().info(f'Started person_tracker_node (pid={self.person_proc.pid})')
            except Exception as e:
                self.get_logger().error(f'Failed to start person_tracker_node: {e}')
            return

        # Special command to stop the person tracker
        if key == 'stop_follow' or key == 'stopfollow' or key == 'stop':
            if self.person_proc is None or self.person_proc.poll() is not None:
                self.get_logger().info('person_tracker_node not running')
                return
            try:
                self.person_proc.terminate()
                self.person_proc.wait(timeout=5)
                self.get_logger().info('person_tracker_node terminated')
            except Exception:
                try:
                    self.person_proc.kill()
                    self.get_logger().info('person_tracker_node killed')
                except Exception as e:
                    self.get_logger().error(f'Failed to stop person_tracker_node: {e}')
            finally:
                self.person_proc = None
            return

        if key not in location_commands:
            self.get_logger().warn(f'Unknown command received: "{msg.data}"; expected one of {sorted(location_commands)} or follow/stop_follow')
            return

        room_doc = self.db.collection("rooms").document(key).get()

        if not room_doc.exists:
            self.get_logger().warn(
                f"Room '{key}' not found"
            )
            return

        room = room_doc.to_dict()

        x = room["x"]
        y = room["y"]

        pose = make_pose_stamped(x, y)

        # Stamp and publish
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = pose.header.frame_id
        out.pose = pose.pose

        self.publisher.publish(out)
        self.get_logger().info(f'Published goal for "{msg.data}" -> {out.header.frame_id} ({out.pose.position.x}, {out.pose.position.y})')


def main(args=None):
    rclpy.init(args=args)
    node = GoalFromWeb()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()