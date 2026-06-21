#!/usr/bin/env python3
"""Camera trigger node.

Subscribes to `goal_sent` (PoseStamped) and `goal_status` (String).
When a succeeded status is received for a goal whose position matches the
configured `room3` coordinates (within tolerance), the node will send a
serial command to an Arduino to move a servo (e.g. to 180 degrees) and
publish a `camera_stream` notification message.
"""
import threading
import time
from math import hypot

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

try:
    import serial
except Exception:
    serial = None


class CameraTrigger(Node):
    def __init__(self):
        super().__init__('camera_trigger')

        # Room3 coordinates and tolerance
        self.declare_parameter('room3_x', 0.0)
        self.declare_parameter('room3_y', 0.0)
        self.declare_parameter('room3_tol', 0.5)

        # Arduino serial params
        self.declare_parameter('arduino_port', '/dev/ttyACM1')
        self.declare_parameter('arduino_baud', 115200)
        self.declare_parameter('servo_angle', 180)
        self.declare_parameter('servo_repeat', 1)
        self.declare_parameter('servo_interval', 0.1)

        # Camera stream topic to notify when to start streaming
        self.declare_parameter('camera_topic', 'camera_stream')
        # Camera capture / Firebase upload params
        self.declare_parameter('camera_mode', 'device')  # 'device' or future 'topic'
        self.declare_parameter('camera_device', 0)
        self.declare_parameter('capture_frames', 10)
        self.declare_parameter('capture_interval', 0.5)
        self.declare_parameter('firebase_upload', True)
        self.declare_parameter('firebase_cred', 'home/kisali/fyp_ws/src/firebase_bridge/config/serviceAccountKey.json')
        self.declare_parameter('firebase_bucket', 'smart-wheelchair-91084.firebasestorage.app')

        self.room3_x = float(self.get_parameter('room3_x').get_parameter_value().double_value)
        self.room3_y = float(self.get_parameter('room3_y').get_parameter_value().double_value)
        self.room3_tol = float(self.get_parameter('room3_tol').get_parameter_value().double_value)

        self.arduino_port = self.get_parameter('arduino_port').get_parameter_value().string_value
        self.arduino_baud = int(self.get_parameter('arduino_baud').get_parameter_value().integer_value)
        self.servo_angle = int(self.get_parameter('servo_angle').get_parameter_value().integer_value)
        self.servo_repeat = int(self.get_parameter('servo_repeat').get_parameter_value().integer_value)
        self.servo_interval = float(self.get_parameter('servo_interval').get_parameter_value().double_value)

        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.camera_mode = self.get_parameter('camera_mode').get_parameter_value().string_value
        self.camera_device = int(self.get_parameter('camera_device').get_parameter_value().integer_value)
        self.capture_frames = int(self.get_parameter('capture_frames').get_parameter_value().integer_value)
        self.capture_interval = float(self.get_parameter('capture_interval').get_parameter_value().double_value)
        self.firebase_upload = bool(self.get_parameter('firebase_upload').get_parameter_value().bool_value)
        self.firebase_cred = self.get_parameter('firebase_cred').get_parameter_value().string_value
        self.firebase_bucket = self.get_parameter('firebase_bucket').get_parameter_value().string_value

        # Subscribers and publishers
        self.goal_sub = self.create_subscription(PoseStamped, 'goal_sent', self.goal_cb, 10)
        self.status_sub = self.create_subscription(String, 'goal_status', self.status_cb, 10)
        self.camera_pub = self.create_publisher(String, self.camera_topic, 10)

        # State
        self.last_goal = None
        self.triggered = False

        self.get_logger().info('CameraTrigger node ready')

    def goal_cb(self, msg: PoseStamped):
        # store last goal
        self.last_goal = msg

    def status_cb(self, msg: String):
        data = msg.data.lower() if msg.data is not None else ''
        if data == 'room3':
            if self.triggered:
                self.get_logger().info('Room3 already triggered')
                return
            self.get_logger().info('Room3 status received, triggering camera/servo')
            self.triggered = True
            threading.Thread(target=self._trigger_actions, daemon=True).start()
            return

        # consider 'succeeded' as arrival
        if 'succeeded' in data or data.startswith('arriv') or 'reached' in data:
            if self.last_goal is None:
                self.get_logger().warn('Received success status but no last goal stored')
                return

            x = float(self.last_goal.pose.position.x)
            y = float(self.last_goal.pose.position.y)
            dist = hypot(x - self.room3_x, y - self.room3_y)
            self.get_logger().info(f'Goal succeeded at ({x:.2f},{y:.2f}), dist to room3={dist:.2f}')
            if dist <= self.room3_tol and not self.triggered:
                self.get_logger().info('Triggering camera/servo for room3')
                self.triggered = True
                threading.Thread(target=self._trigger_actions, daemon=True).start()
            else:
                self.get_logger().info('Not a match for room3 or already triggered')

    def _trigger_actions(self):
        # 1) Send serial commands to Arduino if available
        if serial is None:
            self.get_logger().warning('pyserial not available; skipping Arduino commands')
        else:
            try:
                with serial.Serial(self.arduino_port, self.arduino_baud, timeout=1) as ser:
                    # send exact command like 'SERVO180' followed by newline
                    cmd = f'SERVO{self.servo_angle}\n'
                    for i in range(self.servo_repeat):
                        ser.write(cmd.encode('utf-8'))
                        ser.flush()
                        time.sleep(self.servo_interval)
                    self.get_logger().info(f'Sent servo command {self.servo_angle} to {self.arduino_port}')
            except Exception as e:
                self.get_logger().error(f'Failed to send serial to Arduino: {e}')

        # 2) Publish a camera stream notification
        try:
            msg = String()
            msg.data = 'start'
            self.camera_pub.publish(msg)
            self.get_logger().info(f'Published camera stream start on {self.camera_topic}')
        except Exception as e:
            self.get_logger().error(f'Failed to publish camera stream message: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraTrigger()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
