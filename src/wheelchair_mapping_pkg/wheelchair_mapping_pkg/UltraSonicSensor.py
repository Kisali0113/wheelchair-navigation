#!/usr/bin/env python3
"""Map ultrasonic Range readings into map coordinates.

Subscribes to:
- /ultrasonic/front (sensor_msgs/Range)
- /ultrasonic/left
- /ultrasonic/right
- odom topic (nav_msgs/Odometry) to get robot pose (Xrobot,Yrobot,theta_robot)

Publishes:
- /obstacle/front (geometry_msgs/PointStamped)
- /obstacle/left
- /obstacle/right
"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped


def _yaw_from_quaternion(q):
    """Extract yaw angle from a geometry_msgs/Quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class UltrasonicMapper(Node):
    def __init__(self):
        super().__init__('ultrasonic_mapper')

        # Declare parameters with your exact physical dimensions (Converted to meters)
        self.declare_parameter('odom_topic', '/wheel/odom')
        self.declare_parameter('use_single_topic', False)
        self.declare_parameter('ultrasonic_topic', '/ultrasonic')

        # Front sensor: 107 cm straight ahead
        self.declare_parameter('front_x', 1.07)
        self.declare_parameter('front_y', 0.0)
        self.declare_parameter('front_theta', 0.0)

        # Left sensor: 45 cm forward, 31 cm left, 30 deg Counter-Clockwise
        self.declare_parameter('left_x', 0.45)
        self.declare_parameter('left_y', 0.31)
        self.declare_parameter('left_theta', math.radians(30.0))  

        # Right sensor: 45 cm forward, 31 cm right, 30 deg Clockwise
        self.declare_parameter('right_x', 0.45)
        self.declare_parameter('right_y', -0.31)
        self.declare_parameter('right_theta', math.radians(-30.0)) 

        # Fetch parameter values
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.use_single = self.get_parameter('use_single_topic').get_parameter_value().bool_value
        self.ultrasonic_topic = self.get_parameter('ultrasonic_topic').get_parameter_value().string_value

        self.front_x = self.get_parameter('front_x').get_parameter_value().double_value
        self.front_y = self.get_parameter('front_y').get_parameter_value().double_value
        self.front_theta = self.get_parameter('front_theta').get_parameter_value().double_value

        self.left_x = self.get_parameter('left_x').get_parameter_value().double_value
        self.left_y = self.get_parameter('left_y').get_parameter_value().double_value
        self.left_theta = self.get_parameter('left_theta').get_parameter_value().double_value

        self.right_x = self.get_parameter('right_x').get_parameter_value().double_value
        self.right_y = self.get_parameter('right_y').get_parameter_value().double_value
        self.right_theta = self.get_parameter('right_theta').get_parameter_value().double_value

        # Publishers
        self.pub_front = self.create_publisher(PointStamped, '/obstacle/front', 10)
        self.pub_left = self.create_publisher(PointStamped, '/obstacle/left', 10)
        self.pub_right = self.create_publisher(PointStamped, '/obstacle/right', 10)

        # Cached robot pose
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # Subscriptions
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)

        if self.use_single:
            self.create_subscription(Range, self.ultrasonic_topic, self.multiplex_cb, 10)
            self.get_logger().info(f'Subscribed to single multiplexed topic: {self.ultrasonic_topic}')
        else:
            self.front_sub = self.create_subscription(Range, '/ultrasonic/front', self.front_cb, 10)
            self.left_sub = self.create_subscription(Range, '/ultrasonic/left', self.left_cb, 10)
            self.right_sub = self.create_subscription(Range, '/ultrasonic/right', self.right_cb, 10)
            self.get_logger().info('Subscribed to individual per-sensor topics.')

        self.get_logger().info('Ultrasonic mapper fully initialized with wheelchair dimensions.')

    def odom_cb(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_yaw = _yaw_from_quaternion(msg.pose.pose.orientation)

    def _compute_obstacle(self, d, xs, ys, theta_sensor):
        if d <= 0.0 or math.isinf(d) or math.isnan(d):
            return None

        # 1. Map obstacle location relative to robot base frame (base_link)
        x_robot_frame = xs + (d * math.cos(theta_sensor))
        y_robot_frame = ys + (d * math.sin(theta_sensor))

        # 2. Project into Global Map coordinates using the current robot location/heading
        cos_yaw = math.cos(self.robot_yaw)
        sin_yaw = math.sin(self.robot_yaw)
        
        Xobs = self.robot_x + (x_robot_frame * cos_yaw - y_robot_frame * sin_yaw)
        Yobs = self.robot_y + (x_robot_frame * sin_yaw + y_robot_frame * cos_yaw)

        return Xobs, Yobs

    def _publish_point(self, pub, x, y):
        pt = PointStamped()
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.header.frame_id = 'map'
        pt.point.x = float(x)
        pt.point.y = float(y)
        pt.point.z = 0.0
        pub.publish(pt)

    def front_cb(self, msg: Range):
        res = self._compute_obstacle(msg.range, self.front_x, self.front_y, self.front_theta)
        if res is not None:
            self._publish_point(self.pub_front, res[0], res[1])

    def left_cb(self, msg: Range):
        res = self._compute_obstacle(msg.range, self.left_x, self.left_y, self.left_theta)
        if res is not None:
            self._publish_point(self.pub_left, res[0], res[1])

    def right_cb(self, msg: Range):
        res = self._compute_obstacle(msg.range, self.right_x, self.right_y, self.right_theta)
        if res is not None:
            self._publish_point(self.pub_right, res[0], res[1])

    def multiplex_cb(self, msg: Range):
        frame = msg.header.frame_id.lower() if msg.header.frame_id else ''
        if 'front' in frame:
            self.front_cb(msg)
        elif 'left' in frame:
            self.left_cb(msg)
        elif 'right' in frame:
            self.right_cb(msg)
        else:
            self.get_logger().warning(f"Unknown frame_id: '{msg.header.frame_id}'", throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()