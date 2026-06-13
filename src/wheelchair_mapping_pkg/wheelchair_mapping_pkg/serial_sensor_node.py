#!/usr/bin/env python3

import math
import serial
from serial import SerialException

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Float32, Float64MultiArray
from geometry_msgs.msg import Quaternion


class SerialSensorNode(Node):
    """Read Arduino serial data and publish wheel tick and IMU topics."""

    def __init__(self):
        super().__init__('serial_sensor_node')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('read_rate', 50.0)
        self.declare_parameter('imu_frame_id', 'imu_link')
        self.declare_parameter('heading_topic', '/mag_heading')
        
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.read_rate = self.get_parameter('read_rate').value
        self.imu_frame_id = self.get_parameter('imu_frame_id').value
        self.heading_topic = self.get_parameter('heading_topic').value

        # Core Movement Data Publishers
        self.steer_pub = self.create_publisher(Float32, '/steer_angle', 10)
        self.wheel_pub = self.create_publisher(Float32, '/wheel_speed', 10)
        self.heading_pub = self.create_publisher(Float32, self.heading_topic, 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        
        # Ultrasonic Range Publishers
        self.us_front_pub = self.create_publisher(Range, '/ultrasonic/front', 10)
        self.us_left_pub = self.create_publisher(Range, '/ultrasonic/left', 10)
        self.us_right_pub = self.create_publisher(Range, '/ultrasonic/right', 10)
        
        self.cmd_vel_sub = self.create_subscription(Float64MultiArray, '/cmd_vels', self.cmd_vel_callback, 10)

        self.serial_conn = None
        self.connect_serial()

        self.create_timer(1.0 / self.read_rate, self.read_serial_data)
        self.get_logger().info(f'Serial sensor node started on {self.serial_port}')

    def connect_serial(self) -> None:
        """Open the serial port and recover from disconnects."""
        if self.serial_conn is not None:
            try:
                self.serial_conn.close()
            except SerialException:
                pass

        try:
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=1.0,
            )
            self.serial_conn.reset_input_buffer()
            self.get_logger().info(f'Connected to serial port {self.serial_port}')
        except SerialException as exc:
            self.serial_conn = None
            self.get_logger().error(f'Unable to open serial port {self.serial_port}: {exc}')

    def read_serial_data(self) -> None:
        """Read one line from Arduino and publish topics when valid data arrives."""
        if self.serial_conn is None or not self.serial_conn.is_open:
            self.connect_serial()
            return
        
        try:
            if self.serial_conn.in_waiting == 0:
                return
            
            raw_line = self.serial_conn.readline()
            if not raw_line:
                return

            line = raw_line.decode('utf-8', errors='ignore').strip()
            if line:
                # If there are debug logs from printData mixed in, parse purely data lines
                self.parse_and_publish(line)
        except SerialException as exc:
            self.get_logger().error(f'Serial read error: {exc}')
            self.serial_conn = None
        except Exception as exc:
            self.get_logger().warn(f'Unexpected data error: {exc}')

    # def cmd_vel_callback(self, msg: Float64MultiArray) -> None:
    #     """Send received cmd_vels values to the Arduino over serial safely."""
    #     if len(msg.data) >= 2:
    #         try:
    #             linear_vel = float(msg.data[0])
    #             steer_ang = float(msg.data[1])

    #             command = f'{linear_vel:.2f},{steer_ang:.2f}\n'
    #             self.send_serial_command(command)
    #         except IndexError as exc:
    #             self.get_logger().warn(f'cmd_vels message missing fields: {exc}')
    #         except (TypeError, ValueError) as exc:
    #             self.get_logger().warn(f'cmd_vels parse error: {exc}')

    # def send_serial_command(self, command: str) -> None:
    #     if self.serial_conn is None or not self.serial_conn.is_open:
    #         self.connect_serial()
    #         if self.serial_conn is None:
    #             self.get_logger().error('Cannot send command: serial port not connected')
    #             return

    #     try:
    #         self.serial_conn.write(command.encode('utf-8'))
    #         self.get_logger().info(f'Sent command: {command.strip()}')
    #     except SerialException as exc:
    #         self.get_logger().error(f'Serial write error: {exc}')
    #         self.serial_conn = None
  
  
    def cmd_vel_callback(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 2:
            return

        linear_vel = float(msg.data[0])
        steer_ang = float(msg.data[1])

        command = "{:.2f},{:.2f}\n".format(linear_vel, steer_ang)

        self.send_serial_command(command)


    def send_serial_command(self, command: str) -> None:
        if self.serial_conn is None or not self.serial_conn.is_open:
            return

        try:
            self.serial_conn.write(command.encode())
            self.serial_conn.flush()
            self.get_logger().info(f"TX: {command.strip()}")

        except SerialException as exc:
            self.get_logger().error(f'Serial write error: {exc}')

    def parse_and_publish(self, data_str: str) -> None:
        """Parse Arduino CSV payload and publish ROS2 topics."""
        # Process lines containing comma separation
        parts = data_str.split(',')
        if len(parts) != 9:
            return  # Filter out non-telemetry serial frames silently

        try:
            # Extract standard localization inputs
            steer_angle = float(parts[0])
            wheel_speed = float(parts[1])
            heading_deg = float(parts[2])
            gyro_z = float(parts[3])
            acc_x = float(parts[4])
            acc_y = float(parts[5])
            
            # FIXED: Correctly parse the remaining ultrasonic variables out of the payload
            dist_front_cm = float(parts[6])
            dist_right_cm = float(parts[7])
            dist_left_cm = float(parts[8])
            
        except ValueError as exc:
            self.get_logger().warn(f'Value conversion failure: {exc}')
            return

        now = self.get_clock().now()

        # Publish Core Data Frames
        steer_msg = Float32()
        steer_msg.data = steer_angle
        self.steer_pub.publish(steer_msg)

        wheel_msg = Float32()
        wheel_msg.data = wheel_speed
        self.wheel_pub.publish(wheel_msg)

        heading_msg = Float32()
        heading_msg.data = heading_deg
        self.heading_pub.publish(heading_msg)

        # Publish IMU Message Block
        imu_msg = Imu()
        imu_msg.header.stamp = now.to_msg()
        imu_msg.header.frame_id = self.imu_frame_id

        yaw_rad = math.radians(heading_deg)
        imu_msg.orientation = self._yaw_to_quaternion(yaw_rad)

        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = gyro_z

        imu_msg.linear_acceleration.x = acc_x
        imu_msg.linear_acceleration.y = acc_y
        imu_msg.linear_acceleration.z = 0.0

        imu_msg.orientation_covariance = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.2]
        imu_msg.angular_velocity_covariance = [0.02, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.05]
        imu_msg.linear_acceleration_covariance = [0.05, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.1]
        self.imu_pub.publish(imu_msg)

        # Publish Ultrasonic Ranges
        self.us_front_pub.publish(self.create_range_msg(now, 'ultrasonic_front_link', dist_front_cm))
        self.us_left_pub.publish(self.create_range_msg(now, 'ultrasonic_left_link', dist_left_cm))
        self.us_right_pub.publish(self.create_range_msg(now, 'ultrasonic_right_link', dist_right_cm))

    def create_range_msg(self, stamp, frame_id, dist_cm):
        """Helper to format the Range message."""
        msg = Range()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.5
        msg.min_range = 0.02
        msg.max_range = 2.0
        msg.range = dist_cm / 100.0
        return msg

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Quaternion:
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q


def main(args=None):
    rclpy.init(args=args)
    node = SerialSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()