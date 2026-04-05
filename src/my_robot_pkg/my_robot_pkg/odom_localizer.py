import os
import math
import serial
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion


def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class OdometryLocalizer(Node):
    def __init__(self):
        super().__init__('odom_localizer')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheelbase_mm', 1032.33)
        self.declare_parameter('track_width_mm', 746.2)
        self.declare_parameter('tire_diameter_mm', 220.0)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')

        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.wheelbase_m = self.get_parameter('wheelbase_mm').get_parameter_value().double_value / 1000.0
        self.track_width_m = self.get_parameter('track_width_mm').get_parameter_value().double_value / 1000.0
        self.tire_diameter_m = self.get_parameter('tire_diameter_mm').get_parameter_value().double_value / 1000.0
        self.tire_radius_m = self.tire_diameter_m / 2.0
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.child_frame_id = self.get_parameter('child_frame_id').get_parameter_value().string_value

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = self.get_clock().now()

        self.serial = self._open_serial_port()
        self.create_timer(0.02, self.timer_callback)

        self.get_logger().info(f'OdometryLocalizer initialized on {self.serial_port} at {self.baud_rate} baud.')
        self.get_logger().info(
            f'Wheelbase: {self.wheelbase_m:.3f} m, Track width: {self.track_width_m:.3f} m, Tire diameter: {self.tire_diameter_m:.3f} m')

    def _open_serial_port(self):
        try:
            return serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port {self.serial_port}: {e}')
            return None

    def timer_callback(self):
        if self.serial is None:
            self.serial = self._open_serial_port()
            return

        try:
            if self.serial.in_waiting > 0:
                raw_line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                if raw_line:
                    self.process_serial_line(raw_line)
        except Exception as e:
            self.get_logger().error(f'Serial read error: {e}')
            self.serial = None

    def process_serial_line(self, raw_line: str):
        parsed = self.parse_encoder_line(raw_line)
        if parsed is None:
            self.get_logger().warn(f'Unable to parse encoder line: "{raw_line}"')
            return

        speed, steer_angle = parsed
        self.update_odometry(speed, steer_angle)

    def parse_encoder_line(self, raw_line: str):
        raw_line = raw_line.strip()
        if not raw_line:
            return None

        # support lines like "0.12,15" or "0.12 15" or "speed=0.12,angle=15"
        values = None

        if ',' in raw_line:
            parts = [p.strip() for p in raw_line.split(',') if p.strip()]
            if len(parts) == 2:
                values = parts
        elif ' ' in raw_line:
            parts = [p.strip() for p in raw_line.split() if p.strip()]
            if len(parts) == 2:
                values = parts
        elif '=' in raw_line:
            tokens = [t.strip() for t in raw_line.replace(';', ',').split(',') if t.strip()]
            data = {}
            for token in tokens:
                if '=' in token:
                    key, value = token.split('=', 1)
                    data[key.strip().lower()] = value.strip()
            speed = data.get('speed') or data.get('v') or data.get('linear')
            angle = data.get('angle') or data.get('steer') or data.get('theta')
            if speed is not None and angle is not None:
                return self._convert_to_float(speed, angle)
            rpm = data.get('rpm')
            rps = data.get('rps')
            if rpm is not None and angle is not None:
                speed = self._rpms_to_speed(rpm, per_minute=True)
                return self._convert_to_float(str(speed), angle)
            if rps is not None and angle is not None:
                speed = self._rpms_to_speed(rps, per_minute=False)
                return self._convert_to_float(str(speed), angle)

        if values is not None:
            return self._convert_to_float(values[0], values[1])

        return None

    def _convert_to_float(self, speed_text: str, angle_text: str):
        try:
            speed = float(speed_text)
            angle = float(angle_text)
            return speed, angle
        except ValueError:
            return None

    def _rpms_to_speed(self, rpm_text: str, per_minute: bool = True) -> float:
        try:
            value = float(rpm_text)
        except ValueError:
            return 0.0
        if per_minute:
            return value * (math.pi * self.tire_diameter_m) / 60.0
        return value * (math.pi * self.tire_diameter_m)

    def update_odometry(self, front_speed: float, steer_angle: float):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return

        steer_rad = math.radians(steer_angle) if abs(steer_angle) > 2 * math.pi else steer_angle
        longitudinal_speed = front_speed * math.cos(steer_rad)
        angular_speed = 0.0
        if abs(self.wheelbase_m) > 1e-6:
            angular_speed = front_speed * math.sin(steer_rad) / self.wheelbase_m

        self.x += longitudinal_speed * math.cos(self.yaw) * dt
        self.y += longitudinal_speed * math.sin(self.yaw) * dt
        self.yaw += angular_speed * dt

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion_from_yaw(self.yaw)
        odom.twist.twist.linear.x = longitudinal_speed
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = angular_speed

        self.odom_pub.publish(odom)
        self.last_time = now
        self.get_logger().debug(
            f'odom x={self.x:.3f} y={self.y:.3f} yaw={self.yaw:.3f} v={longitudinal_speed:.3f} omega={angular_speed:.3f}')

    def destroy_node(self):
        if self.serial and self.serial.is_open:
            self.serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OdometryLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('OdometryLocalizer shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
