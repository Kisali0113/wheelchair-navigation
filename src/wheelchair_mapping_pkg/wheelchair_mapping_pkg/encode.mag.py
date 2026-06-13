"""
Fused Odometry Node — Ackermann (3-wheel, front-drive, front-steer)
=====================================================================
Sensor fusion strategy
----------------------
Heading / Yaw
    Complementary filter blends:
      • IMU gyroscope (gz) — low-noise short-term integration
      • Magnetometer absolute heading — prevents gyro drift
    yaw = α * (yaw + gz*dt)  +  (1-α) * mag_heading
    α close to 1  → trust gyro for fast dynamics
    α close to 0  → trust magnetometer (good when stationary)

Position
    x, y integrated from encoder speed (most reliable on hard floors).
    IMU ax/ay are used as a *consistency check* only — if encoder speed
    looks stale (robot hasn't published for > STALE_THRESH seconds) the
    node falls back to integrating ax/ay so position keeps moving.

Angular velocity
    Reported as the blended gyro value (more accurate than tan(steer)/L
    at low speeds or when wheels slip).
"""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from geometry_msgs.msg import Quaternion, TransformStamped
import tf2_ros


# Seconds without an encoder update before IMU acceleration fallback kicks in
ENCODER_STALE_THRESH = 0.5


class EncoderOdomNode(Node):
    """Fused odometry: encoder + steering + IMU (gz, ax, ay) + magnetometer."""

    def __init__(self):
        super().__init__('encoder_odom_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('wheel_base',            1.05)
        self.declare_parameter('velocity_topic',        '/wheel_speed')
        self.declare_parameter('steer_angle_topic',     '/steer_angle')
        self.declare_parameter('heading_topic',         '/mag_heading')
        self.declare_parameter('imu_topic',             '/imu/data')
        self.declare_parameter('odom_frame',            'odom')
        self.declare_parameter('base_frame',            'base_link')

        # Complementary filter weight for gyro vs magnetometer
        # 0.98 = trust gyro 98 % each step, drift corrected by mag
        self.declare_parameter('comp_filter_alpha',     0.98)

        # Low-pass smoothing on raw magnetometer heading
        # 0.1 = heavy smoothing  |  1.0 = no smoothing
        self.declare_parameter('heading_filter_alpha',  0.1)

        # IMU acceleration low-pass (reduces vibration noise)
        self.declare_parameter('accel_filter_alpha',    0.2)

        self.wheel_base         = self.get_parameter('wheel_base').value
        self.velocity_topic     = self.get_parameter('velocity_topic').value
        self.steer_angle_topic  = self.get_parameter('steer_angle_topic').value
        self.heading_topic      = self.get_parameter('heading_topic').value
        self.imu_topic          = self.get_parameter('imu_topic').value
        self.odom_frame         = self.get_parameter('odom_frame').value
        self.base_frame         = self.get_parameter('base_frame').value
        self.comp_alpha         = self.get_parameter('comp_filter_alpha').value
        self.heading_alpha      = self.get_parameter('heading_filter_alpha').value
        self.accel_alpha        = self.get_parameter('accel_filter_alpha').value

        # ── State ─────────────────────────────────────────────────────────────
        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0          # fused heading (rad)

        # Raw sensor values
        self.velocity      = 0.0   # encoder wheel speed   (m/s)
        self.steer_angle   = 0.0   # front-wheel steer     (rad)
        self.mag_heading   = 0.0   # smoothed mag heading  (rad)
        self.gz            = 0.0   # IMU yaw rate          (rad/s)
        self.ax_filtered   = 0.0   # body-frame accel x    (m/s²)
        self.ay_filtered   = 0.0   # body-frame accel y    (m/s²)

        # Velocity from IMU integration (fallback when encoder is stale)
        self.vx_imu = 0.0
        self.vy_imu = 0.0

        # Freshness tracking
        self.last_encoder_time = self.get_clock().now()
        self.encoder_fresh     = False
    #    self.mag_initialised   = False   # wait for first mag reading before fusing

        self.last_time = self.get_clock().now()

        # ── Subscribers ───────────────────────────────────────────────────────
        self.vel_sub = self.create_subscription(
            Float32, self.velocity_topic, self.velocity_callback, 10)
        self.steer_sub = self.create_subscription(
            Float32, self.steer_angle_topic, self.steer_callback, 10)
        self.heading_sub = self.create_subscription(
            Float32, self.heading_topic, self.heading_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, self.imu_topic, self.imu_callback, 10)

        # ── Publisher & TF ────────────────────────────────────────────────────
        self.odom_pub    = self.create_publisher(Odometry, '/wheel/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ── Integration timer (20 Hz) ─────────────────────────────────────────
        self.timer = self.create_timer(0.05, self.update_odometry)

        self.get_logger().info(
            f'Fused odometry node started\n'
            f'  velocity     : {self.velocity_topic}\n'
            f'  steer angle  : {self.steer_angle_topic}\n'
            f'  magnetometer : {self.heading_topic}\n'
            f'  IMU          : {self.imu_topic}\n'
            f'  comp_alpha   : {self.comp_alpha}  '
            f'(gyro weight in complementary filter)\n'
        )

    # =========================================================================
    # Callbacks
    # =========================================================================

    def velocity_callback(self, msg: Float32):
        self.velocity = msg.data
        self.last_encoder_time = self.get_clock().now()
        self.encoder_fresh = True
        # Reset IMU velocity accumulator each time encoder gives a fresh sample
        self.vx_imu = 0.0
        self.vy_imu = 0.0

    def steer_callback(self, msg: Float32):
        raw = msg.data
        self.steer_angle = (
            math.radians(raw) if abs(raw) > 2.0 * math.pi else raw
        )

    def heading_callback(self, msg: Float32):
        """Low-pass filtered magnetometer heading."""
        raw = msg.data
        raw_rad = math.radians(raw) if abs(raw) > 2.0 * math.pi else raw

        if not self.mag_initialised:
            # Seed filter and yaw with first reading
            self.mag_heading   = raw_rad
            self.yaw           = raw_rad
            self.mag_initialised = True
        else:
            # Unwrap to avoid ±π jumps before filtering
            delta = self._angle_diff(raw_rad, self.mag_heading)
            self.mag_heading = self._normalize_angle(
                self.mag_heading + self.heading_alpha * delta
            )

    def imu_callback(self, msg: Imu):
        """
        Extract yaw-rate and planar accelerations from sensor_msgs/Imu.
        Applies a simple low-pass filter to suppress vibration noise.
        """
        self.gz = msg.angular_velocity.z          # rad/s

        # Low-pass filter on accelerations
        self.ax_filtered = (
            self.accel_alpha * msg.linear_acceleration.x
            + (1.0 - self.accel_alpha) * self.ax_filtered
        )
        self.ay_filtered = (
            self.accel_alpha * msg.linear_acceleration.y
            + (1.0 - self.accel_alpha) * self.ay_filtered
        )

    # =========================================================================
    # Main update loop
    # =========================================================================

    def update_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        if dt <= 0.0:
            return

        # ── 1. Fuse heading ──────────────────────────────────────────────────
        # Complementary filter: gyro propagates yaw, magnetometer corrects drift
        if self.mag_initialised:
            gyro_yaw = self._normalize_angle(self.yaw + self.gz * dt)
            self.yaw = self._normalize_angle(
                self.comp_alpha * gyro_yaw
                + (1.0 - self.comp_alpha) * self.mag_heading
            )
        else:
            # No mag yet — pure gyro integration (will drift; wait for mag)
            self.yaw = self._normalize_angle(self.yaw + self.gz * dt)

        # ── 2. Determine linear velocity ─────────────────────────────────────
        encoder_age = (current_time - self.last_encoder_time).nanoseconds / 1e9
        encoder_stale = encoder_age > ENCODER_STALE_THRESH

        if not encoder_stale:
            # Primary: encoder speed projected onto world frame
            linear_vel = self.velocity
            dx = linear_vel * math.cos(self.yaw) * dt
            dy = linear_vel * math.sin(self.yaw) * dt
        else:
            # Fallback: integrate IMU body-frame accelerations into world frame
            # Transform body accel → world frame using current yaw
            cos_y = math.cos(self.yaw)
            sin_y = math.sin(self.yaw)
            ax_world = cos_y * self.ax_filtered - sin_y * self.ay_filtered
            ay_world = sin_y * self.ax_filtered + cos_y * self.ay_filtered

            self.vx_imu += ax_world * dt
            self.vy_imu += ay_world * dt

            dx = self.vx_imu * dt
            dy = self.vy_imu * dt
            linear_vel = math.hypot(self.vx_imu, self.vy_imu)

            if encoder_age > ENCODER_STALE_THRESH * 4:
                # Encoder very stale — decay IMU velocity to prevent runaway drift
                decay = max(0.0, 1.0 - dt * 2.0)
                self.vx_imu *= decay
                self.vy_imu *= decay

        # ── 3. Update position ───────────────────────────────────────────────
        self.x += dx
        self.y += dy

        # ── 4. Angular velocity ──────────────────────────────────────────────
        # Use IMU gz directly (more accurate than kinematic estimate at low speed)
        angular_vel = self.gz

        # ── 5. Publish ───────────────────────────────────────────────────────
        self.publish_odometry(current_time, linear_vel, angular_vel)
        self.last_time = current_time

    # =========================================================================
    # Publish
    # =========================================================================

    def publish_odometry(self, stamp, linear_velocity, angular_velocity):
        odom = Odometry()
        odom.header.stamp    = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id  = self.base_frame

        # Pose
        odom.pose.pose.position.x    = self.x
        odom.pose.pose.position.y    = self.y
        odom.pose.pose.position.z    = 0.0
        odom.pose.pose.orientation   = self._yaw_to_quaternion(self.yaw)

        # Pose covariance — tighter on x/y because encoder is reliable
        # yaw (index 35) is low because complementary filter is good
        odom.pose.covariance = [
            0.005, 0.0,   0.0,   0.0,  0.0,  0.0,
            0.0,   0.005, 0.0,   0.0,  0.0,  0.0,
            0.0,   0.0,   1e6,   0.0,  0.0,  0.0,   # z not observed → large
            0.0,   0.0,   0.0,   1e6,  0.0,  0.0,   # roll  not observed
            0.0,   0.0,   0.0,   0.0,  1e6,  0.0,   # pitch not observed
            0.0,   0.0,   0.0,   0.0,  0.0,  0.02,  # yaw — fused
        ]

        # Twist
        odom.twist.twist.linear.x  = linear_velocity
        odom.twist.twist.angular.z = angular_velocity

        # Twist covariance
        odom.twist.covariance = [
            0.01, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.01, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  1e6,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  1e6,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  1e6,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.02,
        ]

        self.odom_pub.publish(odom)
        self.publish_tf(stamp, odom.pose.pose)

    def publish_tf(self, stamp, pose):
        t = TransformStamped()
        t.header.stamp          = stamp.to_msg()
        t.header.frame_id       = self.odom_frame
        t.child_frame_id        = self.base_frame
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation      = pose.orientation
        self.tf_broadcaster.sendTransform(t)

    # =========================================================================
    # Utilities
    # =========================================================================

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Quaternion:
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        """Signed shortest angular distance from b to a."""
        return math.atan2(math.sin(a - b), math.cos(a - b))


def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()