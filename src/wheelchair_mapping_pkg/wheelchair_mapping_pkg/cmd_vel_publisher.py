import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import math

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.declare_parameter('wheelbase', 1.05)  # Default wheelbase in meters
        self.wheelbase = self.get_parameter('wheelbase').value
        

        # --- Smoothing Filters (Low-Pass Filters) ---
        # Alpha values range from 0.0 to 1.0.
        # Lower values = smoother but slower response. Higher values = faster but twitchier response.
        self.declare_parameter('alpha_linear', 0.20) 
        self.declare_parameter('alpha_steer', 0.15)
        self.alpha_linear = self.get_parameter('alpha_linear').value
        self.alpha_steer = self.get_parameter('alpha_steer').value

        # --- Low-Speed Protection Cutoff ---
        # Below this linear velocity (m/s), we stop computing new Ackermann angles to avoid spikes.
        self.declare_parameter('min_linear_velocity', 0.05) 
        self.min_linear_velocity = self.get_parameter('min_linear_velocity').value

        self.linear_vel = 0.0
        self.angular_vel = 0.0

        # Persistent filtered memory states
        self.filtered_linear_vel = 0.0
        self.filtered_steer_ang = 0.0

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.cmd_vel_pub = self.create_publisher(
            Float64MultiArray,
            '/cmd_vels',
            10
        )

    def cmd_vel_callback(self, msg):
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z
        self.publish_cmd_vels()

    def publish_cmd_vels(self):
        # 1. Apply EMA Filter to linear velocity
        self.filtered_linear_vel = ((self.alpha_linear * self.linear_vel) + ((1.0 - self.alpha_linear) * self.filtered_linear_vel))

        # 2. Calculate steering angle with low-speed spike protection
        if abs(self.linear_vel) > self.min_linear_velocity:
            steer_ang_rad = math.atan(self.wheelbase * self.angular_vel / self.linear_vel)
            raw_steer_ang = math.degrees(steer_ang_rad)
        else:
            # If stopping or creeping too slowly, hold the last known steering angle 
            # instead of letting it spike to 90 degrees or dropping to 0 instantly.
            if self.linear_vel == 0.0 and self.angular_vel == 0.0:
                raw_steer_ang = 0.0  # Safe center if explicitly commanded to a full stop
            else:
                raw_steer_ang = self.filtered_steer_ang

        # 3. Apply EMA Filter to steering angle
        self.filtered_steer_ang = (self.alpha_steer * raw_steer_ang) + ((1.0 - self.alpha_steer) * self.filtered_steer_ang)
        
        # 4. Enforce physical constraints matching your Arduino configuration (Max 60 degrees)
        self.filtered_steer_ang = max(-60.0, min(60.0, self.filtered_steer_ang))

        # 5. Build and publish message
        msg = Float64MultiArray()
        msg.data = [self.filtered_linear_vel, self.filtered_steer_ang]
        self.cmd_vel_pub.publish(msg)
        
        self.get_logger().info(f'Published (Smoothed): Linear={self.filtered_linear_vel:.3f}, Steer={self.filtered_steer_ang:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()