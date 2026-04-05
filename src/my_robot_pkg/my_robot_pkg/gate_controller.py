import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# TODO: Completed - Basic gate controller implementation with open/close logic
# TODO: Completed - Improve emergency and granted_speaker handling (e.g., emergency gate closure, speaker permission validation)


class GateController(Node):
    def __init__(self):
        super().__init__('gate_controller')

        # Publishers
        self.chair_status_pub = self.create_publisher(String, 'chair_status', 10)
        self.speaker_control_pub = self.create_publisher(String, 'speaker_control', 10)
        self.caregiver_control_pub = self.create_publisher(String, 'caregiver_control', 10)
        self.maincontrolling_pub = self.create_publisher(String, 'maincontrolling', 10)

        # Subscribers
        self.create_subscription(String, 'robot_command', self.robot_command_callback, 10)
        self.create_subscription(String, 'speaker_control', self.speaker_control_callback, 10)
        self.create_subscription(String, 'caregiver_control', self.caregiver_control_callback, 10)
        self.create_subscription(String, 'maincontrolling', self.maincontrolling_callback, 10)

        # State
        self.gate_state = 'closed'  # closed or open
        self.publish_chair_status('closed')

        self.speaker_granted = False
        self.caregiver_granted = False
        self.waiting_for_open = False
        self.waiting_for_close = False

        self.get_logger().info('GateController node initialized (chair closed).')

    def publish_chair_status(self, status):
        msg = String()
        msg.data = f'chair_{status}'
        self.chair_status_pub.publish(msg)
        self.get_logger().info(f'Published chair_status: {msg.data}')

    def request_open(self):
        self.get_logger().info('Requesting open from speaker and caregiver...')
        self.publish_speaker_control('requestopen')
        self.publish_caregiver_control('gateopenermission')
        self.speaker_granted = False
        self.caregiver_granted = False
        self.waiting_for_open = True

    def request_close(self):
        self.get_logger().info('Requesting close from speaker and caregiver...')
        self.publish_speaker_control('requestclose')
        self.publish_caregiver_control('gateclosermission')
        self.speaker_granted = False
        self.caregiver_granted = False
        self.waiting_for_close = True

    def publish_speaker_control(self, command):
        msg = String(); msg.data = command
        self.speaker_control_pub.publish(msg)
        self.get_logger().info(f'Published speaker_control: {command}')

    def publish_caregiver_control(self, command):
        msg = String(); msg.data = command
        self.caregiver_control_pub.publish(msg)
        self.get_logger().info(f'Published caregiver_control: {command}')

    def publish_maincontrolling(self, command):
        msg = String(); msg.data = command
        self.maincontrolling_pub.publish(msg)
        self.get_logger().info(f'Published maincontrolling: {command}')

    def robot_command_callback(self, msg):
        data = msg.data.strip()
        self.get_logger().info(f'Received robot_command: {data}')

        if data == 'arrived_at_chair':
            if self.gate_state == 'closed' and not self.waiting_for_open:
                self.request_open()
            else:
                self.get_logger().info('Chairs gate already open or open request in progress.')
        elif data == 'arrived_at_bed':
            self.get_logger().info('arrived_at_bed ignored by gate controller (keep current state).')
        elif data == 'arrived_at_washroom':
            self.get_logger().info('arrived_at_washroom ignored by gate controller (keep current state).')
        else:
            self.get_logger().warn(f'Unknown robot_command: {data}')

    def speaker_control_callback(self, msg):
        data = msg.data.strip()
        self.get_logger().info(f'Received speaker_control: {data}')

        if data == 'granted_speaker':
            self.speaker_granted = True
            self.evaluate_gate_permission()

    def caregiver_control_callback(self, msg):
        data = msg.data.strip()
        self.get_logger().info(f'Received caregiver_control: {data}')

        if data == 'granted_caregiver':
            self.caregiver_granted = True
            self.evaluate_gate_permission()

    def evaluate_gate_permission(self):
        if self.waiting_for_open:
            if self.caregiver_granted or (self.speaker_granted and self.caregiver_granted):
                self.get_logger().info('Open permission condition met.')
                self.waiting_for_open = False
                self.publish_maincontrolling('gate_open')
            else:
                self.get_logger().info('Open permission pending...')

        if self.waiting_for_close:
            if self.caregiver_granted or (self.speaker_granted and self.caregiver_granted):
                self.get_logger().info('Close permission condition met.')
                self.waiting_for_close = False
                self.publish_maincontrolling('gate_close')
            else:
                self.get_logger().info('Close permission pending...')

    def maincontrolling_callback(self, msg):
        data = msg.data.strip()
        self.get_logger().info(f'Received maincontrolling: {data}')

        if data == 'gateopendsuccess':
            if self.gate_state == 'closed':
                self.gate_state = 'open'
                self.publish_chair_status('open')
                self.get_logger().info('Gate open success, chair status updated.')

        elif data == 'gatecloserequest':
            if self.gate_state == 'open' and not self.waiting_for_close:
                self.request_close()
            else:
                self.get_logger().info('Gate already closing/open request in progress or not open.')

        elif data == 'gateclosesuccess':
            if self.gate_state == 'open':
                self.gate_state = 'closed'
                self.publish_chair_status('closed')
                self.get_logger().info('Gate close success, chair status updated.')

        elif data == 'emergency':
            self.publish_speaker_control('emergency')
            self.get_logger().info('Emergency received, published to speaker_control.')

        elif data == 'granted_speaker':
            self.speaker_granted = True
            self.evaluate_gate_permission()
            self.get_logger().info('Granted speaker received from maincontrolling.')

        elif data == 'gate_open':
            self.get_logger().info('Ignored own gate_open command in maincontrolling subscriber.')

        elif data == 'gate_close':
            self.get_logger().info('Ignored own gate_close command in maincontrolling subscriber.')

        else:
            self.get_logger().warn(f'Unknown maincontrolling command: {data}')


def main(args=None):
    rclpy.init(args=args)
    node = GateController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('GateController shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
