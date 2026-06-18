import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import logging
import json
import urllib.request
import urllib.error

# Try to import firebase_admin; if it's not available (e.g. in a dev env or
# the VS Code interpreter doesn't have the package), fall back to a lightweight
# mock so the ROS node can still be linted/run without hard ImportError.
try:
    import firebase_admin
    from firebase_admin import credentials, firestore

    cred = credentials.Certificate(
    '/home/kisali/fyp_ws/src/firebase_bridge/config/serviceAccountKey.json')
    firebase_admin.initialize_app(cred)
    db = firestore.client()
    FIRESTORE_SERVER_TIMESTAMP = firestore.SERVER_TIMESTAMP
    _FIREBASE_AVAILABLE = True
except Exception as e:
    logging.getLogger(__name__).warning(
        'firebase_admin not available: %s. Using mock DB. Install firebase-admin to enable Firestore updates.',
        e,
    )

    class _DummyDoc:
        def __init__(self, doc_id):
            self.doc_id = doc_id

        def update(self, data):
            logging.getLogger(__name__).info('[firebase-mock] update %s: %s', self.doc_id, data)

    class _DummyCollection:
        def __init__(self, name):
            self.name = name

        def document(self, doc_id):
            return _DummyDoc(doc_id)

    class _DummyDB:
        def collection(self, name):
            return _DummyCollection(name)

    db = _DummyDB()
    FIRESTORE_SERVER_TIMESTAMP = None
    _FIREBASE_AVAILABLE = False

WHEELCHAIR_ID = 'wheelchair_1'
MIN_X, MAX_X = 0.0, 10.0
MIN_Y, MAX_Y = 0.0, 8.0

def normalize(value, min_val, max_val):
    return max(0.0, min(1.0, (value - min_val) / (max_val - min_val)))

class OdomBridge(Node):
    def __init__(self):
        super().__init__('odom_bridge')
        self.subscription = self.create_subscription(
            Odometry,
            '/wheel/odom',
            self.odom_callback,
            10,
        )

    def odom_callback(self, msg):
        odom_x = msg.pose.pose.position.x
        odom_y = msg.pose.pose.position.y

        self.get_logger().info( f"Odom = ({odom_x:.2f}, {odom_y:.2f})")

        # ui_x = normalize(odom_x, MIN_X, MAX_X)
        # ui_y = normalize(odom_y, MIN_Y, MAX_Y)

        status = 'Docked' if self.is_docked(odom_x, odom_y) else 'In Transit'

        payload = {
            'id': WHEELCHAIR_ID,
            'location': {'x': odom_x, 'y': odom_y},
            'status': status,
        }

        # Try sending to local HTTP adapter first (no extra deps required here)
        try:
            req = urllib.request.Request(
                'http://localhost:8080/update',
                data=json.dumps(payload).encode('utf-8'),
                headers={'Content-Type': 'application/json'},
                method='POST',
            )
            with urllib.request.urlopen(req, timeout=1.0) as resp:
                # optional: check resp.status
                pass
            return
        except (urllib.error.URLError, urllib.error.HTTPError, TimeoutError) as e:
            # adapter not reachable; fall back to direct Firestore update (mock safe)
            self.get_logger().warning('Adapter unavailable, falling back to Firestore: %s', e)

        # If adapter not available, update Firestore directly (may be mock)
        try:
            db.collection('wheelchairs').document(WHEELCHAIR_ID).update({
                'location': {'x': odom_x, 'y': odom_y},
                'status': status,
                'updatedAt': firestore.SERVER_TIMESTAMP,
            })
        except Exception as e:
            self.get_logger().error('Failed to update Firestore: %s', e)

    def is_docked(self, x, y):
        # implement your dock detection logic here
        return abs(x - 1.2) < 0.2 and abs(y - 0.5) < 0.2

def main(args=None):
    rclpy.init(args=args)
    node = OdomBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()