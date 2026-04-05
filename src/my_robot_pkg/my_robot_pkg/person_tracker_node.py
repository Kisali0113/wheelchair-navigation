import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from ultralytics import YOLO
from deep_sort_realtime.deepsort_tracker import DeepSort
import pyrealsense2 as rs
import numpy as np
import cv2
import time


class PersonTrackerNode(Node):
    def __init__(self):
        super().__init__('person_tracker_node')

        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('target_timeout', 3.0)
        self.declare_parameter('kp_turn', 0.005)
        self.declare_parameter('kd_turn', 0.002)
        self.declare_parameter('kp_dist', 0.8)
        self.declare_parameter('target_distance', 1.2)
        self.declare_parameter('min_conf', 0.5)
        self.declare_parameter('min_width', 50)
        self.declare_parameter('min_height', 100)
        self.declare_parameter('max_distance', 5.0)
        self.declare_parameter('min_distance', 0.3)
        self.declare_parameter('floor_threshold', 0.85)
        self.declare_parameter('shape_ratio', 1.2)
        self.declare_parameter('hist_match_threshold', 0.8)

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.target_timeout = self.get_parameter('target_timeout').get_parameter_value().double_value
        self.kp_turn = self.get_parameter('kp_turn').get_parameter_value().double_value
        self.kd_turn = self.get_parameter('kd_turn').get_parameter_value().double_value
        self.kp_dist = self.get_parameter('kp_dist').get_parameter_value().double_value
        self.target_distance = self.get_parameter('target_distance').get_parameter_value().double_value
        self.min_conf = self.get_parameter('min_conf').get_parameter_value().double_value
        self.min_width = self.get_parameter('min_width').get_parameter_value().integer_value
        self.min_height = self.get_parameter('min_height').get_parameter_value().integer_value
        self.max_distance = self.get_parameter('max_distance').get_parameter_value().double_value
        self.min_distance = self.get_parameter('min_distance').get_parameter_value().double_value
        self.floor_threshold = self.get_parameter('floor_threshold').get_parameter_value().double_value
        self.shape_ratio = self.get_parameter('shape_ratio').get_parameter_value().double_value
        self.hist_match_threshold = self.get_parameter('hist_match_threshold').get_parameter_value().double_value

        self.person_pub = self.create_publisher(PointStamped, 'person_position', 10)
        self.create_subscription(String, 'microcontrolling', self.microcontrolling_callback, 10)

        self.active = False
        self.model = YOLO(model_path)
        self.tracker = DeepSort(max_age=100, n_init=3, max_iou_distance=0.7)

        self.target_id = None
        self.target_hist = None
        self.last_seen_time = time.time()
        self.last_known_cx = None
        self.prev_error_x = 0
        self.prev_time = time.time()

        # RealSense setup
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(config)
        time.sleep(2)

        self.get_logger().info('PersonTrackerNode initialized.')
        self.create_timer(0.1, self.timer_callback)

    def microcontrolling_callback(self, msg: String):
        command = msg.data.strip()
        self.get_logger().info(f'Received microcontrolling: {command}')
        if command == 'followperson':
            self.active = True
            self.get_logger().info('Person following activated.')

    def timer_callback(self):
        if not self.active:
            return

        try:
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                return

            frame = np.asanyarray(color_frame.get_data())
            self.process_frame(frame, depth_frame)
        except Exception as e:
            self.get_logger().error(f'PersonTrackerNode frame processing error: {e}')

    def process_frame(self, frame, depth_frame):
        results = self.model(frame, classes=[0], verbose=False)
        detections = []

        for r in results:
            for box in r.boxes:
                conf = float(box.conf[0])
                if conf < self.min_conf:
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                w = x2 - x1
                h = y2 - y1

                if w < self.min_width or h < self.min_height:
                    continue

                detections.append(([x1, y1, w, h], conf, 'person'))

        tracks = self.tracker.update_tracks(detections, frame=frame)
        current_target = None
        current_time = time.time()

        for track in tracks:
            if not track.is_confirmed():
                continue

            track_id = track.track_id
            l, t, r, b = map(int, track.to_ltrb())

            h_img, w_img, _ = frame.shape

            l = max(0, min(l, w_img - 1))
            r = max(0, min(r, w_img - 1))
            t = max(0, min(t, h_img - 1))
            b = max(0, min(b, h_img - 1))

            if r <= l or b <= t:
                continue

            cx = int((l + r) / 2)
            cy = int(t + 0.3 * (b - t))

            cx = max(0, min(cx, w_img - 1))
            cy = max(0, min(cy, h_img - 1))

            try:
                distance = depth_frame.get_distance(cx, cy)
            except:
                continue

            if distance < self.min_distance or distance > self.max_distance:
                continue

            if cy > int(self.floor_threshold * h_img):
                continue

            if (b - t) / (r - l) < self.shape_ratio:
                continue

            hist = self.get_color_hist(frame, (l, t, r, b))
            if hist is None:
                continue

            if self.target_id is None and distance > 0.5 and (b - t) > 120:
                self.target_id = track_id
                self.target_hist = hist
                self.get_logger().info(f'Target LOCKED → ID {self.target_id}')

            if track_id == self.target_id:
                current_target = (distance, l, t, r, b, cx)
                self.last_seen_time = current_time
                self.last_known_cx = cx

                cv2.rectangle(frame, (l, t), (r, b), (0, 0, 255), 3)
                cv2.putText(frame, f'TARGET {track_id}', (l, t - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            elif (current_time - self.last_seen_time > 1.0) and self.target_hist is not None:
                match_score = cv2.compareHist(self.target_hist, hist, cv2.HISTCMP_CORREL)
                if match_score > self.hist_match_threshold:
                    self.target_id = track_id
                    self.target_hist = hist
                    self.get_logger().info(f'Re-ID → {self.target_id}')
            else:
                cv2.rectangle(frame, (l, t), (r, b), (0, 255, 0), 2)

        if current_target:
            distance, l, t, r, b, cx = current_target
            self.publish_person_position(cx, cy, distance, frame.shape[1])
        elif current_time - self.last_seen_time < self.target_timeout:
            self.get_logger().info('Searching for target...')
        else:
            self.target_id = None
            self.target_hist = None
            self.get_logger().info('Target LOST')

        cv2.imshow('Person Tracker', frame)
        cv2.waitKey(1)

    def get_color_hist(self, frame, bbox):
        x1, y1, x2, y2 = bbox
        if x2 <= x1 or y2 <= y1:
            return None
        roi = frame[y1:y2, x1:x2]
        if roi.size == 0:
            return None
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        hist = cv2.calcHist([hsv], [0], None, [180], [0, 180])
        hist = cv2.normalize(hist, hist)
        return hist

    def publish_person_position(self, cx, cy, distance, img_width):
        # Convert pixel coordinates to camera frame X, Y
        fx, fy = 615.0, 615.0
        cx_cam, cy_cam = 320.0, 240.0

        x = (cx - cx_cam) * distance / fx
        y = (cy - cy_cam) * distance / fy

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.point.x = float(x)
        msg.point.y = float(y)
        msg.point.z = float(distance)

        self.person_pub.publish(msg)
        self.get_logger().info(f'Published person_position x={x:.2f}, y={y:.2f}, z={distance:.2f}')

    def destroy_node(self):
        try:
            cv2.destroyAllWindows()
            self.pipeline.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PersonTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('PersonTrackerNode shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
