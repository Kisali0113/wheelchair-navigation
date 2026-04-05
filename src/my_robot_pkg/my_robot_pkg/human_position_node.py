import math
import os
import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from ultralytics import YOLO
from sklearn.cluster import DBSCAN


class HumanPositionNode(Node):
    def __init__(self):
        super().__init__('human_position_node')

        self.declare_parameter('model_path', os.path.join(os.path.dirname(__file__), 'yolov8n.pt'))
        self.declare_parameter('color_width', 640)
        self.declare_parameter('color_height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('fx', 615.0)
        self.declare_parameter('fy', 615.0)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)
        self.declare_parameter('depth_min_m', 0.2)
        self.declare_parameter('depth_max_m', 3.0)
        self.declare_parameter('dbscan_eps', 0.15)
        self.declare_parameter('dbscan_min_samples', 10)

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.width = self.get_parameter('color_width').get_parameter_value().integer_value
        self.height = self.get_parameter('color_height').get_parameter_value().integer_value
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value
        self.fx = self.get_parameter('fx').get_parameter_value().double_value
        self.fy = self.get_parameter('fy').get_parameter_value().double_value
        self.cx = self.get_parameter('cx').get_parameter_value().double_value
        self.cy = self.get_parameter('cy').get_parameter_value().double_value
        self.depth_min_m = self.get_parameter('depth_min_m').get_parameter_value().double_value
        self.depth_max_m = self.get_parameter('depth_max_m').get_parameter_value().double_value
        self.dbscan_eps = self.get_parameter('dbscan_eps').get_parameter_value().double_value
        self.dbscan_min_samples = self.get_parameter('dbscan_min_samples').get_parameter_value().integer_value

        self.person_pub = self.create_publisher(PointStamped, 'person_position', 10)
        self.create_subscription(String, 'robotcommand', self.robotcommand_callback, 10)

        self.active = False
        self.prev_center = None

        self.model = YOLO(model_path)

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)

        self.get_logger().info(f'HumanPositionNode initialized, model={model_path}')
        self.create_timer(0.1, self.timer_callback)

    def robotcommand_callback(self, msg: String):
        command = msg.data.strip()
        self.get_logger().info(f'Received robotcommand: {command}')
        if command == 'personsearching':
            self.active = True
            self.get_logger().info('Person searching event received, starting human position detection.')

    def timer_callback(self):
        if not self.active:
            return

        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=500)
            frames = self.align.process(frames)
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                return

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            self.process_frame(color_image, depth_image)
        except Exception as e:
            self.get_logger().error(f'HumanPositionNode frame processing error: {e}')

    def process_frame(self, color_image, depth_image):
        results = self.model(color_image, verbose=False)
        for r in results:
            boxes = r.boxes
            for box in boxes:
                cls = int(box.cls[0])
                if cls != 0:
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                points = self.extract_points(depth_image, x1, y1, x2, y2)
                if len(points) < 50:
                    continue

                center = self.cluster_and_center(points)
                if center is None:
                    continue

                smoothed_center = self.smooth_center(center)
                self.publish_person_position(smoothed_center)
                self.draw_visualization(color_image, smoothed_center)

        cv2.imshow('Human Position Detection', color_image)
        cv2.waitKey(1)

    def extract_points(self, depth_image, x1, y1, x2, y2):
        points = []
        step = 4
        for v in range(y1, y2, step):
            for u in range(x1, x2, step):
                depth = depth_image[v, u]
                if depth == 0:
                    continue
                x, y, z = self.pixel_to_3d(u, v, depth)
                if z < self.depth_min_m or z > self.depth_max_m:
                    continue
                if v > (y1 + y2) // 2:
                    continue
                points.append([x, y, z])
        return np.array(points)

    def pixel_to_3d(self, u, v, depth):
        z = depth / 1000.0
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        return x, y, z

    def cluster_and_center(self, points):
        clustering = DBSCAN(eps=self.dbscan_eps, min_samples=self.dbscan_min_samples).fit(points)
        labels = clustering.labels_
        unique_labels = set(labels)
        unique_labels.discard(-1)
        if not unique_labels:
            return None

        best_cluster = None
        max_size = 0
        for label in unique_labels:
            cluster_points = points[labels == label]
            if len(cluster_points) > max_size:
                max_size = len(cluster_points)
                best_cluster = cluster_points

        if best_cluster is None:
            return None
        return np.mean(best_cluster, axis=0)

    def smooth_center(self, center):
        if self.prev_center is None:
            self.prev_center = center
        else:
            self.prev_center = 0.7 * self.prev_center + 0.3 * center
        return self.prev_center

    def publish_person_position(self, center):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.point.x = float(center[0])
        msg.point.y = float(center[1])
        msg.point.z = float(center[2])
        self.person_pub.publish(msg)
        self.get_logger().info(f'Published person_position x={msg.point.x:.2f}, y={msg.point.y:.2f}, z={msg.point.z:.2f}')

    def draw_visualization(self, image, center):
        center_u = int(center[0] * self.fx / center[2] + self.cx)
        center_v = int(center[1] * self.fy / center[2] + self.cy)
        cv2.circle(image, (center_u, center_v), 6, (0, 0, 255), -1)

    def destroy_node(self):
        try:
            cv2.destroyAllWindows()
            self.pipeline.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HumanPositionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('HumanPositionNode shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
