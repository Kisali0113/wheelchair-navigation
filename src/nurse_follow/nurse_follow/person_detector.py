#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32

from cv_bridge import CvBridge

import cv2
import numpy as np

from ultralytics import YOLO


class PersonDetector(Node):

    def __init__(self):

        super().__init__('person_detector')

        self.bridge = CvBridge()

        self.rgb_image = None
        self.depth_image = None

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.model = YOLO("yolov8n.pt")

        self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.rgb_callback,
            10
        )

        self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10
        )

        self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.camera_info_callback,
            10
        )

        self.target_pub = self.create_publisher(
            PointStamped,
            '/person_target',
            10
        )

        self.distance_pub = self.create_publisher(
            Float32,
            '/person_distance',
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.process_frame
        )

        self.get_logger().info("Person detector started")

    def camera_info_callback(self, msg):

        self.fx = msg.k[0]
        self.fy = msg.k[4]

        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def rgb_callback(self, msg):

        self.rgb_image = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

    def depth_callback(self, msg):

        self.depth_image = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='passthrough'
        )

    def get_median_depth(self, cx, cy):

        window = 10

        x1 = max(0, cx - window)
        x2 = min(self.depth_image.shape[1], cx + window)

        y1 = max(0, cy - window)
        y2 = min(self.depth_image.shape[0], cy + window)

        roi = self.depth_image[y1:y2, x1:x2]

        valid_depths = roi[roi > 0]

        if len(valid_depths) == 0:
            return None

        return float(np.median(valid_depths))

    def process_frame(self):

        if self.rgb_image is None:
            return

        if self.depth_image is None:
            return

        if self.fx is None:
            return

        frame = self.rgb_image.copy()

        results = self.model(frame, verbose=False)

        nearest_distance = 999.0

        nearest_X = None
        nearest_Y = None
        nearest_Z = None

        nearest_box = None

        for result in results:

            for box in result.boxes:

                cls = int(box.cls[0])

                # YOLO class 0 = person
                if cls != 0:
                    continue

                x1, y1, x2, y2 = map(
                    int,
                    box.xyxy[0]
                )

                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                depth_mm = self.get_median_depth(
                    cx,
                    cy
                )

                if depth_mm is None:
                    continue

                distance_m = depth_mm / 1000.0

                if distance_m <= 0:
                    continue

                u = float(cx)
                v = float(cy)

                Z = distance_m

                X = (u - self.cx) * Z / self.fx
                Y = (v - self.cy) * Z / self.fy

                if Z < nearest_distance:

                    nearest_distance = Z

                    nearest_X = X
                    nearest_Y = Y
                    nearest_Z = Z

                    nearest_box = (
                        x1,
                        y1,
                        x2,
                        y2,
                        cx,
                        cy
                    )

        if nearest_X is not None:

            msg = PointStamped()

            msg.header.stamp = (
                self.get_clock()
                .now()
                .to_msg()
            )

            msg.header.frame_id = "camera_color_optical_frame"

            msg.point.x = float(nearest_X)
            msg.point.y = float(nearest_Y)
            msg.point.z = float(nearest_Z)

            self.target_pub.publish(msg)

            dist_msg = Float32()
            dist_msg.data = float(nearest_Z)

            self.distance_pub.publish(dist_msg)

            x1, y1, x2, y2, cx, cy = nearest_box

            cv2.rectangle(
                frame,
                (x1, y1),
                (x2, y2),
                (0, 255, 0),
                2
            )

            cv2.circle(
                frame,
                (cx, cy),
                5,
                (0, 0, 255),
                -1
            )

            cv2.putText(
                frame,
                f"X:{nearest_X:.2f}m",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2
            )

            cv2.putText(
                frame,
                f"Y:{nearest_Y:.2f}m",
                (20, 70),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2
            )

            cv2.putText(
                frame,
                f"Z:{nearest_Z:.2f}m",
                (20, 100),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2
            )

        cv2.imshow(
            "Nurse Tracking",
            frame
        )

        cv2.waitKey(1)


def main(args=None):

    rclpy.init(args=args)

    node = PersonDetector()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()