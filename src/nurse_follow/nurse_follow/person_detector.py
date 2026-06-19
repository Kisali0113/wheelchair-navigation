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

        self.target_id = None
        self.follow_enabled = True
        self.lost_frames = 0

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

        results = self.model.track(
            frame,
            persist=True,
            tracker="bytetrack.yaml",
            verbose=False
        )

        target_found = False

        candidate_id = None
        candidate_distance = 999.0

        target_X = None
        target_Y = None
        target_Z = None

        target_box = None

        for result in results:

            if result.boxes.id is None:
                continue

            ids = result.boxes.id.cpu().numpy()
            boxes = result.boxes.xyxy.cpu().numpy()
            classes = result.boxes.cls.cpu().numpy()

            for box, cls, track_id in zip(
                boxes,
                classes,
                ids
            ):

                if int(cls) != 0:
                    continue

                x1, y1, x2, y2 = map(int, box)

                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                depth_mm = self.get_median_depth(cx, cy)

                if depth_mm is None:
                    continue

                Z = depth_mm / 1000.0

                if Z <= 0:
                    continue

                X = (cx - self.cx) * Z / self.fx
                Y = (cy - self.cy) * Z / self.fy

                track_id = int(track_id)

                #
                # NO TARGET YET
                #
                if self.target_id is None:

                    if Z < candidate_distance:

                        candidate_distance = Z
                        candidate_id = track_id

                        target_X = X
                        target_Y = Y
                        target_Z = Z

                        target_box = (
                            x1, y1,
                            x2, y2,
                            cx, cy
                        )

                #
                # TARGET ALREADY LOCKED
                #
                else:

                    if track_id != self.target_id:
                        continue

                    target_found = True

                    target_X = X
                    target_Y = Y
                    target_Z = Z

                    target_box = (
                        x1, y1,
                        x2, y2,
                        cx, cy
                    )

        #
        # LOCK TARGET ONCE
        #
        if self.target_id is None and candidate_id is not None:

            self.target_id = candidate_id

            self.get_logger().info(
                f"LOCKED TARGET ID {self.target_id}"
            )

            target_found = True

        #
        # LOST TARGET LOGIC
        #
        if not target_found:

            self.lost_frames += 1

        else:

            self.lost_frames = 0

        if self.lost_frames > 50:

            self.get_logger().warn(
                "Target Lost"
            )

            self.target_id = None
            self.lost_frames = 0

        #
        # PUBLISH TARGET
        #
        if target_X is not None:

            msg = PointStamped()

            msg.header.stamp = (
                self.get_clock().now().to_msg()
            )

            msg.header.frame_id = (
                "camera_color_optical_frame"
            )

            msg.point.x = float(target_X)
            msg.point.y = float(target_Y)
            msg.point.z = float(target_Z)

            self.target_pub.publish(msg)

            dist_msg = Float32()

            dist_msg.data = float(target_Z)

            self.distance_pub.publish(
                dist_msg
            )

            x1, y1, x2, y2, cx, cy = target_box

            cv2.rectangle(
                frame,
                (x1, y1),
                (x2, y2),
                (0, 0, 255),
                3
            )

            cv2.circle(
                frame,
                (cx, cy),
                5,
                (0, 255, 0),
                -1
            )

            cv2.putText(
                frame,
                f"TARGET ID {self.target_id}",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 0, 255),
                2
            )

            cv2.putText(
                frame,
                f"X={target_X:.2f}",
                (20, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2
            )

            cv2.putText(
                frame,
                f"Y={target_Y:.2f}",
                (20, 120),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2
            )

            cv2.putText(
                frame,
                f"Z={target_Z:.2f}",
                (20, 160),
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
