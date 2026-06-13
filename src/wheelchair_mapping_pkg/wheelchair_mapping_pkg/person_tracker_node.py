#!/usr/bin/env python3
"""
Person Tracker Node — ROS 2 Iron/Jazzy + Intel RealSense (CPU-only)
====================================================================

Pipeline
--------
1. YOLOv8n detects all persons in the RGB frame.
2. On first run, the *nearest* person (smallest median depth) is chosen
   as the TARGET and their shirt-colour HSV histogram is stored as the
   re-identification signature.
3. Each subsequent frame, every detected person is matched against the
   stored signature via cosine similarity on HSV histogram.
4. A lightweight centroid tracker smooths the track across frames.
5. Only when the target is visible the node publishes:
     /target_person/pose        (geometry_msgs/PoseStamped, base_frame XY)
     /target_person/status      (std_msgs/String  — "TRACKING" | "REIDENTIFIED")
     /target_person/debug_image (sensor_msgs/Image — annotated RGB, remap to disable)
6. While target is out of scene: nothing is published.

Topics consumed
---------------
  /camera/color/image_raw          sensor_msgs/Image
  /camera/depth/image_rect_raw     sensor_msgs/Image   (uint16, millimetres)
  /camera/color/camera_info        sensor_msgs/CameraInfo

Parameters
----------
  reid_threshold      float  0.75   cosine similarity floor for re-ID match
  depth_topic         string        override depth topic if needed
  color_topic         string        override color topic if needed
  info_topic          string        override camera_info topic if needed
  base_frame          string base_link
  camera_frame        string camera_color_optical_frame
  shirt_roi_top       float  0.25   top of shirt ROI as fraction of bbox height
  shirt_roi_bottom    float  0.60   bottom of shirt ROI as fraction of bbox height
  publish_debug_image bool   True
"""

import math
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber

from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
import tf2_ros
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_pose

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


# ─── Constants ────────────────────────────────────────────────────────────────
HIST_BINS       = (12, 8, 8)   # H, S, V bins — compact but discriminative
HIST_RANGES     = [0, 180, 0, 256, 0, 256]
DEPTH_MEDIAN_K  = 5            # kernel for depth median patch (pixels)
MAX_DEPTH_M     = 8.0          # ignore detections beyond this distance
MIN_DEPTH_M     = 0.3          # ignore detections closer than this (noise)
SMOOTHING_N     = 5            # pose smoothing window (frames)
REID_COOLDOWN   = 10           # frames to skip re-ID check after confirmed track


# ─── Centroid Tracker (lightweight, no scipy dependency) ─────────────────────
class CentroidTracker:
    """
    Maps detection bounding-boxes to persistent integer IDs using centroid
    distance. Designed for single-target use but handles multiple detections.
    """

    def __init__(self, max_disappeared: int = 15, max_distance: float = 120.0):
        self.next_id       = 0
        self.objects: dict[int, np.ndarray] = {}   # id → centroid [cx, cy]
        self.disappeared: dict[int, int]    = {}   # id → missed-frame count
        self.max_disappeared = max_disappeared
        self.max_distance    = max_distance

    def register(self, centroid: np.ndarray) -> int:
        oid = self.next_id
        self.objects[oid]     = centroid
        self.disappeared[oid] = 0
        self.next_id         += 1
        return oid

    def deregister(self, oid: int):
        del self.objects[oid]
        del self.disappeared[oid]

    def update(self, bboxes: list[tuple]) -> dict[int, np.ndarray]:
        """
        bboxes: list of (x1, y1, x2, y2)
        returns: {object_id: centroid_array}
        """
        if not bboxes:
            for oid in list(self.disappeared):
                self.disappeared[oid] += 1
                if self.disappeared[oid] > self.max_disappeared:
                    self.deregister(oid)
            return self.objects

        new_centroids = np.array(
            [[(x1 + x2) / 2, (y1 + y2) / 2] for x1, y1, x2, y2 in bboxes],
            dtype=float
        )

        if not self.objects:
            for c in new_centroids:
                self.register(c)
            return self.objects

        obj_ids  = list(self.objects.keys())
        obj_cents = np.array(list(self.objects.values()), dtype=float)

        # Pairwise distances
        D = np.linalg.norm(obj_cents[:, None] - new_centroids[None, :], axis=2)

        # Greedy match: sort by distance, assign closest pairs first
        rows = D.min(axis=1).argsort()
        used_rows, used_cols = set(), set()
        for row in rows:
            col = int(D[row].argmin())
            if row in used_rows or col in used_cols:
                continue
            if D[row, col] > self.max_distance:
                continue
            oid = obj_ids[row]
            self.objects[oid]     = new_centroids[col]
            self.disappeared[oid] = 0
            used_rows.add(row)
            used_cols.add(col)

        # Unmatched existing → increment disappeared
        for row, oid in enumerate(obj_ids):
            if row not in used_rows:
                self.disappeared[oid] += 1
                if self.disappeared[oid] > self.max_disappeared:
                    self.deregister(oid)

        # Unmatched new → register
        for col in range(len(new_centroids)):
            if col not in used_cols:
                self.register(new_centroids[col])

        return self.objects


# ─── Main Node ────────────────────────────────────────────────────────────────
class PersonTrackerNode(Node):

    def __init__(self):
        super().__init__('person_tracker_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('reid_threshold',      0.75)
        self.declare_parameter('depth_topic',         '/camera/depth/image_rect_raw')
        self.declare_parameter('color_topic',         '/camera/color/image_raw')
        self.declare_parameter('info_topic',          '/camera/color/camera_info')
        self.declare_parameter('base_frame',          'base_link')
        self.declare_parameter('camera_frame',        'camera_color_optical_frame')
        self.declare_parameter('shirt_roi_top',       0.25)
        self.declare_parameter('shirt_roi_bottom',    0.60)
        self.declare_parameter('publish_debug_image', True)

        self.reid_thresh      = self.get_parameter('reid_threshold').value
        self.depth_topic      = self.get_parameter('depth_topic').value
        self.color_topic      = self.get_parameter('color_topic').value
        self.info_topic       = self.get_parameter('info_topic').value
        self.base_frame       = self.get_parameter('base_frame').value
        self.camera_frame     = self.get_parameter('camera_frame').value
        self.shirt_top        = self.get_parameter('shirt_roi_top').value
        self.shirt_bot        = self.get_parameter('shirt_roi_bottom').value
        self.pub_debug        = self.get_parameter('publish_debug_image').value

        # ── Internal state ─────────────────────────────────────────────────
        self.bridge           = CvBridge()
        self.camera_info      = None          # filled on first CameraInfo msg
        self.target_hist      = None          # HSV histogram signature
        self.target_id        = None          # tracker ID of confirmed target
        self.target_uid       = 1             # stable user-facing UID (always 1)
        self.reid_cooldown    = 0
        self.pose_buffer: deque = deque(maxlen=SMOOTHING_N)
        self.tracker          = CentroidTracker(max_disappeared=20, max_distance=150)

        # ── Load YOLO ─────────────────────────────────────────────────────
        if not YOLO_AVAILABLE:
            self.get_logger().fatal(
                'ultralytics not installed. Run: pip install ultralytics')
            raise RuntimeError('ultralytics missing')

        self.get_logger().info('Loading YOLOv8n … (first load may take a few seconds)')
        self.model = YOLO('yolov8n.pt')   # auto-downloads ~6 MB on first run
        # Force CPU and limit threads to avoid overloading the robot
        self.model.to('cpu')
        self.get_logger().info('YOLOv8n loaded.')

        # ── TF2 ──────────────────────────────────────────────────────────
        self.tf_buffer    = tf2_ros.Buffer()
        self.tf_listener  = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Subscribers (synchronised RGB + Depth) ────────────────────────
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.info_sub = self.create_subscription(
            CameraInfo, self.info_topic, self._info_cb, qos)

        self.rgb_sub   = Subscriber(self, Image, self.color_topic, qos_profile=qos)
        self.depth_sub = Subscriber(self, Image, self.depth_topic, qos_profile=qos)

        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=5, slop=0.05)
        self.sync.registerCallback(self._image_cb)

        # ── Publishers ────────────────────────────────────────────────────
        self.pose_pub   = self.create_publisher(PoseStamped, '/target_person/pose',   10)
        self.status_pub = self.create_publisher(String,      '/target_person/status', 10)
        if self.pub_debug:
            self.debug_pub = self.create_publisher(Image, '/target_person/debug_image', 5)

        self.get_logger().info(
            f'Person tracker ready.\n'
            f'  Depth    : {self.depth_topic}\n'
            f'  Colour   : {self.color_topic}\n'
            f'  Base frame : {self.base_frame}\n'
            f'  Re-ID thresh: {self.reid_thresh}\n'
            f'  → Approach the robot; the nearest person becomes the target.'
        )

    # =========================================================================
    # Camera info
    # =========================================================================

    def _info_cb(self, msg: CameraInfo):
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info(
                f'Camera intrinsics received: fx={msg.k[0]:.1f} fy={msg.k[4]:.1f} '
                f'cx={msg.k[2]:.1f} cy={msg.k[5]:.1f}')

    # =========================================================================
    # Main callback
    # =========================================================================

    def _image_cb(self, rgb_msg: Image, depth_msg: Image):
        if self.camera_info is None:
            return   # wait for intrinsics

        # ── Decode ────────────────────────────────────────────────────────
        bgr   = self.bridge.imgmsg_to_cv2(rgb_msg,   desired_encoding='bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        # RealSense depth is uint16 millimetres
        depth_m = depth.astype(np.float32) / 1000.0

        h, w = bgr.shape[:2]

        # ── Detect persons ────────────────────────────────────────────────
        results    = self.model(bgr, classes=[0], verbose=False, imgsz=320)[0]
        boxes_xyxy = []
        if results.boxes is not None and len(results.boxes):
            for box in results.boxes.xyxy.cpu().numpy():
                x1, y1, x2, y2 = [int(v) for v in box]
                x1, y1 = max(0, x1), max(0, y1)
                x2, y2 = min(w - 1, x2), min(h - 1, y2)
                boxes_xyxy.append((x1, y1, x2, y2))

        # ── Update centroid tracker ────────────────────────────────────────
        tracked = self.tracker.update(boxes_xyxy)   # {tracker_id: centroid}

        # Map tracker centroid → bbox (nearest centroid match)
        id_to_box: dict[int, tuple] = {}
        for tid, centroid in tracked.items():
            best_dist, best_box = float('inf'), None
            for box in boxes_xyxy:
                cx = (box[0] + box[2]) / 2
                cy = (box[1] + box[3]) / 2
                d  = math.hypot(cx - centroid[0], cy - centroid[1])
                if d < best_dist:
                    best_dist, best_box = d, box
            if best_box is not None and best_dist < 60:
                id_to_box[tid] = best_box

        # ── Compute depth for each detection ──────────────────────────────
        id_to_depth: dict[int, float] = {}
        for tid, box in id_to_box.items():
            d = self._median_depth(depth_m, box)
            if d is not None:
                id_to_depth[tid] = d

        # ── Target acquisition (first time or after full loss) ─────────────
        if self.target_hist is None and id_to_depth:
            # Pick the nearest visible person
            nearest_tid = min(id_to_depth, key=lambda k: id_to_depth[k])
            nearest_box = id_to_box[nearest_tid]
            self.target_hist = self._compute_shirt_hist(bgr, nearest_box)
            self.target_id   = nearest_tid
            self.get_logger().info(
                f'TARGET ACQUIRED — tracker_id={nearest_tid}, '
                f'depth={id_to_depth[nearest_tid]:.2f} m. '
                f'Shirt-colour signature stored.'
            )

        if self.target_hist is None:
            # Nothing to track yet
            self._publish_debug(bgr, id_to_box, {}, rgb_msg.header)
            return

        # ── Match target across frames ────────────────────────────────────
        matched_tid  = None
        matched_box  = None
        matched_depth = None
        status_str   = 'TRACKING'

        # 1. Try to keep the same tracker ID (smooth tracking, no histogram needed)
        if self.target_id in id_to_box and self.reid_cooldown > 0:
            matched_tid   = self.target_id
            matched_box   = id_to_box[self.target_id]
            matched_depth = id_to_depth.get(self.target_id)
            self.reid_cooldown -= 1

        # 2. Re-ID by shirt-colour histogram similarity
        else:
            best_sim, best_tid = -1.0, None
            for tid, box in id_to_box.items():
                hist = self._compute_shirt_hist(bgr, box)
                sim  = self._hist_cosine(self.target_hist, hist)
                if sim > best_sim:
                    best_sim, best_tid = sim, tid

            if best_sim >= self.reid_thresh:
                matched_tid    = best_tid
                matched_box    = id_to_box[best_tid]
                matched_depth  = id_to_depth.get(best_tid)
                status_str     = 'REIDENTIFIED' if best_tid != self.target_id else 'TRACKING'
                self.target_id = best_tid
                # Refresh signature with a rolling blend
                new_hist = self._compute_shirt_hist(bgr, matched_box)
                self.target_hist = 0.85 * self.target_hist + 0.15 * new_hist
                self.target_hist /= (self.target_hist.sum() + 1e-9)
                self.reid_cooldown = REID_COOLDOWN

        # ── If no match → publish nothing ────────────────────────────────
        if matched_tid is None or matched_depth is None:
            self._publish_debug(bgr, id_to_box, {}, rgb_msg.header)
            return

        # ── Unproject depth to 3D camera frame ───────────────────────────
        cx_px = (matched_box[0] + matched_box[2]) / 2
        cy_px = (matched_box[1] + matched_box[3]) / 2
        xyz_cam = self._pixel_to_camera(cx_px, cy_px, matched_depth)
        if xyz_cam is None:
            return

        # ── Transform to base_frame ───────────────────────────────────────
        xy_base = self._transform_to_base(xyz_cam, rgb_msg.header.stamp)
        if xy_base is None:
            return

        # ── Smooth position ───────────────────────────────────────────────
        self.pose_buffer.append(xy_base)
        smooth_x = float(np.mean([p[0] for p in self.pose_buffer]))
        smooth_y = float(np.mean([p[1] for p in self.pose_buffer]))

        # ── Publish pose ──────────────────────────────────────────────────
        pose_msg = PoseStamped()
        pose_msg.header.stamp    = rgb_msg.header.stamp
        pose_msg.header.frame_id = self.base_frame
        pose_msg.pose.position.x = smooth_x
        pose_msg.pose.position.y = smooth_y
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.w = 1.0
        self.pose_pub.publish(pose_msg)

        # ── Publish status ────────────────────────────────────────────────
        status_msg      = String()
        status_msg.data = (
            f'{status_str} | uid={self.target_uid} | '
            f'x={smooth_x:.3f} y={smooth_y:.3f} | depth={matched_depth:.2f}m'
        )
        self.status_pub.publish(status_msg)

        # ── Debug image ───────────────────────────────────────────────────
        self._publish_debug(
            bgr, id_to_box,
            {matched_tid: (smooth_x, smooth_y, matched_depth, status_str)},
            rgb_msg.header
        )

    # =========================================================================
    # Depth helpers
    # =========================================================================

    def _median_depth(self, depth_m: np.ndarray, box: tuple) -> float | None:
        """Return robust median depth over a small central patch of the bbox."""
        x1, y1, x2, y2 = box
        bw, bh = x2 - x1, y2 - y1
        if bw < 10 or bh < 10:
            return None

        # Central 30 % of bbox to avoid background contamination
        mx1 = x1 + bw // 3
        mx2 = x2 - bw // 3
        my1 = y1 + bh // 3
        my2 = y2 - bh // 3
        patch = depth_m[my1:my2, mx1:mx2]
        valid = patch[(patch > MIN_DEPTH_M) & (patch < MAX_DEPTH_M)]
        if valid.size < 5:
            return None
        return float(np.median(valid))

    # =========================================================================
    # Shirt colour helpers
    # =========================================================================

    def _shirt_roi(self, bbox: tuple) -> tuple[int, int, int, int]:
        x1, y1, x2, y2 = bbox
        bh = y2 - y1
        sy1 = y1 + int(bh * self.shirt_top)
        sy2 = y1 + int(bh * self.shirt_bot)
        return x1, sy1, x2, sy2

    def _compute_shirt_hist(self, bgr: np.ndarray, bbox: tuple) -> np.ndarray:
        x1, sy1, x2, sy2 = self._shirt_roi(bbox)
        roi = bgr[sy1:sy2, x1:x2]
        if roi.size == 0:
            return np.zeros(HIST_BINS[0] * HIST_BINS[1] * HIST_BINS[2], dtype=np.float32)
        hsv  = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        hist = cv2.calcHist(
            [hsv], [0, 1, 2], None, HIST_BINS, HIST_RANGES
        ).flatten().astype(np.float32)
        total = hist.sum()
        if total > 0:
            hist /= total
        return hist

    @staticmethod
    def _hist_cosine(a: np.ndarray, b: np.ndarray) -> float:
        na = np.linalg.norm(a)
        nb = np.linalg.norm(b)
        if na < 1e-9 or nb < 1e-9:
            return 0.0
        return float(np.dot(a, b) / (na * nb))

    # =========================================================================
    # Geometry helpers
    # =========================================================================

    def _pixel_to_camera(self, u: float, v: float, depth_m: float) -> np.ndarray | None:
        """Unproject (u, v, depth) → 3-D point in camera optical frame."""
        k  = self.camera_info.k
        fx, fy = k[0], k[4]
        cx, cy = k[2], k[5]
        if fx == 0 or fy == 0:
            return None
        x = (u - cx) * depth_m / fx
        y = (v - cy) * depth_m / fy
        z = depth_m
        return np.array([x, y, z], dtype=float)

    def _transform_to_base(self, xyz_cam: np.ndarray, stamp) -> tuple[float, float] | None:
        """Transform camera-frame 3D point to base_frame, return (x, y)."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.camera_frame,
                stamp,
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except TransformException as e:
            # Fallback: try latest available transform
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    self.camera_frame,
                    rclpy.time.Time()
                )
            except TransformException:
                self.get_logger().warn(f'TF unavailable: {e}', throttle_duration_sec=5.0)
                return None

        pose = PoseStamped()
        pose.header.frame_id       = self.camera_frame
        pose.pose.position.x       = float(xyz_cam[0])
        pose.pose.position.y       = float(xyz_cam[1])
        pose.pose.position.z       = float(xyz_cam[2])
        pose.pose.orientation.w    = 1.0

        transformed = do_transform_pose(pose, tf)
        return (transformed.pose.position.x, transformed.pose.position.y)

    # =========================================================================
    # Debug visualisation
    # =========================================================================

    def _publish_debug(
        self,
        bgr: np.ndarray,
        id_to_box: dict,
        target_info: dict,   # {tid: (x, y, depth, status)}
        header
    ):
        if not self.pub_debug:
            return

        vis = bgr.copy()

        # Draw all detections in grey
        for tid, box in id_to_box.items():
            x1, y1, x2, y2 = box
            cv2.rectangle(vis, (x1, y1), (x2, y2), (100, 100, 100), 1)

        # Draw target in green with info overlay
        for tid, (tx, ty, depth, status) in target_info.items():
            box = id_to_box.get(tid)
            if box is None:
                continue
            x1, y1, x2, y2 = box

            # Bbox
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Shirt ROI strip
            _, sy1, _, sy2 = self._shirt_roi(box)
            cv2.rectangle(vis, (x1, sy1), (x2, sy2), (0, 200, 255), 1)

            # Labels
            label = f'UID:{self.target_uid} {status}'
            coord = f'x={tx:.2f}m y={ty:.2f}m d={depth:.2f}m'
            cv2.rectangle(vis, (x1, y1 - 38), (x1 + 220, y1), (0, 255, 0), -1)
            cv2.putText(vis, label, (x1 + 2, y1 - 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)
            cv2.putText(vis, coord, (x1 + 2, y1 - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 0, 0), 1)

        msg = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
        msg.header = header
        self.debug_pub.publish(msg)


# ─── Entry point ─────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = PersonTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
