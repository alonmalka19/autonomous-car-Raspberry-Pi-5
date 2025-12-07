#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO


class DetectorNode(Node):
    def __init__(self):
        super().__init__('detector_node')

        # Parameters
        self.declare_parameter('model_path', 'models/best_mylegs_v5.pt')
        self.declare_parameter('confidence', 0.3)
        self.declare_parameter('inference_size', 192)
        self.declare_parameter('target_class', 'my_legs')
        self.declare_parameter('miss_limit', 3)

        model_path = self.get_parameter('model_path').value
        self.conf = self.get_parameter('confidence').value
        self.inference_size = self.get_parameter('inference_size').value
        self.target_class = self.get_parameter('target_class').value
        self.miss_limit = self.get_parameter('miss_limit').value

        # Load YOLO model
        self.model = YOLO(model_path)
        self.model.to('cpu')

        # Find target class ID
        self.target_class_id = self._find_class_id(self.target_class)

        # CV Bridge
        self.bridge = CvBridge()

        # QoS for real-time sensor streaming - CRITICAL for low latency
        # Must match camera publisher QoS (BEST_EFFORT) for compatibility
        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,  # Only keep latest frame - prevents buffer buildup
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Match camera publisher
            durability=QoSDurabilityPolicy.VOLATILE  # No persistence
        )

        # Subscribers - use sensor QoS to prevent frame accumulation
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, sensor_qos)

        # Publishers - debug_image also uses sensor QoS for Foxglove
        self.position_pub = self.create_publisher(Point, '/target/position', 10)
        self.status_pub = self.create_publisher(String, '/target/tracking_status', 10)
        self.debug_image_pub = self.create_publisher(Image, '/detector/debug_image', sensor_qos)

        # Depth subscriber - also use sensor QoS
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth', self.depth_callback, sensor_qos)
        self.latest_depth = None

        # Tracking state
        self.last_position = None
        self.prev_gray = None
        self.consecutive_misses = 0
        self.tracking_mode = None
        self.last_seen_direction = "left"

        # Frame skipping for low latency - only process latest frame
        self.pending_frame = None
        self.frame_lock = __import__('threading').Lock()
        self.processing = False

        self.get_logger().info(f'Detector node started with model: {model_path}')

    def _find_class_id(self, class_name):
        names = self.model.names
        for k, v in names.items():
            if class_name.lower() in v.lower():
                return k
        return 0  # fallback

    def depth_callback(self, msg):
        """Store latest depth image"""
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().error(f'Depth conversion error: {str(e)}')

    def image_callback(self, msg):
        # Skip frames if still processing previous one (prevents queue buildup)
        if self.processing:
            return

        try:
            self.processing = True

            # Convert ROS Image to OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            debug_frame = frame.copy()  # For visualization
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # YOLO inference
            results = self.model(frame, conf=self.conf, imgsz=self.inference_size, verbose=False)[0]
            boxes = results.boxes

            selected_box = None
            found_by_yolo = False

            # YOLO detection logic
            if boxes is not None:
                for box in boxes:
                    cls = int(box.cls[0])
                    if cls == self.target_class_id:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        cx = (x1 + x2) // 2
                        cy = (y1 + y2) // 2

                        # Update direction - store where target was last seen
                        self.last_seen_direction = "left" if cx < frame.shape[1] // 2 else "right"

                        # Track closest
                        if self.last_position is not None:
                            dist = np.linalg.norm(self.last_position[0] - [cx, cy])
                            selected_box = (x1, y1, x2, y2)
                            self.last_position = np.array([[cx, cy]], dtype=np.float32)
                        else:
                            self.last_position = np.array([[cx, cy]], dtype=np.float32)
                            selected_box = (x1, y1, x2, y2)

                        self.tracking_mode = "yolo"
                        found_by_yolo = True
                        break

            if found_by_yolo:
                self.consecutive_misses = 0
            else:
                self.consecutive_misses += 1

            # Optical flow fallback
            if not found_by_yolo and self.consecutive_misses < self.miss_limit:
                if self.last_position is not None and self.prev_gray is not None:
                    new_pos, status, _ = cv2.calcOpticalFlowPyrLK(
                        self.prev_gray, gray, self.last_position, None,
                        winSize=(15, 15), maxLevel=2)

                    if status[0][0] == 1:
                        self.last_position = new_pos
                        cx, cy = int(new_pos[0][0]), int(new_pos[0][1])
                        selected_box = (cx-50, cy-80, cx+50, cy+80)
                        self.tracking_mode = "flow"

            # Publish results
            if selected_box:
                x1, y1, x2, y2 = selected_box
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2
                area = (x2 - x1) * (y2 - y1)

                # Draw bounding box on debug frame
                cv2.rectangle(debug_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(debug_frame, (cx, cy), 5, (0, 0, 255), -1)

                # Calculate distance from depth (NEW!)
                distance = -1.0  # default if no depth available
                if self.latest_depth is not None:
                    try:
                        # Sample depth in center of bounding box
                        depth_h, depth_w = self.latest_depth.shape
                        if 0 <= cx < depth_w and 0 <= cy < depth_h:
                            # Average depth in 5x5 patch around center
                            patch_size = 5
                            x_start = max(0, cx - patch_size // 2)
                            x_end = min(depth_w, cx + patch_size // 2 + 1)
                            y_start = max(0, cy - patch_size // 2)
                            y_end = min(depth_h, cy + patch_size // 2 + 1)

                            depth_patch = self.latest_depth[y_start:y_end, x_start:x_end]
                            valid_depths = depth_patch[depth_patch > 0]  # filter invalid

                            if len(valid_depths) > 0:
                                distance = float(np.median(valid_depths))  # meters
                    except Exception as e:
                        self.get_logger().warn(f'Depth extraction error: {str(e)}')

                # Use bounding box ONLY as safety override when box is VERY large
                box_height = y2 - y1
                frame_height = frame.shape[0]
                box_ratio = box_height / frame_height

                # Only override LiDAR if box is exceptionally large (person very close)
                if box_ratio > 0.7:
                    # Box is huge - person is definitely very close, override LiDAR
                    distance = 0.15
                elif box_ratio > 0.6 and (distance <= 0 or distance > 0.5):
                    # Box is very large and LiDAR seems wrong
                    distance = 0.20
                # Otherwise trust LiDAR distance

                # Draw distance and tracking mode on debug frame
                text = f"{self.tracking_mode} | {distance:.2f}m"
                cv2.putText(debug_frame, text, (x1, y1 - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                # Publish position with distance (LiDAR or estimated)
                pos_msg = Point()
                pos_msg.x = float(cx)
                pos_msg.y = float(cy)
                pos_msg.z = distance
                self.position_pub.publish(pos_msg)

                # Publish status
                status_msg = String()
                status_msg.data = f"tracking:{self.tracking_mode}"
                self.status_pub.publish(status_msg)

            elif self.consecutive_misses >= self.miss_limit:
                # Lost tracking - draw text
                cv2.putText(debug_frame, f"LOST - searching {self.last_seen_direction}",
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

                status_msg = String()
                status_msg.data = f"lost:{self.last_seen_direction}"
                self.status_pub.publish(status_msg)

            # Publish debug image with current timestamp
            debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
            debug_msg.header.stamp = self.get_clock().now().to_msg()
            debug_msg.header.frame_id = 'camera_frame'
            self.debug_image_pub.publish(debug_msg)

            # Store gray for optical flow (no copy needed - we're done with it)
            self.prev_gray = gray

        except Exception as e:
            self.get_logger().error(f'Detector error: {str(e)}')
        finally:
            self.processing = False


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
