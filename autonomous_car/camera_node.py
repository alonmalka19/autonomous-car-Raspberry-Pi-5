#!/usr/bin/env python3
"""
Camera Node for Autonomous Car
Supports both IP Webcam (HTTP) and Record3D (USB) integration
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import requests
import threading
import time
from record3d import Record3DStream


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Parameters
        self.declare_parameter('camera_type', 'ip_webcam')  # ip_webcam / record3d
        self.declare_parameter('camera_url', 'http://192.168.1.116:8080/shot.jpg')
        self.declare_parameter('camera_quality', 60)
        self.declare_parameter('max_frame_width', 640)
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('publish_depth', False)  # Not available for IP Webcam
        self.declare_parameter('rotate_90', False)  # Rotate 90 degrees for landscape mode

        self.camera_type = self.get_parameter('camera_type').value
        self.camera_url = self.get_parameter('camera_url').value
        self.quality = self.get_parameter('camera_quality').value
        self.max_width = self.get_parameter('max_frame_width').value
        self.rate = self.get_parameter('publish_rate').value
        self.publish_depth = self.get_parameter('publish_depth').value
        self.rotate_90 = self.get_parameter('rotate_90').value

        # QoS for real-time sensor streaming - CRITICAL for low latency
        # Based on ROS 2 SensorDataQoS best practices
        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,  # Only keep latest frame - prevents buffer buildup
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Drop frames if slow, don't retry
            durability=QoSDurabilityPolicy.VOLATILE  # No persistence
        )

        # Publishers with sensor QoS for real-time streaming
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', sensor_qos)
        self.depth_pub = self.create_publisher(Image, '/camera/depth', sensor_qos)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)

        # CV Bridge
        self.bridge = CvBridge()

        # Record3D state variables
        self.stream_session = None
        self.new_frame_event = threading.Event()
        self.stream_lock = threading.Lock()
        self.latest_rgb = None
        self.latest_depth = None

        # Initialize camera based on type
        if self.camera_type == 'record3d':
            self._init_record3d()
        else:
            self._init_ip_webcam()

        # Timer for publishing
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)

        self.get_logger().info(f'Camera node started with {self.camera_type}')

    def _init_ip_webcam(self):
        """Initialize IP Webcam (HTTP streaming)"""
        self.get_logger().info(f'Using IP Webcam: {self.camera_url}')
        self.get_logger().warn('Depth data not available with IP Webcam')

    def _init_record3d(self):
        """Initialize Record3D (USB streaming) - REAL IMPLEMENTATION"""
        try:
            self.get_logger().info('🔍 Searching for iPhone devices...')
            devs = Record3DStream.get_connected_devices()

            if not devs:
                self.get_logger().error('❌ No iPhone found! Make sure:')
                self.get_logger().error('  1. iPhone connected via USB')
                self.get_logger().error('  2. Record3D app running')
                self.get_logger().error('  3. USB Streaming enabled in app')
                raise RuntimeError('No Record3D device found')

            self.get_logger().info(f'🔗 Connecting to device {devs[0].product_id}...')
            self.stream_session = Record3DStream()
            self.stream_session.on_new_frame = self._on_new_frame
            self.stream_session.connect(devs[0])

            time.sleep(2)  # Wait for camera to stabilize

            self.get_logger().info('✅ Record3D connected successfully!')

        except Exception as e:
            self.get_logger().error(f'Failed to initialize Record3D: {str(e)}')
            self.get_logger().info('Falling back to IP Webcam...')
            self.camera_type = 'ip_webcam'

    def _on_new_frame(self):
        """Callback when Record3D receives new frame"""
        with self.stream_lock:
            self.latest_rgb = self.stream_session.get_rgb_frame()
            if self.publish_depth:
                self.latest_depth = self.stream_session.get_depth_frame()
        self.new_frame_event.set()

    def timer_callback(self):
        if self.camera_type == 'ip_webcam':
            self._capture_ip_webcam()
        elif self.camera_type == 'record3d':
            self._capture_record3d()

    def _capture_ip_webcam(self):
        """Capture frame from IP Webcam"""
        try:
            url = f"{self.camera_url}?quality={self.quality}"
            resp = requests.get(url, timeout=0.5)
            img_arr = np.frombuffer(resp.content, dtype=np.uint8)
            frame = cv2.imdecode(img_arr, cv2.IMREAD_COLOR)

            if frame is None:
                self.get_logger().warn('Failed to decode frame')
                return

            # Resize if needed
            if frame.shape[1] > self.max_width:
                h, w = frame.shape[:2]
                scale = self.max_width / w
                frame = cv2.resize(frame, (self.max_width, int(h * scale)))

            # Convert to ROS Image message
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = 'camera_frame'

            # Publish
            self.image_pub.publish(image_msg)

        except Exception as e:
            self.get_logger().error(f'Camera error: {str(e)}')

    def _capture_record3d(self):
        """Capture frame from Record3D - OPTIMIZED for low latency"""
        try:
            # Don't wait - just check if frame is available (non-blocking)
            if not self.new_frame_event.is_set():
                return  # No new frame, skip this cycle

            with self.stream_lock:
                if self.latest_rgb is None or self.latest_rgb.size == 0:
                    return

                # Use the frame directly without copy when possible
                rgb = self.latest_rgb
                depth = self.latest_depth

            self.new_frame_event.clear()

            # Rotate 90 degrees if enabled (portrait to landscape)
            if self.rotate_90:
                rgb = cv2.rotate(rgb, cv2.ROTATE_90_COUNTERCLOCKWISE)
                if depth is not None:
                    depth = cv2.rotate(depth, cv2.ROTATE_90_COUNTERCLOCKWISE)

            # Resize RGB if needed
            if rgb.shape[1] > self.max_width:
                h, w = rgb.shape[:2]
                scale = self.max_width / w
                new_size = (self.max_width, int(h * scale))
                rgb = cv2.resize(rgb, new_size, interpolation=cv2.INTER_LINEAR)
                if depth is not None:
                    depth = cv2.resize(depth, new_size, interpolation=cv2.INTER_NEAREST)

            # Convert RGB to BGR for ROS
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

            # Timestamp
            timestamp = self.get_clock().now().to_msg()

            # Publish RGB image
            image_msg = self.bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
            image_msg.header.stamp = timestamp
            image_msg.header.frame_id = 'camera_frame'
            self.image_pub.publish(image_msg)

            # Publish Depth image (if enabled)
            if self.publish_depth and depth is not None:
                depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding='32FC1')
                depth_msg.header.stamp = timestamp
                depth_msg.header.frame_id = 'camera_frame'
                self.depth_pub.publish(depth_msg)

        except Exception as e:
            self.get_logger().error(f'Record3D capture error: {str(e)}')

    def destroy_node(self):
        """Clean up resources"""
        if self.stream_session is not None:
            try:
                self.stream_session.disconnect()
                self.get_logger().info('Record3D disconnected')
            except Exception as e:
                self.get_logger().warn(f'Error disconnecting Record3D: {str(e)}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
