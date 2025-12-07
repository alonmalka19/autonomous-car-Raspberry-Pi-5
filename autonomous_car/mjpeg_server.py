#!/usr/bin/env python3
"""
MJPEG Server Node - Streams /detector/debug_image as HTTP MJPEG
Access from browser: http://192.168.0.104:8080/video_feed
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from flask import Flask, Response
import threading

app = Flask(__name__)
latest_frame = None
frame_lock = threading.Lock()


class MJPEGServerNode(Node):
    def __init__(self):
        super().__init__('mjpeg_server')

        self.bridge = CvBridge()

        # Subscribe to debug image
        self.image_sub = self.create_subscription(
            Image, '/detector/debug_image', self.image_callback, 1)

        self.get_logger().info('MJPEG Server started')
        self.get_logger().info('Open browser: http://192.168.0.104:8080/video_feed')

    def image_callback(self, msg):
        global latest_frame
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # Encode as JPEG (quality 80 for balance)
            ret, jpeg = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 80])

            if ret:
                with frame_lock:
                    latest_frame = jpeg.tobytes()
        except Exception as e:
            self.get_logger().error(f'Error encoding frame: {e}')


def generate_frames():
    """Generator for MJPEG stream"""
    while True:
        with frame_lock:
            if latest_frame is not None:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + latest_frame + b'\r\n')


@app.route('/video_feed')
def video_feed():
    """HTTP endpoint for MJPEG stream"""
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/')
def index():
    """Simple HTML page with video player"""
    return '''
    <html>
    <head><title>Autonomous Car - Live Feed</title></head>
    <body style="background: #000; margin: 0;">
        <h1 style="color: #fff; text-align: center;">🚗 Autonomous Car - Live Feed</h1>
        <div style="text-align: center;">
            <img src="/video_feed" style="max-width: 100%; height: auto;">
        </div>
        <p style="color: #fff; text-align: center;">No buffering, no lag - pure MJPEG stream</p>
    </body>
    </html>
    '''


def main(args=None):
    rclpy.init(args=args)
    node = MJPEGServerNode()

    # Start Flask in separate thread
    flask_thread = threading.Thread(
        target=lambda: app.run(host='0.0.0.0', port=8080, threaded=True),
        daemon=True
    )
    flask_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
