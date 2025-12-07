#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from gpiozero import Motor
import time
import threading
import queue
import numpy as np


class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')

        # Parameters (swapped left/right to fix direction)
        self.declare_parameter('left_backward_pin', 27)
        self.declare_parameter('left_forward_pin', 22)
        self.declare_parameter('right_backward_pin', 23)
        self.declare_parameter('right_forward_pin', 24)
        self.declare_parameter('stop_distance', 0.6)  # meters - stop when target is 60cm away
        self.declare_parameter('left_zone', 0.25)
        self.declare_parameter('right_zone', 0.75)
        self.declare_parameter('turn_pulse', 0.05)  # For target tracking
        self.declare_parameter('obstacle_turn_pulse', 0.20)  # For obstacle avoidance (longer)
        self.declare_parameter('search_turn_pulse', 0.30)  # For searching when target lost
        self.declare_parameter('frame_width', 640)

        # Motor speed limit for 3-battery setup (12V protection)
        self.declare_parameter('max_speed', 0.65)  # 65% of full power (safe for 12V)
        self.declare_parameter('left_speed_factor', 1.0)  # Left motor adjustment
        self.declare_parameter('right_speed_factor', 1.0)  # Right motor adjustment
        self.max_speed = self.get_parameter('max_speed').value
        self.left_speed_factor = self.get_parameter('left_speed_factor').value
        self.right_speed_factor = self.get_parameter('right_speed_factor').value

        # GPIO setup with PWM enabled
        self.left_motor = Motor(
            forward=self.get_parameter('left_forward_pin').value,
            backward=self.get_parameter('left_backward_pin').value,
            pwm=True  # Enable PWM for speed control
        )
        self.right_motor = Motor(
            forward=self.get_parameter('right_forward_pin').value,
            backward=self.get_parameter('right_backward_pin').value,
            pwm=True  # Enable PWM for speed control
        )

        self.stop_distance = self.get_parameter('stop_distance').value  # meters
        self.left_zone = self.get_parameter('left_zone').value
        self.right_zone = self.get_parameter('right_zone').value
        self.turn_pulse = self.get_parameter('turn_pulse').value
        self.obstacle_turn_pulse = self.get_parameter('obstacle_turn_pulse').value
        self.search_turn_pulse = self.get_parameter('search_turn_pulse').value
        self.frame_width = self.get_parameter('frame_width').value

        # Obstacle detection parameter
        self.declare_parameter('obstacle_distance', 0.5)
        self.obstacle_distance = self.get_parameter('obstacle_distance').value

        # Motor command queue
        self.motor_queue = queue.Queue(maxsize=10)
        self.motor_worker_running = True
        self.motor_thread = threading.Thread(target=self._motor_worker, daemon=True)
        self.motor_thread.start()

        # Depth sensor for obstacle detection
        self.bridge = CvBridge()
        self.latest_depth = None
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth', self.depth_callback, 10)

        # Subscribers
        self.position_sub = self.create_subscription(
            Point, '/target/position', self.position_callback, 10)
        self.status_sub = self.create_subscription(
            String, '/target/tracking_status', self.status_callback, 10)

        # Publisher (optional - for monitoring)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Search cooldown - prevent spamming search commands
        self.last_search_time = 0
        self.search_cooldown = 1.0  # seconds between search commands

        self.get_logger().info('Motor node started with gpiozero')

    def depth_callback(self, msg):
        """Store latest depth image for obstacle detection"""
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().error(f'Depth conversion error: {str(e)}')

    def _check_obstacle(self, target_distance=None):
        """Check for obstacles in front using LiDAR depth data

        Args:
            target_distance: Distance to the target we're tracking (to ignore it)
        """
        if self.latest_depth is None:
            return None, None

        try:
            h, w = self.latest_depth.shape
            # Check center region (middle 40% width, bottom 50% height)
            center_left = int(w * 0.3)
            center_right = int(w * 0.7)
            top = int(h * 0.5)

            center_region = self.latest_depth[top:, center_left:center_right]
            valid_depths = center_region[(center_region > 0.1) & (center_region < 5.0)]

            if len(valid_depths) == 0:
                return None, None

            min_distance = np.min(valid_depths)

            # IGNORE the target we're tracking! Only detect OTHER obstacles
            if target_distance is not None and abs(min_distance - target_distance) < 0.5:
                # This is probably the target, not an obstacle - ignore it
                return None, None

            # Check if obstacle is too close
            if min_distance < self.obstacle_distance:
                # Determine which side is more clear
                left_region = self.latest_depth[top:, :center_left]
                right_region = self.latest_depth[top:, center_right:]

                left_valid = left_region[(left_region > 0.1) & (left_region < 5.0)]
                right_valid = right_region[(right_region > 0.1) & (right_region < 5.0)]

                left_avg = np.mean(left_valid) if len(left_valid) > 0 else 5.0
                right_avg = np.mean(right_valid) if len(right_valid) > 0 else 5.0

                # Return obstacle detected and preferred avoidance direction
                avoid_direction = "left" if right_avg > left_avg else "right"
                return min_distance, avoid_direction

            return None, None

        except Exception as e:
            self.get_logger().error(f'Obstacle check error: {str(e)}')
            return None, None

    def _motor_worker(self):
        while self.motor_worker_running:
            try:
                command, duration = self.motor_queue.get(timeout=0.3)
            except queue.Empty:
                continue

            try:
                if command == "stop":
                    self._stop_motors()
                elif command == "forward":
                    self._forward()
                    if duration > 0:
                        time.sleep(duration)
                        self._stop_motors()
                elif command == "left":
                    self._turn_left(duration)
                elif command == "right":
                    self._turn_right(duration)
            except Exception as e:
                self.get_logger().error(f'Motor execution error: {str(e)}')

    def _stop_motors(self):
        self.left_motor.stop()
        self.right_motor.stop()

    def _forward(self):
        left_speed = self.max_speed * self.left_speed_factor
        right_speed = self.max_speed * self.right_speed_factor
        self.left_motor.forward(speed=left_speed)
        self.right_motor.forward(speed=right_speed)

    def _turn_left(self, duration):
        self.left_motor.forward(speed=self.max_speed)
        self.right_motor.backward(speed=self.max_speed)
        time.sleep(duration)
        self._stop_motors()

    def _turn_right(self, duration):
        self.left_motor.backward(speed=self.max_speed)
        self.right_motor.forward(speed=self.max_speed)
        time.sleep(duration)
        self._stop_motors()

    def _send_motor_command(self, command, duration=0.0):
        try:
            self.motor_queue.put_nowait((command, duration))
        except queue.Full:
            self.get_logger().warn(f'Motor queue full, dropping: {command}')

    def position_callback(self, msg):
        cx = msg.x
        distance = msg.z  # REAL distance in meters from LiDAR!

        # PRIORITY 1: Check for obstacles FIRST (but ignore the target itself!)
        obstacle_dist, avoid_direction = self._check_obstacle(target_distance=distance)

        if obstacle_dist is not None:
            # Obstacle detected! Override target tracking for safety
            self.get_logger().warn(
                f'OBSTACLE detected at {obstacle_dist:.2f}m - avoiding {avoid_direction}')

            if avoid_direction == "left":
                self._send_motor_command("left", self.obstacle_turn_pulse)
            else:
                self._send_motor_command("right", self.obstacle_turn_pulse)
            return  # Skip target tracking logic

        # PRIORITY 2: Check if too close to target - STOP
        if distance > 0 and distance < self.stop_distance:
            self._send_motor_command("stop")
            self.get_logger().info(f'Target TOO CLOSE ({distance:.2f}m < {self.stop_distance}m) - STOPPING')
            return

        # PRIORITY 3: Normal target tracking
        if cx < self.frame_width * self.left_zone:
            # Turn left toward target
            self._send_motor_command("left", self.turn_pulse)
            self.get_logger().info(f'Target LEFT (dist: {distance:.2f}m)')

        elif cx > self.frame_width * self.right_zone:
            # Turn right toward target
            self._send_motor_command("right", self.turn_pulse)
            self.get_logger().info(f'Target RIGHT (dist: {distance:.2f}m)')

        else:
            # Move forward toward target
            self._send_motor_command("forward")
            self.get_logger().info(f'Target CENTERED - FORWARD (dist: {distance:.2f}m)')

    def status_callback(self, msg):
        status = msg.data

        if status.startswith("lost:"):
            # Check cooldown - don't spam search commands
            current_time = time.time()
            if current_time - self.last_search_time < self.search_cooldown:
                return  # Skip, still in cooldown

            self.last_search_time = current_time
            direction = status.split(":")[1]
            self.get_logger().warn(f'Target lost - searching {direction}')

            # Search pattern - turn toward where target was last seen
            if direction == "right":
                self._send_motor_command("right", self.search_turn_pulse)
            else:
                self._send_motor_command("left", self.search_turn_pulse)

    def destroy_node(self):
        self.motor_worker_running = False
        self._stop_motors()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
