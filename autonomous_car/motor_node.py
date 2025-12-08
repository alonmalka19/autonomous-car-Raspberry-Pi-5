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


class PIDController:
    """PID Controller for smooth tracking"""
    def __init__(self, kp=1.0, ki=0.0, kd=0.1, min_output=-1.0, max_output=1.0):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        self.min_output = min_output
        self.max_output = max_output

        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()

    def compute(self, error):
        """Compute PID output based on error"""
        current_time = time.time()
        dt = current_time - self.last_time

        if dt <= 0:
            dt = 0.01  # Minimum time step

        # Proportional term
        p_term = self.kp * error

        # Integral term (with anti-windup)
        self.integral += error * dt
        self.integral = max(-1.0, min(1.0, self.integral))  # Clamp integral
        i_term = self.ki * self.integral

        # Derivative term
        derivative = (error - self.prev_error) / dt
        d_term = self.kd * derivative

        # Calculate output
        output = p_term + i_term + d_term
        output = max(self.min_output, min(self.max_output, output))

        # Store for next iteration
        self.prev_error = error
        self.last_time = current_time

        return output

    def reset(self):
        """Reset PID state"""
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()


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
        self.declare_parameter('turn_pulse', 0.05)  # For target tracking (fallback)
        self.declare_parameter('obstacle_turn_pulse', 0.20)  # For obstacle avoidance (longer)
        self.declare_parameter('search_turn_pulse', 0.30)  # For searching when target lost
        self.declare_parameter('frame_width', 640)

        # Motor speed limit for 3-battery setup (12V protection)
        self.declare_parameter('max_speed', 0.65)  # 65% of full power (safe for 12V)
        self.declare_parameter('min_speed', 0.3)  # Minimum speed when target is close
        self.declare_parameter('left_speed_factor', 1.0)  # Left motor adjustment
        self.declare_parameter('right_speed_factor', 1.0)  # Right motor adjustment

        # PID parameters for smooth steering
        self.declare_parameter('pid_kp', 1.5)  # Proportional gain
        self.declare_parameter('pid_ki', 0.05)  # Integral gain
        self.declare_parameter('pid_kd', 0.3)  # Derivative gain

        # Dynamic speed parameters
        self.declare_parameter('speed_distance_factor', 0.5)  # How much distance affects speed
        self.declare_parameter('far_distance', 3.0)  # Distance considered "far" (full speed)

        self.max_speed = self.get_parameter('max_speed').value
        self.min_speed = self.get_parameter('min_speed').value
        self.left_speed_factor = self.get_parameter('left_speed_factor').value
        self.right_speed_factor = self.get_parameter('right_speed_factor').value

        # Initialize PID controller for steering
        self.steering_pid = PIDController(
            kp=self.get_parameter('pid_kp').value,
            ki=self.get_parameter('pid_ki').value,
            kd=self.get_parameter('pid_kd').value,
            min_output=-1.0,
            max_output=1.0
        )

        self.speed_distance_factor = self.get_parameter('speed_distance_factor').value
        self.far_distance = self.get_parameter('far_distance').value

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
                command, duration, steering, speed, pivot = self.motor_queue.get(timeout=0.3)
            except queue.Empty:
                continue

            try:
                if command == "stop":
                    self._stop_motors()
                    self.steering_pid.reset()  # Reset PID when stopping
                elif command == "forward":
                    self._forward()
                    if duration > 0:
                        time.sleep(duration)
                        self._stop_motors()
                elif command == "left":
                    self._turn_left(duration)
                elif command == "right":
                    self._turn_right(duration)
                elif command == "smooth":
                    # Smooth driving command with PID (with optional pivot)
                    self._smooth_drive(steering, speed, pivot)
            except Exception as e:
                self.get_logger().error(f'Motor execution error: {str(e)}')

    def _stop_motors(self):
        self.left_motor.stop()
        self.right_motor.stop()

    def _forward(self, speed=None):
        """Move forward at specified speed (or max_speed if not specified)"""
        if speed is None:
            speed = self.max_speed
        left_speed = speed * self.left_speed_factor
        right_speed = speed * self.right_speed_factor
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

    def _smooth_drive(self, steering, speed, pivot=False):
        """
        Smooth driving with differential steering using PID output.

        Args:
            steering: -1.0 (full left) to 1.0 (full right), 0 = straight
            speed: Base speed (0.0 to 1.0)
            pivot: If True, use pivot turn (one wheel forward, one backward)
        """
        if pivot:
            # Pivot turn - one wheel forward, one backward (for searching)
            turn_speed = speed  # Full speed for pivot (no reduction)
            if steering > 0:
                # Turn right: right forward, left backward
                self.left_motor.backward(speed=turn_speed * self.left_speed_factor)
                self.right_motor.forward(speed=turn_speed * self.right_speed_factor)
            else:
                # Turn left: left forward, right backward
                self.left_motor.forward(speed=turn_speed * self.left_speed_factor)
                self.right_motor.backward(speed=turn_speed * self.right_speed_factor)
            return

        # Normal differential steering for tracking
        # steering > 0 means turn right (left wheel faster)
        # steering < 0 means turn left (right wheel faster)

        left_speed = speed * (1.0 + steering)
        right_speed = speed * (1.0 - steering)

        # Clamp speeds to valid range
        left_speed = max(0.0, min(1.0, left_speed))
        right_speed = max(0.0, min(1.0, right_speed))

        # Apply motor balance factors
        left_speed *= self.left_speed_factor
        right_speed *= self.right_speed_factor

        # Apply to motors
        if left_speed > 0:
            self.left_motor.forward(speed=left_speed)
        else:
            self.left_motor.stop()

        if right_speed > 0:
            self.right_motor.forward(speed=right_speed)
        else:
            self.right_motor.stop()

    def _calculate_dynamic_speed(self, distance):
        """
        Calculate speed based on distance to target.
        Closer = slower, farther = faster.
        """
        if distance <= 0:
            return self.max_speed

        # Normalize distance (0 = at stop_distance, 1 = at far_distance)
        normalized = (distance - self.stop_distance) / (self.far_distance - self.stop_distance)
        normalized = max(0.0, min(1.0, normalized))

        # Interpolate between min and max speed
        speed = self.min_speed + normalized * (self.max_speed - self.min_speed)

        return speed

    def _send_motor_command(self, command, duration=0.0, steering=0.0, speed=0.0, pivot=False):
        try:
            self.motor_queue.put_nowait((command, duration, steering, speed, pivot))
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

        # PRIORITY 2: Check if too close to target - STOP or PIVOT to follow
        if distance > 0 and distance < self.stop_distance:
            # Calculate how far from center the target is
            center_x = self.frame_width / 2.0
            error = (cx - center_x) / center_x  # -1 to 1

            # If target moved to the side, pivot to follow (don't just stop)
            if abs(error) > 0.3:  # Target is more than 30% off center
                pivot_speed = self.max_speed * 0.4  # Slow pivot when close
                if error > 0:
                    # Target moved right, pivot right
                    self._send_motor_command("smooth", steering=1.0, speed=pivot_speed, pivot=True)
                    self.get_logger().info(f'Target CLOSE but RIGHT - pivoting right')
                else:
                    # Target moved left, pivot left
                    self._send_motor_command("smooth", steering=-1.0, speed=pivot_speed, pivot=True)
                    self.get_logger().info(f'Target CLOSE but LEFT - pivoting left')
            else:
                # Target is centered and close - stop
                self._send_motor_command("stop")
                self.get_logger().info(f'Target TOO CLOSE ({distance:.2f}m < {self.stop_distance}m) - STOPPING')
            return

        # PRIORITY 3: SMOOTH PID-based target tracking

        # Calculate error: how far from center (-1 to 1)
        # 0 = center, negative = left, positive = right
        center_x = self.frame_width / 2.0
        error = (cx - center_x) / center_x  # Normalized error (-1 to 1)

        # Dead zone - if target is close to center, just go straight (no zigzag)
        dead_zone = 0.30  # 30% of frame width - large dead zone!
        if abs(error) < dead_zone:
            error = 0.0  # Treat as centered
            self.steering_pid.reset()  # Reset PID to prevent buildup

        # Get PID steering output (negate because of motor wiring)
        steering = -self.steering_pid.compute(error)

        # Calculate dynamic speed based on distance
        dynamic_speed = self._calculate_dynamic_speed(distance)

        # Reduce speed when turning sharply
        turn_speed_factor = 1.0 - (abs(steering) * 0.5)  # Up to 50% reduction when turning hard
        final_speed = dynamic_speed * turn_speed_factor

        # Send smooth drive command
        self._send_motor_command("smooth", steering=steering, speed=final_speed)

        self.get_logger().info(
            f'SMOOTH: error={error:.2f}, steer={steering:.2f}, speed={final_speed:.2f}, dist={distance:.2f}m'
        )

    def status_callback(self, msg):
        status = msg.data

        if status.startswith("lost:"):
            # Reset PID when target is lost
            self.steering_pid.reset()

            direction = status.split(":")[1]
            self.get_logger().warn(f'Target lost - pivot searching {direction}')

            # Pivot search - both wheels work (one forward, one backward)
            # Turn TOWARD the direction where target was last seen
            search_speed = self.max_speed * 0.65  # 65% power for pivot search
            if direction == "right":
                # Last seen on right, pivot RIGHT to find it
                self._send_motor_command("smooth", steering=1.0, speed=search_speed, pivot=True)
            else:
                # Last seen on left, pivot LEFT to find it
                self._send_motor_command("smooth", steering=-1.0, speed=search_speed, pivot=True)

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
