from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Camera Node - ULTRA LOW LATENCY settings
    # Lower resolution + higher FPS = less delay
    camera_node = Node(
        package='autonomous_car',
        executable='camera_node',
        name='camera',
        parameters=[{
            'camera_type': 'record3d',
            'max_frame_width': 240,   # Lower resolution for faster processing
            'publish_rate': 15.0,     # 15 FPS - faster updates
            'publish_depth': True,    # Enable depth for distance tracking
            'rotate_90': True         # Rotate to landscape mode
        }],
        output='screen'
    )

    # Detector Node - Using PyTorch YOLO
    detector_node = Node(
        package='autonomous_car',
        executable='detector_node',
        name='detector',
        parameters=[{
            'model_path': '/home/alonmalka/ros2_ws/src/autonomous_car/models/best_mylegs_v5.pt',
            'confidence': 0.2,
            'inference_size': 192,
            'target_class': 'my_legs',
            'miss_limit': 3,

            # Motion prediction settings
            'prediction_enabled': True,        # Enable position prediction
            'prediction_lookahead': 0.1,       # Predict 100ms ahead
            'prediction_history_size': 10,     # Keep last 10 positions
            'prediction_min_velocity': 5.0     # Min velocity to trigger prediction (pixels/sec)
        }],
        output='screen'
    )

    # Motor Node - With PID smooth tracking and dynamic speed
    motor_node = Node(
        package='autonomous_car',
        executable='motor_node',
        name='motors',
        parameters=[{
            # GPIO pins
            'left_backward_pin': 23,
            'left_forward_pin': 24,
            'right_backward_pin': 27,
            'right_forward_pin': 22,

            # Distance settings
            'stop_distance': 0.6,  # Stop when target is 60cm away
            'far_distance': 3.0,   # Distance considered "far" (full speed)

            # Zone thresholds (fallback for obstacle avoidance)
            'left_zone': 0.25,
            'right_zone': 0.75,

            # Pulse durations (for obstacle avoidance and search)
            'turn_pulse': 0.06,
            'obstacle_turn_pulse': 0.06,
            'search_turn_pulse': 0.2,

            # Frame settings
            'frame_width': 240,  # Match camera width
            'obstacle_distance': 100.0,  # Track from ANY distance

            # Speed settings
            'max_speed': 0.6,      # 60% power at far distance
            'min_speed': 0.4,      # 40% power when close to target
            'left_speed_factor': 1.0,   # Left motor at 100%
            'right_speed_factor': 1.0,  # Right motor at 100%

            # PID Controller settings for smooth steering
            'pid_kp': 0.15,  # Proportional: minimal reaction
            'pid_ki': 0.0,   # Integral: disabled
            'pid_kd': 0.8,   # Derivative: very strong damping

            # Dynamic speed settings
            'speed_distance_factor': 0.5  # How much distance affects speed
        }],
        output='screen'
    )

    # Foxglove Bridge - Optimized for real-time video streaming
    # Based on: https://github.com/foxglove/foxglove-sdk/blob/main/ros/src/foxglove_bridge/README.md
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',  # Listen on all interfaces
            'num_threads': 2,  # Use 2 threads for parallel processing

            # QoS settings - CRITICAL for low latency
            'min_qos_depth': 1,  # Minimum buffer - no frame accumulation
            'max_qos_depth': 1,  # Maximum buffer - always show latest frame only

            # Force BEST_EFFORT QoS for image topics - drops frames instead of buffering
            'best_effort_qos_topic_whitelist': [
                '/camera/.*',           # All camera topics
                '/detector/debug_image' # Debug visualization
            ],

            # Buffer settings
            'send_buffer_limit': 10000000,  # 10MB - enough for a few frames
            'use_compression': False,  # No WebSocket compression (CPU overhead)

            # Topic whitelist - INCLUDE video topics for visualization
            'topic_whitelist': [
                '/detector/debug_image',  # Video with YOLO annotations
                '/camera/image_raw',      # Raw camera feed
                '/camera/depth',          # Depth image
                '/target/.*',             # Position and status
                '/cmd_vel'                # Motor commands
            ],

            'capabilities': ['clientPublish', 'connectionGraph', 'assets'],
            'use_sim_time': False  # Real-time mode
        }],
        output='screen'
    )

    return LaunchDescription([
        camera_node,
        detector_node,
        motor_node,
        foxglove_bridge
    ])
