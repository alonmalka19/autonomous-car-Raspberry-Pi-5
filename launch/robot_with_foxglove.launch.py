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
            'confidence': 0.3,
            'inference_size': 192,
            'target_class': 'my_legs',
            'miss_limit': 3
        }],
        output='screen'
    )

    # Motor Node
    motor_node = Node(
        package='autonomous_car',
        executable='motor_node',
        name='motors',
        parameters=[{
            'left_backward_pin': 23,
            'left_forward_pin': 24,
            'right_backward_pin': 27,
            'right_forward_pin': 22,
            'stop_distance': 0.6,  # Stop when target is 60cm away
            'left_zone': 0.25,
            'right_zone': 0.75,
            'turn_pulse': 0.06,
            'obstacle_turn_pulse': 0.06,
            'search_turn_pulse': 0.2,
            'frame_width': 240,  # Match camera width
            'obstacle_distance': 100.0,  # Track from ANY distance (effectively disabled obstacle avoidance)
            'max_speed': 1.0,  # 100% power
            'left_speed_factor': 0.7,  # Left motor at 60%
            'right_speed_factor': 1.0  # Right motor at 100%
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
