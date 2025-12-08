from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory for portable paths
    pkg_dir = get_package_share_directory('autonomous_car')
    model_path = os.path.join(pkg_dir, 'models', 'best_mylegs_v5.pt')
    return LaunchDescription([
        # Camera Node
        Node(
            package='autonomous_car',
            executable='camera_node',
            name='camera',
            parameters=[{
                'camera_type': 'record3d',
                'max_frame_width': 640,
                'publish_rate': 30.0,
                'publish_depth': True
            }],
            output='screen'
        ),

        # Detector Node
        Node(
            package='autonomous_car',
            executable='detector_node',
            name='detector',
            parameters=[{
                'model_path': model_path,
                'confidence': 0.3,
                'inference_size': 320,
                'target_class': 'my_legs',
                'miss_limit': 3
            }],
            output='screen'
        ),

        # Motor Node
        Node(
            package='autonomous_car',
            executable='motor_node',
            name='motors',
            parameters=[{
                'left_backward_pin': 23,
                'left_forward_pin': 24,
                'right_backward_pin': 27,
                'right_forward_pin': 22,
                'stop_distance': 0.25,
                'left_zone': 0.25,
                'right_zone': 0.75,
                'turn_pulse': 0.05,
                'obstacle_turn_pulse': 0.10,  # Longer turns for obstacle avoidance
                'search_turn_pulse': 0.15,  # Even longer for searching when target lost
                'frame_width': 640,
                'obstacle_distance': 0.5,
                'max_speed': 0.65  # 65% power limit for 3-battery setup (12V protection)
            }],
            output='screen'
        ),
    ])
