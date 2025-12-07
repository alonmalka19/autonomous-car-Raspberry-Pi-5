# Autonomous Leg-Tracking Robot - Full Project Documentation

## Project Overview
This is a ROS2-based autonomous robot that tracks and follows human legs using computer vision. Built on Raspberry Pi 5 with real-time YOLO detection and motor control.

**Author**: Alon Malka
**Platform**: Raspberry Pi 5, Ubuntu 24.04, ROS2 Jazzy
**GitHub Repo Name**: `autonomous-leg-tracker`

---

## Hardware Components

| Component | Description |
|-----------|-------------|
| Raspberry Pi 5 | Main computer running ROS2 |
| L298N Motor Driver | Controls 4 DC motors via GPIO |
| 4x DC Motors | Tank-style drive (2 left, 2 right) |
| iPhone with Record3D | RGB + LiDAR depth camera via USB |
| Power Supply | Battery pack for motors |

### GPIO Pin Configuration
```
Left Motor:
  - Forward: GPIO 24
  - Backward: GPIO 23

Right Motor:
  - Forward: GPIO 22
  - Backward: GPIO 27
```

---

## Software Architecture

### ROS2 Nodes

#### 1. Camera Node (`camera_node.py`)
- **Purpose**: Captures RGB and depth frames from iPhone via Record3D USB streaming
- **Publishes**:
  - `/camera/image_raw` (sensor_msgs/Image) - RGB frames
  - `/camera/depth` (sensor_msgs/Image) - Depth frames from LiDAR
- **Parameters**:
  - `camera_type`: 'record3d' (default)
  - `max_frame_width`: 240 (resolution)
  - `publish_rate`: 15.0 (FPS)
  - `publish_depth`: True

#### 2. Detector Node (`detector_node.py`)
- **Purpose**: Detects legs using YOLOv5 custom model with optical flow fallback
- **Subscribes**:
  - `/camera/image_raw`
  - `/camera/depth`
- **Publishes**:
  - `/target/position` (geometry_msgs/Point) - x, y position + distance in z
  - `/target/tracking_status` (std_msgs/String) - "tracking", "lost:left", "lost:right"
  - `/detector/debug_image` (sensor_msgs/Image) - Annotated frame for visualization
- **Parameters**:
  - `model_path`: Path to YOLOv5 .pt model
  - `confidence`: 0.3 (detection threshold)
  - `inference_size`: 192 (YOLO input size)
  - `target_class`: 'my_legs'
  - `miss_limit`: 3 (frames before "lost")

#### 3. Motor Node (`motor_node.py`)
- **Purpose**: Controls motors based on target position, implements Tank Turn steering
- **Subscribes**:
  - `/target/position`
  - `/target/tracking_status`
  - `/camera/depth` (for obstacle detection)
- **Publishes**:
  - `/cmd_vel` (geometry_msgs/Twist) - For monitoring
- **Parameters**:
  - `stop_distance`: 0.6 (meters - stops when target is 60cm away)
  - `left_zone`: 0.35 (35% of frame = left zone)
  - `right_zone`: 0.65 (65% of frame = right zone)
  - `turn_pulse`: 0.06 (seconds - tracking turn duration)
  - `search_turn_pulse`: 0.4 (seconds - search turn duration)
  - `max_speed`: 1.0 (100% motor power)
  - `search_cooldown`: 1.0 (seconds between search commands)

#### 4. MJPEG Server (`mjpeg_server.py`)
- **Purpose**: HTTP streaming server for browser-based monitoring
- **Port**: 8080
- **Streams**: `/detector/debug_image` as MJPEG

#### 5. Foxglove Bridge (external package)
- **Purpose**: WebSocket bridge for Foxglove Studio visualization
- **Port**: 8765
- **QoS**: BEST_EFFORT with depth=1 for low latency

---

## Behavior Logic

### Target Tracking
```
IF target in LEFT zone (x < 35%):
    Turn LEFT (Tank Turn: left wheels backward, right wheels forward)

ELSE IF target in RIGHT zone (x > 65%):
    Turn RIGHT (Tank Turn: right wheels backward, left wheels forward)

ELSE IF target CENTERED (35% < x < 65%):
    IF distance > stop_distance (0.6m):
        Move FORWARD
    ELSE:
        STOP (too close)
```

### Target Lost (Search Mode)
```
IF target lost:
    Remember last seen direction (left/right)
    Turn toward last seen direction
    Wait search_cooldown (1.0s) between turns
    Turn duration: search_turn_pulse (0.4s)
```

### Tank Turn Implementation
- **Turn Left**: Left motor backward + Right motor forward
- **Turn Right**: Right motor backward + Left motor forward
- More powerful turning than single-side drive

---

## File Structure

```
autonomous_car/
├── autonomous_car/           # Python package
│   ├── __init__.py
│   ├── camera_node.py       # Record3D camera driver
│   ├── detector_node.py     # YOLO leg detection
│   ├── motor_node.py        # Motor control + Tank Turn
│   └── mjpeg_server.py      # HTTP video streaming
│
├── launch/
│   ├── robot.launch.py              # Basic launch
│   └── robot_with_foxglove.launch.py # Launch with Foxglove
│
├── models/
│   ├── best_mylegs_v5.pt    # Custom YOLOv5 leg model (22MB)
│   ├── best_mylegs_v5.onnx  # ONNX version (43MB) - not used
│   └── yolov8n.pt           # YOLOv8 nano (6MB) - backup
│
├── record3d/                # Record3D library (submodule)
│   └── ...
│
├── test/                    # ROS2 tests
├── config/                  # (empty)
├── resource/
│   └── autonomous_car       # Package marker
│
├── package.xml              # ROS2 package manifest
├── setup.py                 # Python setup
└── setup.cfg                # Setup configuration
```

---

## Current Parameters (Tuned & Working)

From `robot_with_foxglove.launch.py`:

```python
# Camera Node
'camera_type': 'record3d'
'max_frame_width': 240
'publish_rate': 15.0
'publish_depth': True

# Detector Node
'model_path': '/home/alonmalka/ros2_ws/src/autonomous_car/models/best_mylegs_v5.pt'
'confidence': 0.3
'inference_size': 192
'target_class': 'my_legs'
'miss_limit': 3

# Motor Node
'stop_distance': 0.6        # Stop at 60cm
'left_zone': 0.35           # Left = 0-35%
'right_zone': 0.65          # Right = 65-100%
'turn_pulse': 0.06          # 60ms tracking turn
'obstacle_turn_pulse': 0.06 # 60ms obstacle turn
'search_turn_pulse': 0.4    # 400ms search turn
'max_speed': 1.0            # 100% power
'frame_width': 240
'obstacle_distance': 100.0  # Disabled
```

In `motor_node.py`:
```python
'search_cooldown': 1.0      # 1 second between search turns
```

---

## Dependencies

### System
- Ubuntu 24.04 (arm64)
- ROS2 Jazzy
- Python 3.12

### Python Packages
- `ultralytics` (YOLO)
- `opencv-python`
- `numpy`
- `gpiozero` (GPIO control)
- `flask` (MJPEG server)

### ROS2 Packages
- `rclpy`
- `sensor_msgs`
- `geometry_msgs`
- `std_msgs`
- `cv_bridge`
- `foxglove_bridge`

### External
- Record3D iOS app (on iPhone)
- Record3D Python library (included in project)

---

## Usage

### Start the Robot
```bash
cd /home/alonmalka/ros2_ws
source install/setup.bash
ros2 launch autonomous_car robot_with_foxglove.launch.py
```

### Monitor via Foxglove
1. Open Foxglove Studio
2. Connect to: `ws://RASPBERRY_PI_IP:8765`
3. Add Image panel for `/detector/debug_image`

### Monitor via Browser
```
http://RASPBERRY_PI_IP:8080
```

---

## Key Optimizations Made

1. **Low Latency Video**:
   - QoS: BEST_EFFORT with depth=1
   - Non-blocking frame capture
   - Frame skipping when processing is slow
   - Record3D set to 15 FPS (not 60)

2. **Smooth Motor Control**:
   - Tank Turn for powerful rotation
   - Search cooldown prevents motor spam
   - Pulse-based turns (not continuous)

3. **Reliable Tracking**:
   - Optical flow fallback when YOLO misses
   - Last-seen direction memory for search
   - Configurable zones and thresholds

---

## Backup Location
```
/home/alonmalka/ros2_ws/src/autonomous_car_backup_20251206_working
```

---

## GitHub Upload Instructions

### Recommended Repo Name
`autonomous-leg-tracker`

### Files to Create
1. `.gitignore` - Exclude pycache, build artifacts
2. `.gitattributes` - Git LFS for model files (*.pt, *.onnx)
3. `README.md` - Professional documentation with badges
4. `LICENSE` - MIT license

### Large Files (Use Git LFS)
- `models/best_mylegs_v5.pt` (22MB)
- `models/best_mylegs_v5.onnx` (43MB)
- `models/yolov8n.pt` (6MB)

### Cleanup Before Upload
- Remove `record3d/.git/` directory
- Remove `**/__pycache__/` directories
- Remove `*.tmp.*` files

---

## Contact
For questions about this project, contact Alon Malka.
