# Autonomous Leg-Tracking Robot

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![Python](https://img.shields.io/badge/Python-3.12-green)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%205-red)](https://www.raspberrypi.com/)

A ROS2-based autonomous robot that tracks and follows human legs using computer vision. Built on Raspberry Pi 5 with real-time YOLOv5 detection and motor control.

<!-- Add your demo GIF/image here -->
<!-- ![Demo](docs/demo.gif) -->

## Features

- **Real-time Leg Detection** - Custom YOLOv5 model trained for leg tracking
- **Depth Sensing** - iPhone LiDAR via Record3D for accurate distance measurement
- **Tank-Turn Steering** - Powerful rotation using differential drive
- **Search Mode** - Automatically searches for lost targets
- **Low Latency** - Optimized for real-time performance (~15 FPS)
- **Web Monitoring** - Foxglove Studio and MJPEG streaming support

## Hardware Requirements

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

## Software Requirements

- Ubuntu 24.04 (arm64)
- ROS2 Jazzy
- Python 3.12
- Record3D iOS app

## Installation

### 1. Clone the Repository

```bash
cd ~/ros2_ws/src
git clone https://github.com/YOUR_USERNAME/autonomous-leg-tracker.git autonomous_car
```

### 2. Install Dependencies

```bash
# Python packages
pip install ultralytics opencv-python numpy gpiozero flask

# ROS2 packages
sudo apt install ros-jazzy-cv-bridge ros-jazzy-foxglove-bridge
```

### 3. Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select autonomous_car
source install/setup.bash
```

## Usage

### Start the Robot

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch autonomous_car robot_with_foxglove.launch.py
```

### Monitor via Foxglove Studio

1. Open [Foxglove Studio](https://foxglove.dev/)
2. Connect to: `ws://RASPBERRY_PI_IP:8765`
3. Add Image panel for `/detector/debug_image`

### Monitor via Browser

```
http://RASPBERRY_PI_IP:8080
```

## Architecture

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│   Camera Node   │────▶│  Detector Node   │────▶│   Motor Node    │
│  (Record3D USB) │     │  (YOLOv5 + Flow) │     │  (L298N GPIO)   │
└─────────────────┘     └──────────────────┘     └─────────────────┘
        │                        │                        │
        ▼                        ▼                        ▼
  /camera/image_raw       /target/position            /cmd_vel
  /camera/depth           /target/tracking_status
                          /detector/debug_image
```

### ROS2 Nodes

| Node | Purpose |
|------|---------|
| `camera_node` | Captures RGB + depth from iPhone via Record3D |
| `detector_node` | YOLOv5 leg detection with optical flow fallback |
| `motor_node` | Tank-turn motor control based on target position |
| `mjpeg_server` | HTTP streaming for browser monitoring |

## Parameters

### Motor Node

| Parameter | Default | Description |
|-----------|---------|-------------|
| `stop_distance` | 0.6 | Stop when target is 60cm away |
| `left_zone` | 0.35 | Left zone threshold (0-35% of frame) |
| `right_zone` | 0.65 | Right zone threshold (65-100% of frame) |
| `turn_pulse` | 0.06 | Tracking turn duration (seconds) |
| `search_turn_pulse` | 0.4 | Search turn duration (seconds) |
| `max_speed` | 1.0 | Motor power (0.0-1.0) |

### Detector Node

| Parameter | Default | Description |
|-----------|---------|-------------|
| `confidence` | 0.3 | YOLO detection threshold |
| `inference_size` | 192 | YOLO input resolution |
| `miss_limit` | 3 | Frames before target is "lost" |

## Behavior Logic

### Target Tracking
```
IF target in LEFT zone (x < 35%):
    Turn LEFT (Tank Turn)
ELSE IF target in RIGHT zone (x > 65%):
    Turn RIGHT (Tank Turn)
ELSE IF target CENTERED:
    IF distance > 0.6m: Move FORWARD
    ELSE: STOP (too close)
```

### Search Mode
```
IF target lost:
    Turn toward last seen direction
    Wait 1.0s cooldown between turns
```

## Project Structure

```
autonomous_car/
├── autonomous_car/           # Python package
│   ├── camera_node.py       # Record3D camera driver
│   ├── detector_node.py     # YOLO leg detection
│   ├── motor_node.py        # Motor control
│   └── mjpeg_server.py      # HTTP streaming
├── launch/
│   └── robot_with_foxglove.launch.py
├── models/
│   ├── best_mylegs_v5.pt    # Custom YOLOv5 model (Git LFS)
│   └── yolov8n.pt           # Backup model
├── record3d/                 # Record3D library
├── package.xml
└── setup.py
```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Author

**Alon Malka**

## Acknowledgments

- [Ultralytics](https://ultralytics.com/) for YOLOv5
- [Record3D](https://record3d.app/) for iPhone LiDAR streaming
- [Foxglove](https://foxglove.dev/) for visualization tools
