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
- **PID Controller** - Smooth proportional steering with dead-zone filtering to prevent zigzag motion
- **Motion Prediction** - Velocity-based position prediction to anticipate target movement
- **Hybrid Tracking** - YOLO detection + Optical Flow fallback for continuous tracking
- **Smart Search Mode** - Uses velocity history to predict target direction when lost
- **Low Latency** - Optimized for real-time performance (~15 FPS)
- **Web Monitoring** - Foxglove Studio for remote debugging and visualization

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
git clone https://github.com/alonmalka19/autonomous-car-Raspberry-Pi-5.git autonomous_car
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
│  (Record3D USB) │     │ YOLOv5 + Motion  │     │ PID Controller  │
│  RGB + Depth    │     │    Predictor     │     │  (L298N GPIO)   │
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
| `camera_node` | Captures RGB + depth from iPhone via Record3D USB |
| `detector_node` | YOLOv5 detection + motion prediction + optical flow fallback |
| `motor_node` | PID-based smooth steering with dead-zone filtering |

## Parameters

### Motor Node

| Parameter | Default | Description |
|-----------|---------|-------------|
| `stop_distance` | 0.6 | Stop when target is 60cm away |
| `far_distance` | 3.0 | Distance considered "far" for speed scaling |
| `max_speed` | 0.6 | Maximum motor power (60%) |
| `min_speed` | 0.4 | Minimum motor power when close |
| `pid_kp` | 0.15 | PID proportional gain |
| `pid_ki` | 0.0 | PID integral gain |
| `pid_kd` | 0.8 | PID derivative gain (damping) |

### Detector Node

| Parameter | Default | Description |
|-----------|---------|-------------|
| `confidence` | 0.2 | YOLO detection threshold |
| `inference_size` | 192 | YOLO input resolution |
| `miss_limit` | 3 | Frames before target is "lost" |
| `prediction_enabled` | true | Enable motion prediction |
| `prediction_lookahead` | 0.1 | Predict position 100ms ahead |
| `prediction_history_size` | 10 | Position samples for velocity calculation |

## Behavior Logic

### Target Tracking with PID
```
1. Get target position (or predicted position if moving)
2. Calculate error = target_x - frame_center
3. Apply dead zone (30%) - ignore small errors
4. PID calculates steering: Kp*error + Ki*integral + Kd*derivative
5. Differential drive: adjust left/right motor speeds
6. Speed scales with distance (slower when closer)
```

### Motion Prediction
```
1. Store position history with timestamps
2. Calculate smoothed velocity over history
3. Predict future position: pos + velocity * lookahead_time
4. Motor node steers toward predicted position
```

### Search Mode
```
IF target lost:
    Use velocity direction (if target was moving)
    Otherwise use last seen direction
    Turn toward expected location
```

## Project Structure

```
autonomous_car/
├── autonomous_car/           # Python package
│   ├── camera_node.py       # Record3D camera driver (RGB + depth)
│   ├── detector_node.py     # YOLO + motion prediction + optical flow
│   └── motor_node.py        # PID controller + differential drive
├── launch/
│   └── robot_with_foxglove.launch.py
├── models/
│   └── best_mylegs_v5.pt    # Custom YOLOv5 model
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
