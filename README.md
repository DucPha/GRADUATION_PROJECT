# Autonomous Vehicle Graduation Project

Mini PC (x86/ARM64, Linux/ROS 2) + ESP32-S3 firmware for autonomous vehicle.

## Hardware
- **Mini PC**: ROS 2 Humble/Iron, Ubuntu 22.04/24.04
- **ESP32-S3**: Arduino/PlatformIO, 230400 baud UART
- **Sensors**: USB Camera (640x400@120fps), RPLiDAR A1/A2
- **Actuators**: DC motor + ESC, Servo steering, Hall speed sensor
- **AI Acceleration**: NCNN + Vulkan (iGPU)

## Repository Structure
```
GRADUATION_PROJECT/
├── src/
│   ├── autonomous_vehicle/      # Main C++ ROS 2 package
│   ├── traffic_light_detector/  # NCNN traffic light detector (Python)
│   └── turn_detector/           # NCNN turn direction detector (Python)
├── firmware/
│   └── esp32s3/Autonomous_Vehicle/  # ESP32-S3 firmware
├── tools/                       # Camera stream utilities
├── docs/                        # Documentation
├── skills/                      # Coding/debugging guidelines
└── launch/                      # System bringup launch files
```

## Quick Start (Mini PC)

### 1. Install Dependencies
```bash
# System dependencies
sudo apt update && sudo apt install -y \
  ros-humble-desktop ros-humble-cv-bridge ros-humble-vision-opencv \
  python3-opencv python3-numpy python3-ncnn-vulkan \
  v4l-utils udev

# ROS 2 workspace
mkdir -p ~/autocar_ws/src
cd ~/autocar_ws
ln -s /path/to/GRADUATION_PROJECT/src/* src/
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### 2. Hardware Permissions
```bash
# Camera
sudo usermod -a -G video $USER

# Serial (ESP32)
sudo usermod -a -G dialout $USER
sudo udevadm control --reload-rules && sudo udevadm trigger

# LiDAR
echo 'KERNEL=="ttyUSB*", MODE="0666"' | sudo tee /etc/udev/rules.d/99-lidar.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 3. Model Files
Place NCNN model files:
```
~/yolo_ws/runs/detect/traffic_all_red_turn_e10/weights/best_ncnn_model/
  ├── model.ncnn.param
  └── model.ncnn.bin

~/models/turn_lr_ncnn_model/
  ├── model.ncnn.param
  └── model.ncnn.bin
```

### 4. Run
```bash
# Terminal 1: Bringup all nodes
ros2 launch autonomous_vehicle bringup.launch.py

# Terminal 2: Visualization (optional)
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix autonomous_vehicle)/share/autonomous_vehicle/rviz/autonomous.rviz
```

## ESP32-S3 Firmware

### PlatformIO (Recommended)
```bash
cd firmware/esp32s3/Autonomous_Vehicle
pio run -t upload
pio device monitor -b 230400
```

### Arduino IDE
1. Install ESP32 board package
2. Select **ESP32S3 Dev Module**
3. Set upload speed: **921600**, monitor: **230400**
4. Open `Autonomous_Vehicle.ino` and upload

### Pin Mapping
| Function | GPIO |
|----------|------|
| Steering Servo | 33 |
| Hall Sensor | 23 |
| Turn Left LED | 21 |
| Turn Right LED | 18 |
| Brake LED | 15 |
| ESC (Motor) | 32 |

## Communication Protocol

### Mini PC → ESP32 (11 bytes)
```
AB CD | DEV_H DEV_L | SPEED_X10 | EMG | 00 00 00 00 | XOR
```
- `dev_final_px`: int16_t, lane deviation (px)
- `speed_control`: uint8_t, target speed × 10 (km/h)
- `emergency_stop`: bool

### ESP32 → Mini PC (7 bytes)
```
DC BA | FLOAT0 FLOAT1 FLOAT2 FLOAT3 | XOR
```
- `velocity_kmh`: float, current speed

## Key Parameters (Launch)
| Parameter | Default | Description |
|-----------|---------|-------------|
| `serial_port` | /dev/ttyUSB0 | ESP32 serial port |
| `camera_index` | 0 | V4L2 camera index |
| `camera_fps` | 120 | Camera FPS |
| `viz_hz` | 120 | Visualization rate |
| `scan_topic` | /scan | LiDAR topic |
| `traffic_model_param` | ~/yolo_ws/.../model.ncnn.param | Traffic model |
| `turn_model_param` | ~/models/.../model.ncnn.param | Turn model |

## Troubleshooting

### Camera not opening
```bash
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --list-formats-ext
# Ensure MJPEG 640x400@120fps supported
```

### Serial permission denied
```bash
sudo chmod 666 /dev/ttyUSB0
# Or reboot after adding user to dialout group
```

### LiDAR no data
```bash
ros2 topic echo /scan --once
# Check baudrate 115200/256000, frame_id=laser
```

### Model not loading
```bash
# Verify model paths in launch file match actual files
ls ~/yolo_ws/runs/detect/traffic_all_red_turn_e10/weights/best_ncnn_model/
ls ~/models/turn_lr_ncnn_model/
```

## Performance Targets
- End-to-end latency: < 50ms (camera → actuation)
- Camera pipeline: 120 FPS (capture + process)
- LiDAR processing: 10 Hz
- UART latency: < 10ms (with optimizations)
- CPU usage: < 70% on 4-core Mini PC

## License
MIT (TODO: update license in package.xml files)