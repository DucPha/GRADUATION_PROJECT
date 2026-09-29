# Autonomous Vehicle

Project map for the Mini PC (ROS 2) and ESP32-S3 autonomous vehicle.

## Main Components

- `src/PhamMinhDuc/`: main C++ ROS 2 package for camera lanes, LiDAR, obstacle avoidance, fusion, and ESP32 UART control.
- `src/traffic_light_detector/`: ROS 2 NCNN traffic-light/sign inference node.
- `src/turn_detector/`: ROS 2 NCNN turn-direction inference node.
- `firmware/esp32s3/autonomous_refactored_v5.0/`: active ESP32-S3 Arduino sketch. The sketch filename matches its folder name.
- `firmware/legacy/`: prior controller versions, firmware variants, and motor/servo/Wi-Fi tests. These are retained because they are not byte-identical alternatives.
- `tools/stream_laptop_cam/`: laptop camera streaming utility.
- `docs/`: project-level documentation.
## ROS 2 Build

From the project root on the Linux Mini PC with ROS 2 and `colcon` installed:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Run the main fusion node and inference nodes separately:

```bash
ros2 run autonomous_vehicle autonomous_vehicle_node
ros2 run traffic_light_detector traffic_light_detector
ros2 run turn_detector turn_detector
```

The inference nodes consume images from `/image_raw`; provide a camera publisher for that topic. They publish decisions on `/traffic_light/decision` and `/turn_detector/decision`. NCNN/Vulkan runtime libraries and model files are external prerequisites; model paths are currently configured inside each inference source file. The main node also requires a ROS 2 LiDAR driver publishing `sensor_msgs/msg/LaserScan` and a configured ESP32 serial device.

## Generated Files

Build outputs, Python bytecode caches, and PlatformIO dependency/build directories are ignored by `.gitignore`. Do not edit ROS 2 `build/`, `install/`, or `log/` output by hand.
