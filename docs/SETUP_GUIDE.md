# HƯỚNG DẪN CÀI ĐẶT VÀ CHẠY (NUC - ROS 2)

## 1. Yêu cầu
- Ubuntu 22.04/24.04 + ROS 2 (Jazzy/Humble)
- RAM 8GB (ưu tiên không chạy đồng thời nhiều node nặng)
- Qt5: `qtbase5-dev`
- OpenCV 4.x, jsoncpp, v4l-utils

```bash
sudo apt update && sudo apt install -y \
  build-essential cmake \
  ros-$ROS_DISTRO-rclcpp ros-$ROS_DISTRO-sensor-msgs ros-$ROS_DISTRO-std-msgs \
  ros-$ROS_DISTRO-cv-bridge \
  libopencv-dev libjsoncpp-dev qtbase5-dev v4l-utils
```

## 2. Build
```bash
cd ~/GRADUATION_PROJECT
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select autonomous_vehicle autonomous_vehicle_gui --symlink-install --cmake-args -DBUILD_TESTING=ON
```

## 3. Chạy trên Mini-PC (NUC)
```bash
source ~/GRADUATION_PROJECT/install/setup.bash
# Core
ros2 launch autonomous_vehicle minipc.launch.py
# GUI Qt5 (full debug + ảnh)
ros2 launch autonomous_vehicle_gui dashboard.launch.py
# Chỉ xem GUI (demo không cần robot)
ros2 launch autonomous_vehicle_gui dashboard.launch.py demo:=true
```

## 4. Tắt Debug để tiết kiệm CPU/RAM
`autonomous_vehicle_node` có `enable_dbg` (default true). Tắt khi không cần GUI:

```bash
ros2 run autonomous_vehicle autonomous_vehicle_node --ros-args -p enable_dbg:=false -p dbg_rate_hz:=5.0
```
Hoặc trong launch thêm param `enable_dbg`, `dbg_rate_hz`.

## 5. Camera
- `/dev/video0`. Kiểm tra: `v4l2-ctl -d /dev/video0 --list-ctrls`
- Tắt auto-exposure: `v4l2-ctl -d /dev/video0 --set-ctrl=exposure_auto=1 --set-ctrl=exposure_absolute=50 --set-ctrl=gain=64` (thử từng xe)

## 6. Serial ESP32
`serial_port:=/dev/ttyUSB0`. Cấp quyền: `sudo usermod -aG dialout $USER && reboot`

## 7. Test
```bash
colcon test --packages-select autonomous_vehicle
./build/autonomous_vehicle/test_lidar_module
```
