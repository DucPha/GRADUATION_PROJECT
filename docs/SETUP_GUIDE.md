# HƯỚNG DẪN CÀI ĐẶT VÀ CHẠY (Mini PC — ROS 2 Jazzy)

## 1. Yêu cầu

- Ubuntu 24.04 + ROS 2 Jazzy
- OpenCV 4.x, v4l-utils
- Camera 640x480, ESP32-S3 (USB-OTG), RPLIDAR A1

```bash
sudo apt update && sudo apt install -y \
  build-essential cmake v4l-utils libopencv-dev \
  ros-$ROS_DISTRO-desktop \
  ros-$ROS_DISTRO-rplidar-ros
```

## 2. Quyền phần cứng

```bash
sudo usermod -a -G video,dialout $USER
# rồi đăng xuất và đăng nhập lại
id -nG | tr ' ' '\n' | grep -xE 'video|dialout'
```

## 3. Build

```bash
mkdir -p ~/autocar_ws/src
cd ~/autocar_ws
ln -s /path/to/GRADUATION_PROJECT/software/* src/
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

`gui_node` cần Qt5 (`sudo apt install qtbase5-dev`). Nếu chưa cài mà chưa dùng GUI thì
bỏ qua nó:

```bash
colcon build --symlink-install --packages-select \
  camera_node esp32s3_node sllidar_ros2_node fusion_node
```

## 4. Chạy

```bash
cd ~/GRADUATION_PROJECT
./run_all.sh
# hoặc
ros2 launch fusion_node fusion.launch.py
```

Cổng serial tự nhận. Ghi đè khi cần:

```bash
ros2 launch fusion_node fusion.launch.py serial_port:=/dev/ttyACM0 camera_index:=0
```

## 5. Kiểm tra nhanh

```bash
ros2 topic echo /lane/status          # two_lanes, dev, emg, fps
ros2 topic hz /lane/vis               # ~10 Hz
ros2 topic hz /scan                   # ~10 Hz nếu LiDAR đã tìm thấy
```

## 6. Camera

```bash
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --list-formats-ext      # xác nhận có 640x480
v4l2-ctl -d /dev/video0 --set-ctrl=exposure_auto=1 --set-ctrl=exposure_absolute=50
```

Node in kích thước thật của frame lúc mở. Nếu khác 640x480 vẫn chạy được (hệ số quy
đổi tính lúc chạy) nhưng nên chỉnh lại cho đúng tỉ lệ khung.

## 7. Serial ESP32

ESP32-S3 dùng USB-OTG nên ra `/dev/ttyACM*` (không phải `/dev/ttyUSB*`). Không cần chỉnh
baud tay: CDC bỏ qua baud, code vẫn đặt 230400 cho khớp firmware.

```bash
ls -l /dev/serial/by-id/ /dev/ttyACM* /dev/ttyUSB*
dmesg | tail -20          # xác nhận board đã enumerate
```

Node tự dò: by-id chứa `esp` → `ttyACM*` → `ttyUSB*`. Có thể ép cổng bằng `serial_port:=`.

## 8. Lỗi thường gặp

| Triệu chứng | Nguyên nhân | Xử lý |
|---|---|---|
| `Camera failed to start` | chưa vào nhóm `video` | `usermod -aG video $USER`, đăng nhập lại |
| `serial FAIL` | cáp USB chỉ nạp điện, hoặc board chưa nạp | thử cáp khác, nạp lại firmware |
| `two_lanes=0` luôn | chưa chỉnh ROI/ngưỡng | xem `docs/TUNING.md` |
| `age=-1ms` | chưa có frame nào | camera chưa mở được |
| LiDAR không có `/scan` | không tìm thấy cổng USB thứ hai | xem dòng log `[fusion] LiDAR port (auto)` |
| `ParameterTypeException` | không dùng launch mà truyền tham số tay | ép kiểu: `-p camera_fps:=30` (số, không để chuỗi) |

## 9. Kiểm tra firmware

```bash
arduino-cli compile --fqbn esp32:esp32:esp32s3 \
  firmware/esp32s3/autonomous_vehicle/autonomous_vehicle
arduino-cli upload -p /dev/ttyACM0 --fqbn esp32:esp32:esp32s3 --input-dir /tmp/esp32build
```

ESP32-S3 dùng chính cổng USB để nạp và để truyền dữ liệu, không cần cổng UART riêng.