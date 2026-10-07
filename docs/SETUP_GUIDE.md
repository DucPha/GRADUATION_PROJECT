# HƯỚNG DẪN CÀI ĐẶT VÀ CHẠY (Mini PC — ROS 2 Jazzy)

## 1. Yêu cầu

- Ubuntu 24.04 + ROS 2 Jazzy
- OpenCV 4.x, v4l-utils
- Camera MJPG 1920x1080, ESP32 thường (UART0 qua CP2102 → `/dev/ttyUSB0`), RPLIDAR A1 (`/dev/ttyUSB1`)

```bash
./tools/install_deps.sh     # apt + PySide6 + rosdep + nhóm dialout/video
```

Script cài `build-essential cmake v4l-utils libopencv-dev libjpeg-turbo8-dev python3-opencv
python3-numpy python3-matplotlib python3-serial python3-pyside2.* ros-jazzy-desktop
ros-jazzy-rplidar-ros ros-jazzy-cv-bridge` và `PySide6-Essentials` (pip, cho user).

## 2. Quyền phần cứng

```bash
sudo usermod -a -G video,dialout $USER
# rồi đăng xuất và đăng nhập lại
id -nG | tr ' ' '\n' | grep -xE 'video|dialout'
```

## 3. Build

Repo **chính là** workspace colcon: `src/` nằm ngay trong repo, nên clone về `~/autocar_ws`
rồi build luôn, không cần symlink.

```bash
git clone <repo> ~/autocar_ws
cd ~/autocar_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Chỉ build phần xe (bỏ 2 detector NCNN chưa dùng, cần `ncnn` + model riêng):

```bash
colcon build --symlink-install --packages-select \
  camera_node esp32s3_node sllidar_ros2_node fusion_node
```

## 4. Chạy

```bash
cd ~/autocar_ws
./run.sh                  # build, rồi chạy xe + dashboard
./run.sh --no-build       # chỉ chạy
./run.sh --no-gui         # không dashboard (xe đứng yên trừ khi require_start:=false)
./run.sh speed_x10:=30    # tham số launch truyền nguyên

# hoặc gọi launch trực tiếp
ros2 launch fusion_node fusion.launch.py
```

Xe **đứng yên** cho tới khi bấm **SPACE** (hoặc nút CHẠY XE) trên dashboard; bấm lại,
ESC, hoặc đóng dashboard ⇒ dừng. Cổng mặc định ESP32 `/dev/ttyUSB0`, LiDAR `/dev/ttyUSB1`.
Ghi đè khi cần:

```bash
ros2 launch fusion_node fusion.launch.py serial_port:=/dev/ttyUSB1 lidar_port:=/dev/ttyUSB0
```

## 5. Kiểm tra nhanh

```bash
ros2 topic echo /lane/status          # two_lanes, dev, emg, run, fps, lidar...
ros2 topic hz /lane/status            # 10 Hz
ros2 topic hz /scan                   # ~8 Hz nếu LiDAR đã tìm thấy
```

## 6. Camera

```bash
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --list-formats-ext      # MJPG 1920x1080 / 1280x720 / 640x480 @ 30
```

Node in kích thước thật của frame lúc mở và tự tính lại hình học theo đúng tỉ lệ khung.
Camera này ở chế độ phơi sáng tự động tự hạ còn ~25 fps khi phòng tối; muốn giữ 30 fps
thì phơi sáng tay: `camera_exposure:=250` (25 ms). Đổi độ phân giải:
`camera_width:=1280 camera_height:=720`.

## 7. Serial ESP32 (UART)

ESP32 nối với Mini PC qua **UART0 + chip CP2102** trên board ⇒ `/dev/ttyUSB*`, 230400 8N1.
**Không dùng USB-OTG** (không có `/dev/ttyACM*`).

```bash
ls -l /dev/serial/by-id/ /dev/ttyUSB*
dmesg | tail -20          # xác nhận cp210x đã gán ttyUSB0 / ttyUSB1
```

ESP32 và LiDAR đều là CP2102 cùng số serial `0001` ⇒ `/dev/serial/by-id` chỉ còn 1 link,
không phân biệt được. Mặc định ESP32 = `ttyUSB0`, LiDAR = `ttyUSB1` (thứ tự cắm). Nếu cổng
cấu hình không tồn tại, launch/node tự dò: mở từng cổng ở 230400 baud và nghe gói telemetry
`DC BA` mà ESP32 gửi 50 Hz (LiDAR im lặng nên không bị nhận nhầm). Đổi thứ tự cắm thì
truyền tay: `serial_port:=/dev/ttyUSB1 lidar_port:=/dev/ttyUSB0`.

Rút cáp ESP32 khi đang chạy: node báo `serial FAIL`, gửi EMG được ngay khi cắm lại (tự mở
lại cổng mỗi giây).

## 8. Hai detector NCNN (tu chọn)

`traffic_light_detector` và `turn_detector` cần `ncnn` + file model `.param`/`.bin`.
`ncnn` **không** phải rosdep key nên phải cài tay, và cần Vulkan driver:

```bash
pip install ncnn
vulkaninfo | head -5          # xác nhận GPU/Vulkan hoạt động
```

Cài rồi build được cả 7 package. Nếu chưa cài hoặc chưa có model thì dùng
`--packages-select` ở mục 3 để build phần xe. Phần cần thiết để chạy xe là
`camera_node`, `esp32s3_node`, `sllidar_ros2_node`, `fusion_node`.

## 9. Lỗi thường gặp

| Triệu chứng | Nguyên nhân | Xử lý |
|---|---|---|
| `Chua mo duoc camera, dang tu thu lai` | camera đang cắm lại / app khác giữ / chưa vào nhóm `video` | node tự thử lại mỗi 2 s; đóng app kia (cheese...) hoặc `usermod -aG video $USER` |
| LiDAR: áp tay sát mà không thấy điểm | vùng mù 15 cm của RPLIDAR A1 | đưa tay xa hơn 15 cm; vòng xám ở tâm bản đồ là vùng mù |
| `serial FAIL` | cáp USB chỉ nạp điện, hoặc board chưa nạp | thử cáp khác, nạp lại firmware |
| `two_lanes=0` luôn | chưa chỉnh ROI/ngưỡng | xem `docs/TUNING.md` |
| xe không chạy dù thấy làn | chưa bấm SPACE / dashboard đóng | bấm SPACE; hoặc `require_start:=false` |
| dashboard báo `MẤT KẾT NỐI FUSION NODE` | `fusion_node` chưa chạy hoặc khác `ROS_DOMAIN_ID` | chạy launch, cùng terminal đã `source` |
| `No module named PySide6` | chưa cài | `./tools/install_deps.sh` (hoặc dùng PySide2 có sẵn) |
| `age=-1ms` | chưa có frame nào | camera chưa mở được |
| LiDAR không có `/scan` | không tìm thấy cổng USB thứ hai | xem dòng log `[fusion] LiDAR port (auto)` |
| `ParameterTypeException` | không dùng launch mà truyền tham số tay | ép kiểu: `-p camera_fps:=30` (số, không để chuỗi) |

## 10. Kiểm tra firmware

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 --build-path /tmp/esp32build \
  firmware/esp32s3/autonomous_vehicle
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32 --input-dir /tmp/esp32build
```

Chip là **ESP32 thường** (board "ESP32 Dev Module"). Nạp và truyền dữ liệu dùng chung cổng
UART0/CP2102. Arduino IDE: **Core Debug Level = None**. Tắt `ros2 launch` trước khi nạp.