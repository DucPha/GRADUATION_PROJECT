# Autonomous Vehicle Graduation Project

Mini PC (x86/ARM64, Linux/ROS 2) + ESP32-S3 firmware for autonomous vehicle.

## Hardware
- **Mini PC**: ROS 2 Jazzy (Ubuntu 24.04). Xem `Dockerfile` cho phần dựng container.
- **ESP32-S3**: Arduino/PlatformIO, 230400 baud UART
- **Sensors**: USB Camera (640x400), RPLiDAR A1/A2
- **Actuators**: DC motor + ESC, Servo steering, Hall speed sensor
- **AI Acceleration**: NCNN + Vulkan (iGPU)

## Repository Structure
```
GRADUATION_PROJECT/
├── src/
│   ├── autonomous_vehicle/      # Main C++ ROS 2 package
│   │   ├── launch/bringup.launch.py  # launch file duy nhất khai báo node
│   │   └── test/                    # unit test (ament_add_gtest)
│   ├── traffic_light_detector/  # NCNN traffic light detector (Python)
│   └── turn_detector/           # NCNN turn direction detector (Python)
├── firmware/
│   └── esp32s3/Autonomous_Vehicle/  # ESP32-S3 firmware
├── Dockerfile                   # Container image (build arg ROS_DISTRO)
├── rosdep.yaml                  # rosdep keys
├── tools/                       # esp32_sim.py (ESP32 giả lập), stream_laptop_cam
├── docs/                        # Documentation (FIX_REPORT.md: danh sách lỗi đã sửa)
├── skills/                      # Coding/debugging guidelines
└── build/install/log/           # generated; không sửa
```

## Quick Start (Mini PC)

### 1. Install Dependencies
```bash
export ROS_DISTRO=jazzy   # hoặc humble nếu chạy Ubuntu 22.04

# System dependencies
sudo apt update && sudo apt install -y \
  ros-${ROS_DISTRO}-desktop ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-vision-opencv \
  python3-opencv python3-numpy \
  v4l-utils udev

# Python binding NCNN (KHÔNG cài opencv-python bằng pip: sẽ đè lên cv2 của ROS)
pip3 install --break-system-packages ncnn-vulkan

# ROS 2 workspace
mkdir -p ~/autocar_ws/src
cd ~/autocar_ws
ln -s /path/to/GRADUATION_PROJECT/src/* src/
cp /path/to/GRADUATION_PROJECT/rosdep.yaml src/rosdep.yaml
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DBUILD_TESTING=ON
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
# Bringup all nodes (bringup.launch.py là nguồn duy nhất)
ros2 launch autonomous_vehicle bringup.launch.py

# minipc.launch.py chỉ là wrapper của bringup.launch.py, giữ cho tương thích
ros2 launch autonomous_vehicle minipc.launch.py

# RViz2 riêng (nếu không dùng enable_rviz:=true)
ros2 run rviz2 rviz2 -d \
  $(ros2 pkg prefix autonomous_vehicle)/share/autonomous_vehicle/rviz/autonomous.rviz
```

## Test Without Hardware

Chạy được toàn bộ đường điều khiển mà không cần xe, bằng ESP32 giả lập nói đúng
protocol nhị phân của firmware.

```bash
# Cổng serial ảo (terminal 1)
python3 tools/esp32_sim.py

# Hệ thống ROS 2 (terminal 2)
source install/setup.bash
ros2 launch autonomous_vehicle bringup.launch.py \
  serial_port:=/tmp/ttyESP32sim \
  enable_traffic:=false enable_turn:=false
```

Log của sim in ra mỗi 0,5 s:
```
[sim] RUN  dev= -25px  target= 3.0km/h  cur= 3.0km/h  pwm= 99  frames=1802 bad=0
```
Đọc được: `dev` là góc lái node fusion gửi xuống, `target` là tốc độ lệnh, `cur` là
tốc độ mô phỏng bám theo, `bad` là số frame sai CRC — **phải luôn bằng 0**.

Trường hợp nên kiểm tra:
- `bad > 0` → lệch protocol giữa PC và firmware.
- `RUN` trong khi bạn chưa cấp camera/LiDAR → sai ở `sensors_ok`, phải là `STOP`.
- Node in `STALE` liên tục → watchdog đang chạy đúng (cảm biến chưa có).

Mô phỏng cả vòng lặp watchdog khi nhận serial bằng cách tắt sim giữa chừng:
node sẽ in `ESP32 link down, stopping vehicle` rồi thử mở lại mỗi 3 s.

## ESP32-S3 Firmware

### Kiểm tra trước khi nạp (không cần board)
```bash
arduino-cli compile --fqbn esp32:esp32:esp32s3 \
  firmware/esp32s3/Autonomous_Vehicle
```
Phải không có `error:`. Cảnh báo từ thư viện ESP32Servo và driver MCPWM là của
thư viện, không phải của sketch.

### Nạp firmware

**Arduino IDE**
1. Cài ESP32 board package
2. Chọn **ESP32S3 Dev Module**
3. Upload speed **921600**, monitor **230400**
4. Mở `firmware/esp32s3/Autonomous_Vehicle/Autonomous_Vehicle.ino` và nạp

**arduino-cli**
```bash
arduino-cli compile --fqbn esp32:esp32:esp32s3 \
  --output-dir /tmp/esp32build firmware/esp32s3/Autonomous_Vehicle
arduino-cli upload -p /dev/ttyACM0 --fqbn esp32:esp32:esp32s3 \
  --input-dir /tmp/esp32build
```

**PlatformIO**
```bash
cd firmware/esp32s3/Autonomous_Vehicle
pio run -t upload
```

> Monitor serial **không** hiện gì khi xe chạy. UART đó chỉ mang protocol nhị phân
> 7 byte, in chữ vào đó sẽ chèn ASCII vào giữa các frame và làm MiniPC mất đồng bộ.
> Xem trạng thái qua HUD `/fusion_viz/image`, hoặc dùng `pio device monitor` chỉ để
> xác nhận board đã nạp xong rồi đóng lại.

Sau khi nạp, tìm cổng serial thật:
```bash
ls /dev/ttyACM* /dev/ttyUSB*
ros2 launch autonomous_vehicle bringup.launch.py serial_port:=/dev/ttyACM0
```

### Pin Mapping
| Function | GPIO |
|----------|------|
| Steering Servo | 33 |
| Hall Sensor | 23 |
| Turn Left LED | 21 |
| Turn Right LED | 18 |
| Brake LED | 15 |
| ESC (Motor) | 32 |

> **Cảnh báo phần cứng (chưa kiểm chứng trên xe thật)**
> - GPIO 33 trên ESP32-S3 xung đột với chân PSRAM octal. Nếu module đang bật PSRAM
>   octal, servo sẽ không nhận xung. Phải đo lại trên thiết bị thực tế.
> - Các hằng số `WHEEL_DIA_M`, `PULSES_PER_REV`, `GEAR_RATIO` quyết định độ chính xác
>   của vận tốc đo. Đối chiếu với thông số thực của bánh/encoder trước khi dùng.

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
| `baudrate` | 230400 | UART baudrate (phải khớp firmware) |
| `camera_index` | 0 | V4L2 camera index |
| `camera_fps` | 30 | Camera FPS yêu cầu |
| `viz_hz` | 120.0 | Tần suố vẽ HUD |
| `image_topic` | /fusion_viz/image | Topic ảnh HUD có vẽ bản đồ |
| `raw_image_topic` | /image_raw | Topic ảnh thô cho node AI |
| `raw_image_hz` | 5.0 | Tần suố phát ảnh thô |
| `scan_topic` | /scan | LiDAR topic |
| `lidar_mount_offset_deg` | -90.0 | Góc lệch lắp LiDAR (0=trước, 90=trái theo REP-103) |
| `camera_stale_timeout_s` | 1.0 | Ngưỡng coi camera đứng hình → phanh |
| `lidar_stale_timeout_s` | 0.5 | Ngưỡng coi LiDAR mất dữ liệu → phanh |
| `ai_timeout_s` | 2.0 | Ngưỡng coi quyết định AI cũ → bỏ qua |
| `turn_blend_px` | 30 | Góc lái bù khi nhận biển rẽ (dương = phải) |
| `turn_speed_x10` | 40 | Trần tốc độ khi nhận biển rẽ |
| `traffic_light_decision_topic` | /traffic_light/decision | Node C++ **và** node AI cùng dùng |
| `turn_detector_decision_topic` | /turn_detector/decision | Node C++ **và** node AI cùng dùng |
| `enable_rviz` | false | Khởi chạy RViz2 kèm theo |

Mọi topic khai ở đây được truyền cho **cả hai chiều**: node C++ subscribe,
node Python publish. Không có chỗ nào tự viết tên topic trong code, nên không thể
đổi tên ở một bên mà bên kia vẫn giữ tên cũ.

Giá trị watchdog càng nhỏ càng phanh sớm, đổi lại xe phản ứng chậm hơn khi cảm biến chập chờn.

### Safety behaviour
Khi camera đứng hình, LiDAR mất dữ liệu, hoặc liên lạc UART với ESP32 đứt, node fusion gửi
`emergency_stop = true` và tốc độ 0. Trạng thái này hiện trên HUD (`ESP32: FAIL`) và được log
bằng `RCLCPP_WARN_THROTTLE`.

## Troubleshooting

### Camera not opening
```bash
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --list-formats-ext
# Ensure MJPEG 640x400@120fps supported

# Node in "Cannot open camera" nhưng ls /dev/video* thấy bình thường
# => thường là user chưa thuộc nhóm video (phải logout/login lại sau usermod)
id -nG | tr ' ' '\n' | grep -x video || echo "CHƯA vào nhóm video"
sudo usermod -a -G video $USER   # rồi đăng xuất và đăng nhập lại
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
# hoặc override trực tiếp:
ros2 launch autonomous_vehicle bringup.launch.py \
  traffic_model_param:=$HOME/mymodel/model.ncnn.param
```

### `import ncnn` fails
```bash
python3 -c "import ncnn"
# Nếu lỗi, cài binding:
pip3 install --break-system-packages ncnn-vulkan
# KHÔNG cài opencv-python bằng pip: bản đó đè lên cv2 của ROS, làm node AI chết
# với lỗi kiểu dữ liệu numpy.
```

### `ParameterTypeException` khi khởi động node C++
Tham số số phải được ép kiểu trong launch (`ParameterValue(..., value_type=int)`).
Launch truyền chuỗi; nếu không ép, `declare_parameter("camera_fps", 30)` sẽ ném exception
và node chết ngay.

## Tests
```bash
# Build sạch từ đầu
rm -rf build install log
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DBUILD_TESTING=ON

# Unit test
./build/autonomous_vehicle/test_lidar_module

# Kiểm tra launch file parse được và tham số ép kiểu đúng
ros2 launch autonomous_vehicle bringup.launch.py --show-args

# Firmware compile (không cần board)
arduino-cli compile --fqbn esp32:esp32:esp32s3 firmware/esp32s3/Autonomous_Vehicle

# Python node syntax
python3 -m py_compile \
  src/traffic_light_detector/traffic_light_detector/traffic_light_ncnn.py \
  src/turn_detector/turn_detector/turn_detector_ncnn.py \
  tools/esp32_sim.py
```

Unit test cho phép tính sector của LiDAR (đây là nơi từng có bug "góc 90° không phải
phía trước"), giữ để chặn tái phát.

## Checklist trước khi chạy xe thật

Bánh kê gạt, mặt đất bằng phẳng, có người cầm cút để cắt nguồn.

1. `arduino-cli compile` sạch → nạp firmware → xác nhận cổng serial xuất hiện.
2. `ros2 launch ... serial_port:=<cổng thật>` → HUD phải hiện `ESP32: OK`, không phải `FAIL`.
3. LiDAR: `ros2 topic hz /scan` phải > 0, không phải `LiDAR stale` lặp lại.
4. Camera: HUD vẽ được vạch làn, `camera_cmd` đổi FWD/LEFT/RIGHT theo lái xe.
5. Đặt vật cản ở khoảng cách an toàn, xác nhận `bypass_state` chuyển
   NORMAL → SLOW_DOWN → DETECT → SWERVE → BYPASS → RETURN → NORMAL rồi mới tăng tốc.
6. Cắm rút ESP32 giữa chừng → node phải in `ESP32 link down` và xe dừng, không chạy tiếp.

Chưa kiểm chứng được bằng mô phỏng: độ chính xác sector LiDAR khi đặt góc lắp thật,
độ cong đường camera, và độ trễ lái cơ. Ba thứ đó phải đo trên xe.

## Performance Targets

## License
MIT (TODO: update license in package.xml files)