# Autonomous Vehicle — Graduation Project

Xe tự hành 1/10: Mini PC chạy ROS 2 Jazzy (Ubuntu 24.04) đọc camera, giữ xe giữa 2 làn
và gửi lệnh lái/tốc độ xuống ESP32-S3 qua **USB-OTG**.

## Phần cứng

| Vai trò | Thiết bị | Kết nối |
|---|---|---|
| Camera | USB camera **640x480** | `/dev/video*` (V4L2) |
| Controller | ESP32-S3 | **USB-OTG (CDC)** → `/dev/ttyACM*` |
| LiDAR | RPLIDAR A1 | USB-UART → `/dev/ttyUSB*`, baud 115200 |
| Động cơ | DC motor + ESC | GPIO 33 |
| Steering | Servo | GPIO 32 |

Pin lấy từ `firmware/esp32s3/autonomous_vehicle/autonomous_vehicle/autonomous_vehicle.ino`:
`PIN_STEER=32`, `PIN_ESC=33`, `PIN_TURN_L=25`, `PIN_TURN_R=26`, `PIN_BRAKE=27`, `PIN_HALL=4`.

## Cấu trúc

```
GRADUATION_PROJECT/
├── software/                      # workspace con đưa vào src/ của colcon
│   ├── camera_node/               # package camera_node  — detector 2 làn (OpenCV, không rclcpp)
│   ├── esp32s3_node/              # package esp32s3_node — driver serial (không rclcpp)
│   ├── sllidar_ros2_node/         # package sllidar_ros2_node — LidarModule + ObstacleAvoidance
│   ├── fusion_node/               # package fusion_node — node điều khiển duy nhất + launch
│   ├── gui_node/                  # package autonomous_vehicle_gui (Qt5, đang tự phát triển)
│   ├── traffic_light_detector/    # Python, chưa nối vào phase 1
│   ├── turn_detector/             # Python, chưa nối vào phase 1
│   └── launch/dashboard.launch.py # (tham chiếu, GUI tự làm sau)
├── firmware/esp32s3/
│   ├── autonomous_vehicle/autonomous_vehicle/  # firmware xe (đang dùng)
│   ├── motor_test/  servo_test/  esp32s3_wifi/  # sketch kiểm tra từng phần
├── docs/                          # SETUP_GUIDE, TUNING, ARCHITECTURE, FIX_REPORT
├── spec/, template/, skills/      # đặc tả và hướng dẫn làm việc
├── graduation_project-master/     # tài liệu tham khảo gốc
└── run_all.sh                     # chạy toàn bộ hệ thống
```

Phase 1 chỉ dùng **camera + serial**. LiDAR vẫn chạy và `/scan` vẫn có dữ liệu để quan
sát, nhưng **không** đưa vào quyết định lái: `ObstacleAvoidance` có cờ emergency stop
khi mất dữ liệu, chưa phù hợp khi chưa dùng tới.

## Cài đặt (Mini PC)

```bash
export ROS_DISTRO=jazzy

sudo apt update && sudo apt install -y \
  ros-${ROS_DISTRO}-desktop libopencv-dev python3-opencv python3-numpy v4l-utils

# quyền phần cứng
sudo usermod -a -G video,dialout $USER     # rồi đăng xuất/đăng nhập lại

# workspace
mkdir -p ~/autocar_ws/src
cd ~/autocar_ws
ln -s /path/to/GRADUATION_PROJECT/software/* src/
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Chạy

```bash
./run_all.sh                       # hoặc:
ros2 launch fusion_node fusion.launch.py

# ghi đè tham số
ros2 launch fusion_node fusion.launch.py speed_x10:=30 camera_index:=0 dev_sign:=-1
```

Cổng serial **tự nhận**: ESP32 ưu tiên `/dev/serial/by-id/*esp*` rồi `/dev/ttyACM*`;
LiDAR là `/dev/ttyUSB*` còn lại. Launch tự spawn driver:

```
ros2 run rplidar_ros rplidar_composition --ros-args \
  -p serial_port:=<tự dò> -p serial_baudrate:=115200 -p frame_id:=laser \
  -p inverted:=false -p angle_compensate:=true
```

Không tìm thấy cổng USB thứ hai thì bỏ qua driver LiDAR, xe vẫn chạy, chỉ không có `/scan`.

### Topic

| Topic | Kiểu | Tần suố | Nội dung |
|---|---|---|---|
| `/lane/status` | `std_msgs/String` | 5 Hz | `two_lanes=1 dev=-12 emg=0 age=12ms proc=2.1ms fps=29.4 lidar=ok front=142cm serial=open w=50cm devm=-4cm` |
| `/lane/vis` | `sensor_msgs/Image` (bgr8) | 10 Hz | ảnh camera + 2 làn (trái xanh lá, phải xanh dương), đường giữa vàng, khung cửa sổ |
| `/scan` | `sensor_msgs/LaserScan` | 10 Hz | do driver LiDAR phát, chỉ để quan sát |

### Tham số launch

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `serial_port` | `""` | Cổng ESP32. Rỗng = tự dò |
| `lidar_port` | tự dò | Cổng LiDAR |
| `camera_index` | `-1` | -1 = tự dò 0..3 |
| `camera_fps` | `30` | FPS yêu cầu |
| `roi_top_frac` | `0.58` | Bỏ qua phần trên ảnh (trời, chân trời) |
| `speed_x10` | `40` | Tốc độ khi đủ 2 làn (km/h × 10) |
| `speed_hold_x10` | `20` | Tốc độ khi mất 2 làn (km/h × 10) |
| `lane_lost_stop_ms` | `400` | Mất 2 làn quá lâu thì dừng |
| `dev_sign` | `1` | `-1` nếu servo lắp ngược |
| `control_hz` | `100` | Tần suố gửi lệnh xuống ESP32 |
| `viz_hz` | `10` | Tần suố phát `/lane/vis` |
| `enable_viz` | `true` | Tắt để nhẹ máy |
| `enable_lidar` | `true` | Chỉ quyết định có subscribe `/scan` |
| `log_level` | `info` | `debug` để in nhiều hơn |

## Detector 2 làn (cửa sổ trượt)

Ảnh 640x480 → resize 320x240 → xám → **Gaussian blur(5,5)** → **CLAHE** → **mask ROI
hình thang** → Otsu `BINARY_INV` → `CLOSE(5,15)` + `OPEN(3,3)`.

Chỉ xét vùng `[0.58, 0.97]` chiều cao ảnh, chia **5 cửa sổ × 18 hàng**:

```
y=0..139     bỏ trên (trời)
y=139..233   5 cửa sổ, quét từ cửa sổ XA nhất xuống cửa sổ GẦN xe
              cửa sổ 4 (y 143..161) = xa nhất, 2 vạch hẹp nhất
              cửa sổ 0 (y 215..233) = gần xe nhất, 2 vạch RỘNG NHẤT
y=233..240   bỏ đáy (sát đầu xe: nhòe, 2 vạch dính nhau)
```

> `ROI_BOTTOM_FRAC = 0.97` kéo vùng tới sát đầu xe. Cửa sổ 0 là nơi lấy seed và
> nơi đo bề rộng làn — ở đó 2 vạch rộng nhất nên seed đáng tin nhất. Đổi lại phải
> nới `LANE_WIDTH_MAX`, xem `docs/TUNING.md`.

- Cửa sổ 4 → 3 → 2 lần lượt thử tách 2 vạch: lấy **đoạn trái nhất** và **đoạn phải
  nhất** làm seed, loại đoạn rộng > 25 px (bóng đổ, vật cản).
- Mỗi cửa sổ phía trên chỉ dò trong `seed ± 30 px`, cập nhật seed riêng cho từng bên
  ⇒ 2 làn không bao giờ chạy chéo. Cửa sổ nào `trái ≥ phải` hoặc 2 vạch cách < 30 px thì loại.
- `two_lanes = true` khi **≥ 3/5 cửa sổ** khớp cả 2 phía và bề rộng làn ở **cửa sổ 0**
  ∈ [50, 230] px. Đo ở cửa sổ 0 vì đó là điểm rộng nhất; đo ở cửa sổ 4 sẽ lo nhầm
  làn thon (làn 30 cm chỉ còn 32 px < 50).
- **Fit đường bậc 2** qua các điểm mỗi bên (`x = a·t² + b·t + c`, toạ độ y chuẩn hoá
  [0,1], giải bằng `cv::solve` vì OpenCV 4.11 không có `cv::fitPoly`). `dev_px` lấy từ
  đường fit tại hàng nhìn trước; nếu fit hỏng thì lùi về trung vị 5 cửa sổ như cũ.
- `dev_px = (centre − 160) × frame_w/320`, lọc EMA 0.35.
  Đơn vị là **pixel ảnh gốc**, giữ đúng quy ước firmware đã dùng.
- **Không dùng IPM.** Bề rộng làn ra centimet bằng công thức pinhole
  `W = w_px · CAMERA_HEIGHT_M / (y − HORIZON_Y)` — chỉ cần 2 số, không cần chessboard,
  không cần biến đổi phối cảnh. `w` và `devm` chỉ để báo cáo, **firmware không đổi**.
- Không có 2 làn ⇒ `dev_px = 0` và `two_lanes = false`; node điều khiển tự giữ hướng lái
  cuối và hạ tốc độ trong `lane_lost_stop_ms`.

Không cần calibrate vị trí làn: 2 làn được tự tìm từ ảnh.

## Giao thức

**Mini PC → ESP32 (11 byte)**
```
AB CD | DEV_H DEV_L | SPEED_X10 | EMG | 00 00 00 00 | XOR(byte 2..9)
```
**ESP32 → Mini PC (7 byte)**
```
DC BA | float32 velocity_kmh | XOR(byte 2..5)
```

> ⚠️ **Dừng xe: phải bật EMG, không được gửi `speed = 0`.**
> Firmware kẹp PWM bằng `[ESC_MIN_FWD=95, ESC_MAX_FWD=180]`, nên `speed = 0` bị đẩy
> lên 95 ≈ 1.55 km/h — xe tự bò. `emg = 1` mới là tín hiệu dừng thật (firmware ghi
> ESC_NEUTRAL). Watchdog 500 ms phía ESP32 là lưới an toàn cuối.

## Vòng điều khiển

| Trạng thái | `dev` | `speed` | `emg` |
|---|---|---|---|
| camera OK + đủ 2 làn | `dev_px × dev_sign` | `speed_x10` | 0 |
| camera OK, mất 2 làn < `lane_lost_stop_ms` | giữ hướng lái cuối | `speed_hold_x10` | 0 |
| mất 2 làn ≥ `lane_lost_stop_ms` | 0 | 0 | **1** |
| camera mất frame (> 200 ms) | 0 | 0 | **1** |

Khi node tắt, nó gửi một lệnh EMG trước khi đóng serial.

## ESP32-S3 firmware

```bash
arduino-cli compile --fqbn esp32:esp32:esp32s3 \
  firmware/esp32s3/autonomous_vehicle/autonomous_vehicle
arduino-cli upload -p /dev/ttyACM0 --fqbn esp32:esp32:esp32s3 --input-dir /tmp/esp32build
```

> Monitor serial **không** in chữ khi xe đang chạy: cổng đó mang protocol nhị phân 7 byte,
> chèn ASCII vào sẽ làm Mini PC mất đồng bộ state machine. Muốn xem trạng thái thì dùng
> `/lane/status`.

## Thử không cần xe

ESP32-S3 dùng cáp USB chính là cáp nạp và truyền dữ liệu. Nạp firmware rồi cắm
USB là chạy, không cần cổng UART riêng.

```bash
arduino-cli upload -p /dev/ttyACM0 --fqbn esp32:esp32:esp32s3 \
  --input-dir /tmp/esp32build
```

## Checklist trước khi chạy xe thật

1. Tay trên vô-la, bánh kê gạt, có người cầm cút cắt nguồn.
2. `/lane/vis` phải hiện `2 LANES OK` và 2 đường làn nằm đúng vạch.
3. Xe đặt lệch phải ⇒ `dev` **âm** ⇒ servo **< 90** và bánh kéo về giữa. Sai chiều thì
   chạy lại với `dev_sign:=-1`.
4. Che camera ⇒ `emg=1` và ESC về neutral, node không crash.
5. Rút USB serial ⇒ log `serial FAIL`, node vẫn chạy.
6. Bỏ vạch khỏi ảnh ⇒ `two_lanes=0`, tốc độ về `speed_hold_x10` rồi dừng sau 400 ms.

## Tài liệu khác

- `docs/SETUP_GUIDE.md` — cài đặt và xử lý sự cố
- `docs/TUNING.md` — chỉnh detector
- `docs/ARCHITECTURE.md` — luồng dữ liệu giữa các thành phần
- `docs/FIX_REPORT.md` — lịch sử lỗi đã sửa (theo bố cục code cũ)

## License

MIT