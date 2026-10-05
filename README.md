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

Pin lấy từ `firmware/esp32s3/autonomous_vehicle/autonomous_vehicle.ino`:
`PIN_STEER=32`, `PIN_ESC=33`, `PIN_TURN_L=25`, `PIN_TURN_R=26`, `PIN_BRAKE=27`, `PIN_HALL=4`.

## Cấu trúc

Repo **là** workspace colcon của ROS 2: `src/` nằm ngay trong repo nên clone về
`~/autocar_ws` là build được, không cần symlink.

```
GRADUATION_PROJECT/                  # = ~/autocar_ws
├── run.sh                           # build + chạy toàn bộ hệ thống
├── src/                             # package ROS 2 (colcon chỉ tìm ở đây)
│   ├── camera_node/                  # thư viện detector 2 làn (OpenCV, không rclcpp)
│   │   └── camera_node.hpp           #   toàn bộ hằng số detector
│   ├── esp32s3_node/                 # thư viện driver serial 11/7 byte
│   ├── sllidar_ros2_node/            # LidarModule + ObstacleAvoidance
│   ├── fusion_node/                  # node điều khiển duy nhất
│   │   ├── fusion_viz_node.cpp
│   │   └── launch/fusion.launch.py   #   launch có tham số
│   ├── gui_matplotlib/               # dashboard Python + matplotlib
│   ├── traffic_light_detector/       # Python, chưa nối vào phase 1
│   └── turn_detector/                # Python, chưa nối vào phase 1
├── firmware/esp32s3/                 # Arduino sketch (.ino)
│   ├── autonomous_vehicle/           # firmware xe đang dùng
│   └── motor_test/  servo_test/  esp32s3_wifi/
├── docs/                             # tài liệu
│   ├── SETUP_GUIDE.md  TUNING.md  ARCHITECTURE.md  FIX_REPORT.md
│   ├── camera_lane_phan_tich.docx    # phân tích detector làn
│   ├── lidar_module_phan_tich.docx   # phân tích LiDAR
│   ├── notes/                        # task_plan, progress, findings
│   └── legacy/                       # tài liệu v1.0.1 đã cũ
├── tools/docgen/                     # script sinh file .docx
└── .agent/                           # skills + template cho agent
```

Package C++ đặt `.cpp`/`.hpp` ngay gốc — mỗi package chỉ 1–4 file nên tách `src/`,
`include/` chỉ thêm tầng thư mục không mang thông tin. Chỉ giữ lại hai thư mục con có
ý nghĩa chức năng: `launch/` (nơi `ros2 launch` tìm file) và `resource/<tên_package>`
(marker ament index, thiếu là `ros2 run` không tìm thấy package). Package Python bắt buộc
phải có thư mục trùng tên vì `setup.py` khai báo `packages=[<tên>]`.

Header vẫn được `install(FILES ...)` vào `include/` trong không gian cài đặt, nên các
package khác include phẳng (`#include "camera_node.hpp"`) là đúng.

Phase 1 chỉ dùng **camera + serial**. LiDAR vẫn chạy và `/scan` vẫn có dữ liệu để quan
sát, nhưng **không** đưa vào quyết định lái: `ObstacleAvoidance` có cờ emergency stop
khi mất dữ liệu, chưa phù hợp khi chưa dùng tới.

## Cài đặt (Mini PC)

```bash
export ROS_DISTRO=jazzy

sudo apt update && sudo apt install -y \
  ros-${ROS_DISTRO}-desktop libopencv-dev python3-opencv python3-numpy \
  python3-matplotlib v4l-utils

# quyền phần cứng
sudo usermod -a -G video,dialout $USER     # rồi đăng xuất/đăng nhập lại

# workspace
git clone <repo> ~/autocar_ws
cd ~/autocar_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

`traffic_light_detector` và `turn_detector` cần `ncnn` + model riêng. Nếu chưa có, build
phần xe bằng:

```bash
colcon build --symlink-install --packages-select \
  camera_node esp32s3_node sllidar_ros2_node fusion_node gui_matplotlib
```

## Chạy

```bash
./run.sh                              # build nếu cần, rồi launch
./run.sh --no-build                   # chỉ launch
./run.sh --gui                        # kèm dashboard matplotlib
./run.sh speed_x10:=30 camera_index:=0 dev_sign:=-1

# hoặc gọi launch trực tiếp
ros2 launch fusion_node fusion.launch.py
```

`./run.sh` tự cấp quyền `/dev/tty*` `/dev/video*`, build lại, source môi trường rồi launch.
Nội dung phần cứng: cổng serial **tự nhận** — ESP32 ưu tiên `/dev/serial/by-id/*esp*` rồi
`/dev/ttyACM*`; LiDAR là `/dev/ttyUSB*` còn lại. Launch tự spawn driver:

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
| `lidar_mount_offset_deg` | `-90.0` | Góc lệch lắp LiDAR, `0` = trước, `90` = trái (REP-103) |
| `log_level` | `info` | `debug` để in nhiều hơn |

## Detector 2 làn (cửa sổ trượt)

Ảnh 640x480 → resize 320x240 → xám → **Gaussian blur(5,5)** → **CLAHE** → **mask ROI
hình thang** → Otsu `BINARY_INV` → `CLOSE(5,15)` + `OPEN(3,3)`.

Chỉ xét vùng `[0.58, 0.93]` chiều cao ảnh, chia **5 cửa sổ × 16 hàng**:

```
y=0..139     bỏ trên (trời)
y=139..223   5 cửa sổ, quét từ cửa sổ XA nhất xuống cửa sổ GẦN xe
              cửa sổ 4 (y 143..159) = xa nhất, 2 vạch hẹp nhất
              cửa sổ 0 (y 207..223) = gần xe nhất, 2 vạch RỘNG NHẤT
y=223..240   bỏ đáy (sát đầu xe: nhòe, 2 vạch dính nhau)
```

> `ROI_BOTTOM_FRAC = 0.93` chừa khoảng trống hai bên làn thay vì kéo tới sát đầu xe.
> Cửa sổ 0 là nơi lấy seed và nơi đo bề rộng làn — ở đó 2 vạch rộng nhất nên seed đáng
> tin nhất. Đổi lại phải nới `LANE_WIDTH_MAX`, xem `docs/TUNING.md`.

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

## Lịch sử phát triển

20 commit, 29/09/2026 → 04/10/2026. Chi tiết từng lỗi đã sửa: `docs/FIX_REPORT.md`
(34 mục A1–A20, B1–B14).

### 29/09 — Dựng workspace

| Commit | Nội dung |
|---|---|
| `2a5d116` | Cấu trúc thư mục gốc |
| `23d5aea` | Bật camera visualization, bridge `/image_raw` cho node AI, thêm launch file |
| `d03669c` | Hỗ trợ cả ROS 2 Jazzy lẫn Humble (đường dẫn header `cv_bridge`) |
| `e6ad491` | Sửa xung đột GPIO 19 ESP32 USB, exposure âm trên V4L2, mặc định an toàn |

### 30/09 – 02/10 — Tính năng chính

| Commit | Nội dung |
|---|---|
| `1c5361f`, `a7dcb44` | Cập nhật code ROS 2 |
| `de9acb7` | Lane detection, obstacle avoidance, bộ công cụ test |

### 03/10 — Sửa lỗi diện rộng và an toàn

| Commit | Nội dung |
|---|---|
| `43531f2`, `883f27b` | Qt5 dashboard, debug publishers, `roi_y0`; sửa phép fit 3x3; giải quyết conflict |
| `c647b1b` | `.gitignore`, tài liệu setup/arch/tuning; dọn cho NUC 8 GB |
| `f60f2b7` | Sửa build Linux/ROS 2, V4L2 + `feedback_age_ms`, CMake, GUI deps |
| `2c40c2a` | Dùng tham số `lidar_mount_offset_deg`, tự dò camera, chặn tốc độ ESP32, an toàn luồng GUI |
| `a827256` | Thiếu `rclcpp::init` trong GUI main |
| `19988f4` | Bỏ lặp tốc độ của detector cho state `NORMAL` |
| `59845cf` | `SPEED_SWERVE`/`SPEED_RETURN` chưa khai báo, lỗi kiểu `min()` |
| `ab9b951` | Sửa `lane_mask`, `fusion_viz_node`, `bringup.launch.py`, `.vscode`. **Message gốc là rác** (`ádasdasd`) |
| `104f0f5` | Viết lại firmware (329 dòng) + `bringup.launch.py` (251 dòng), thêm `run_all.sh`. **Message gốc là rác** (`ádasdasd`) |

Hai commit `ab9b951` và `104f0f5` để trống message nên không đọc được nội dung gì từ
`git log`. Nội dung thật lấy từ `git show --stat` như bảng trên.

Phần lớn commit trong ngày này thuộc đợt sửa 34 lỗi ghi ở `docs/FIX_REPORT.md`, trong đó
lỗi chí mạng nhất: vùng quét LiDAR lệch 90° khiến xe **không bao giờ** thấy vật cản
phía trước (A1), in chẩn đoán trên đúng UART mang protocol nhị phân làm mất gói (A10),
vùng trước/sau chồng nhau kẹt BYPASS vĩnh viễn (A2).

### 04/10 — Tái cấu trúc và hoàn thiện pipeline ảnh

| Commit | Nội dung |
|---|---|
| `67afea9` | Bỏ layout cũ (`src/`, `tools/`, Docker, 3 video 65 MB), tách `software/` |
| `0bbce1e` | 7 package ROS 2; pipeline 9 bước; đo cm bằng công thức pinhole thay IPM; sửa lỗi đo bề rộng ở sai cửa sổ |
| `3f3855a` | Viết lại README và docs cho kiến trúc mới |

Hai quyết định thiết kế đáng chú ý ở đợt này:

- **Bỏ IPM.** Đo bề rộng làn theo centimet bằng công thức pinhole chỉ cần 2 số
  (`CAMERA_HEIGHT_M`, `HORIZON_Y`) thay vì 4 điểm hiệu chỉnh + `warpPerspective`.
  Nhờ vậy `dev_px` vẫn là pixel ảnh gốc nên **firmware không phải sửa**, protocol 11 byte
  giữ nguyên.
- **Sửa lỗi đo bề rộng.** Code đo bề rộng ở cửa sổ xa nhất (2 vạch hẹp nhất) thay vì
  cửa sổ gần xe. Với làn 30 cm, ở cửa sổ xa chỉ còn 32 px < `LANE_WIDTH_MIN` nên bị lo.

### 06/10 — Chuyển `software/` thành `src/`, gộp dashboard

| Commit | Nội dung |
|---|---|
| (hiện tại) | Repo thành workspace colcon thật: `software/*` → `src/*`. Bỏ `gui_node` (Qt5), giữ `gui_matplotlib`. Ghi chú → `docs/notes/`, script sinh docx → `tools/docgen/`, skills → `.agent/`. Gộp `run_all.sh` thành `run.sh` build + chạy. Thêm `.gitattributes` ép LF cho `.sh` |

Bỏ `gui_node` vì dashboard Qt5 bị thay bằng bản matplotlib nhẹ hơn, build được trên mọi
máy không cần `qtbase5-dev`.

## Tài liệu

| File | Nội dung |
|---|---|
| `docs/SETUP_GUIDE.md` | cài đặt, chạy, xử lý sự cố |
| `docs/ARCHITECTURE.md` | luồng dữ liệu giữa các thành phần |
| `docs/TUNING.md` | chỉnh detector (hằng số trong `src/camera_node/camera_node.hpp`) |
| `docs/FIX_REPORT.md` | 34 lỗi đã sửa, theo bố cục code cũ |
| `docs/camera_lane_phan_tich.docx` | phân tích detector làn |
| `docs/lidar_module_phan_tich.docx` | phân tích LiDAR |
| `docs/notes/findings.md` | số liệu chứng minh cho quyết định thiết kế (vì sao ROI 0.45 sai, vì sao bỏ IPM, bề rộng làn theo từng hàng) |
| `docs/notes/task_plan.md`, `docs/notes/progress.md` | kế hoạch và tiến độ các đợt làm việc |

Sinh lại 2 file `.docx` trong `docs/` bằng `tools/docgen/` (cần `python3-docx`):

```bash
python3 -m pip install python-docx
python3 tools/docgen/build.py
```

## License

MIT