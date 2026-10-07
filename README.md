# Autonomous Vehicle — Graduation Project

Xe tự hành 1/10: Mini PC chạy ROS 2 Jazzy (Ubuntu 24.04) đọc camera, giữ xe giữa 2 làn
và gửi lệnh lái/tốc độ xuống ESP32 qua **UART (serial)**. Dashboard PySide6 hiển thị
realtime và có nút **SPACE** để cho xe chạy/dừng.

## Phần cứng

| Vai trò | Thiết bị | Kết nối |
|---|---|---|
| Camera | USB camera **MJPG 1920x1080 @ 30 fps** | `/dev/video*` (V4L2) |
| Controller | **ESP32 thường** (DevKit, chip USB-UART **CP2102**) | **UART0** → `/dev/ttyUSB0`, 230400 baud |
| LiDAR | RPLIDAR A1 (CP2102) | UART → `/dev/ttyUSB1`, 115200 baud |
| Động cơ | DC motor + ESC | GPIO 33 |
| Steering | Servo | GPIO 32 |

> **Không dùng USB-OTG.** ESP32 nói chuyện với Mini PC qua UART0 + CP2102 nên Linux thấy
> `/dev/ttyUSB*`, không phải `/dev/ttyACM*`. ESP32 và LiDAR đều là CP2102 **cùng số serial
> `0001`**, nên `/dev/serial/by-id` không phân biệt được. Launch mặc định
> ESP32 = `ttyUSB0`, LiDAR = `ttyUSB1`; nếu cổng mặc định không tồn tại thì tự dò: cổng nào
> gửi gói telemetry `DC BA` của ESP32 là ESP32, cổng còn lại là LiDAR.

Pin lấy từ `firmware/esp32s3/autonomous_vehicle/autonomous_vehicle.ino`:
`PIN_STEER=32`, `PIN_ESC=33`, `PIN_TURN_L=25`, `PIN_TURN_R=26`, `PIN_BRAKE=27`, `PIN_HALL=4`.

## Cấu trúc

Repo **là** workspace colcon của ROS 2: clone về `~/autocar_ws` là build được.

```
GRADUATION_PROJECT/                  # = ~/autocar_ws
├── run.sh                           # build + chạy xe + dashboard
├── tools/install_deps.sh            # cài toàn bộ thư viện Ubuntu/ROS còn thiếu
├── src/
│   ├── camera_node/                  # thư viện detector 2 làn (OpenCV + libjpeg, không rclcpp)
│   │   └── camera_node.hpp           #   toàn bộ hằng số detector
│   ├── esp32s3_node/                 # driver UART 11/7 byte, tự dò + tự kết nối lại
│   ├── sllidar_ros2_node/            # LidarModule + ObstacleAvoidance
│   ├── fusion_node/                  # node điều khiển duy nhất
│   │   ├── fusion_viz_node.cpp
│   │   └── launch/fusion.launch.py   #   launch có tham số
│   ├── gui/gui.py                    # dashboard PySide6 (chạy trực tiếp)
│   ├── traffic_light_detector/       # Python, chưa nối vào phase 1
│   └── turn_detector/                # Python, chưa nối vào phase 1
├── firmware/esp32s3/                 # Arduino sketch (.ino)
│   ├── autonomous_vehicle/           # firmware xe đang dùng
│   └── motor_test/  servo_test/  esp32s3_wifi/
├── docs/                             # tài liệu
└── tools/docgen/                     # script sinh file .docx
```

Phase 1 chỉ dùng **camera + serial** để lái. LiDAR vẫn chạy, `/scan` hiện trên bản đồ
dashboard và khoảng cách trước/trái/phải/sau có trong `/lane/status`, nhưng **không**
đưa vào quyết định lái.

## Cài đặt (Mini PC)

```bash
git clone <repo> ~/autocar_ws
cd ~/autocar_ws
./tools/install_deps.sh        # apt + PySide6 + rosdep + nhóm dialout/video
# đăng xuất / đăng nhập lại nếu script vừa thêm nhóm
./run.sh
```

`install_deps.sh` cài: `build-essential cmake v4l-utils libopencv-dev libjpeg-turbo8-dev
python3-opencv python3-numpy python3-matplotlib python3-serial python3-colcon-common-extensions
python3-rosdep python3-pyside2.* ros-jazzy-desktop ros-jazzy-rplidar-ros ros-jazzy-cv-bridge`
và `PySide6-Essentials` (pip, chỉ cho user). Không có PySide6 thì dashboard tự dùng PySide2.

`traffic_light_detector` và `turn_detector` cần `ncnn` + model riêng, chưa dùng ở phase 1.
Chỉ build phần xe:

```bash
colcon build --symlink-install --packages-select \
  camera_node esp32s3_node sllidar_ros2_node fusion_node
```

## Chạy

```bash
./run.sh                              # build, chạy xe + dashboard
./run.sh --no-build                   # chỉ chạy
./run.sh --no-gui require_start:=false   # không dashboard, xe tự chạy khi thấy làn
./run.sh speed_x10:=30 dev_sign:=-1   # tham số launch truyền nguyên

# hoặc tách riêng
ros2 launch fusion_node fusion.launch.py
python3 src/gui/gui.py
```

### Chạy / dừng xe (SPACE)

Khi khởi động, xe **luôn đứng yên** (gửi EMG) cho tới khi người dùng cho chạy:

| Thao tác | Kết quả |
|---|---|
| Bấm **SPACE** hoặc nút **CHẠY XE** | xe chạy (tốc độ tăng dần theo ramp) |
| Bấm **SPACE** lần nữa / nút **DỪNG XE** | xe dừng ngay (EMG) |
| Bấm **ESC** | dừng ngay |
| Đóng dashboard, dashboard treo | mất heartbeat > `start_timeout_ms` (600 ms) ⇒ xe dừng |
| `fusion_node` khởi động lại | dashboard tự bỏ lệnh chạy, phải bấm SPACE lại |

**Điều kiện để bánh xe quay** — phải đủ CẢ 4:

1. Đã bấm SPACE và dashboard còn gửi heartbeat (`run=1` trong `/lane/status`).
2. Camera có frame mới (< 200 ms, `age=`).
3. Detector thấy ít nhất 1 vạch (`track=two` hoặc `one`), hoặc mất vạch chưa quá
   `lane_lost_stop_ms` (400 ms).
4. Cổng ESP32 mở (`serial=open`) và ESP32 nhận gói đều (watchdog 500 ms).

Khi đủ, lệnh tốc độ = `speed_x10 × speed_scale` (giảm khi vào cua / ít cửa sổ thấy vạch),
**không thấp hơn `speed_min_x10`** (mặc định 2.5 km/h). Lý do: ESC kẹp mọi tốc độ < 1.55
km/h về PWM 95 — mức thấp nhất; xe đặt dưới đất ở mức đó thường không đủ lực thắng ma sát
nên bánh đứng yên dù Mini PC đã gửi lệnh chạy. Đặt xe xuống mà bánh vẫn không quay thì
tăng `speed_min_x10:=30` (hoặc 35).

Dashboard gửi `std_msgs/Bool` lên `/autocar/run` 10 lần/giây. Không có dashboard vẫn cho
chạy được bằng tay:

```bash
ros2 topic pub -r 10 /autocar/run std_msgs/msg/Bool "{data: true}"
```

### Dashboard

```
┌ AUTOCAR MONITOR ─────────────── ● FUSION ● CAMERA ● LIDAR ● ESP32 ── giờ · uptime ┐
│ LANE DETECTION (overlay do C++ vẽ, 16:9)        │ VEHICLE CONTROL                 │
│                                                 │  STOPPED / FOLLOW LANE ...      │
│                                                 │  [ ▶ CHẠY XE   [SPACE] ]        │
│                                                 │  9 ô số liệu                    │
├────────────────────────┬────────────────────────┼─────────────────────────────────┤
│ ROI                    │ BINARY MASK            │ LiDAR MAP (polar, trước ↑)      │
└────────────────────────┴────────────────────────┴─────────────────────────────────┘
```

- Kích thước mặc định 1280x760 (tối thiểu 1024x640), `--width/--height` để đổi.
- **Mọi số liệu xử lý ảnh do C++ tính** (`fusion_node`): ảnh overlay, ROI, mask và mọi
  con số đều lấy từ topic. GUI không tự xử lý ảnh nên thứ thấy trên màn hình là đúng
  thứ bộ điều khiển đang dùng.
- Ảnh chỉ vẽ lại khi có khung mới, bản đồ LiDAR dùng blit (chỉ vẽ lại các điểm) ⇒ GUI
  chạy theo đúng fps camera mà gần như không tốn CPU.
- `fusion_node` chỉ nén/gửi ảnh khi có người xem, và việc nén chạy ở luồng riêng nên
  không làm trễ vòng điều khiển 100 Hz.
- Lăn chuột trên bản đồ LiDAR để đổi tầm nhìn 0.5–12 m (mặc định 3 m).
- Camera rút ra / bị app khác chiếm: khung ảnh báo `CAMERA CHƯA KẾT NỐI`, node tự thử
  mở lại mỗi 2 s — cắm lại là có hình, không cần chạy lại.

### Bản đồ LiDAR — quy luật dữ liệu

Giống hệt `LidarModule` (C++):

1. Tia `i` có góc ROS `a = angle_min + i·angle_increment` (ngược chiều kim đồng hồ,
   0 = trục +X của LiDAR). RPLIDAR A1 + `angle_compensate` cho 720 tia/vòng (0.5°), ~8 Hz.
2. Chỉ giữ tia có vật thật: số hữu hạn, > 0 và trong `[range_min, range_max]`
   (`inf` = không phản hồi, **không** phải vật cản).
3. Đổi sang **khung xe**: `góc_xe = a + lidar_mount_offset_deg` ⇒
   0° = TRƯỚC, 90° = TRÁI, 180° = SAU, 270° = PHẢI (REP-103).
4. Vẽ 0° ở phía **trên**, tăng ngược chiều kim đồng hồ ⇒ trái là trái, phải là phải.
5. Màu theo đúng ngưỡng cảnh báo C++: **đỏ < 0.4 m** (DANGER), **cam < 0.6 m** (WARNING),
   vàng < 1.5 m, xanh xa hơn — tông đậm, điểm có viền mảnh để nổi rõ trên nền trắng. Tầm nhìn mặc định 3 m: ở thang 12 m, vật cách 20–50 cm chỉ
   cách tâm vài pixel và bị biểu tượng xe che, trông như "không hiện".

> **Vùng mù 15 cm** (vòng xám ở tâm): RPLIDAR A1 không đo được vật gần hơn 15 cm — driver
> không trả về tia nào (đã kiểm 20 vòng quét: 0 giá trị < 0.15 m). Áp tay sát LiDAR thì tay
> **không** hiện và còn che mất vật phía sau; đưa tay ra xa hơn 15 cm thì điểm hiện ngay.

`lidar_mount_offset_deg` (mặc định −90) được `fusion_node` gửi trong `/lane/status`
(`lofs=`), nên GUI và C++ luôn dùng cùng một góc lắp.

### Topic

| Topic | Kiểu | Tần suất | Nội dung |
|---|---|---|---|
| `/lane/status` | `std_msgs/String` | 10 Hz | `two_lanes=1 track=two dev=-12 emg=0 run=1 age=12ms proc=1.0ms fps=26.0 cam=1920x1080 gate=0 lidar=ok front=142cm left=.. right=.. rear=.. alert=CLEAR lofs=-90.0 serial=open w=55cm devm=-4.0cm curv=0.120 scale=0.92 spd=2.8 kmh=0.00 fbage=12 servo=96.0` |
| `/autocar/run` | `std_msgs/Bool` | 10 Hz (GUI phát) | `true` = cho xe chạy |
| `/autocar/dbg/lane_vis/compressed` | `CompressedImage` (jpeg) | = fps camera | ảnh 640x360 + overlay detector |
| `/autocar/dbg/lane_roi/compressed` | `CompressedImage` (jpeg) | = fps camera | vùng ROI |
| `/autocar/dbg/lane_bin/compressed` | `CompressedImage` (png) | = fps camera | mask nhị phân |
| `/autocar/dbg/cam_raw/compressed` | `CompressedImage` (jpeg) | = fps camera | ảnh camera 640x360 chưa vẽ |
| `/lane/vis` | `sensor_msgs/Image` (bgr8) | = fps camera | overlay, cho `rqt_image_view` |
| `/scan` | `sensor_msgs/LaserScan` | ~8 Hz | do driver LiDAR phát |

Các topic ảnh chỉ được phát khi có subscriber.

### Tham số launch

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `serial_port` | `/dev/ttyUSB0` | Cổng UART của ESP32 (không tồn tại ⇒ tự dò) |
| `lidar_port` | `/dev/ttyUSB1` | Cổng UART của LiDAR |
| `camera_index` | `-1` | -1 = tự dò 0..3 |
| `camera_width` / `camera_height` | `1920` / `1080` | độ phân giải xin camera (MJPG) |
| `camera_fps` | `30` | FPS yêu cầu |
| `camera_exposure` | `-1` | -1 = phơi sáng tự động; > 0 = phơi sáng tay (100 µs), vd `250` để giữ 30 fps |
| `horizon_frac` | `0.20` | **Chân trời** (tỉ lệ chiều cao ảnh). Vạch tím trên overlay phải đi qua điểm 2 vạch thẳng kéo dài gặp nhau. Sai giá trị này ⇒ ghép nhầm cặp vạch |
| `camera_height_m` | `0.30` | Độ cao camera so với mặt đường (m), dùng đổi pixel → cm |
| `roi_top_frac` | `0.52` | Đầu ROI tính từ trên xuống; nhỏ hơn = thấy xa hơn |
| `speed_x10` | `30` | Tốc độ khi đủ 2 làn & đi thẳng; cua tự giảm theo độ cong (km/h × 10) |
| `speed_hold_x10` | `15` | Tốc độ giữ hướng khi mất cả 2 vạch |
| `speed_corner_x10` | `15` | Tốc độ khi chỉ thấy 1 vạch |
| `speed_ramp_x10` | `8` | Mức tăng tốc, x10 mỗi giây (`8` = 0.8 km/h/s) |
| `speed_min_x10` | `25` | Tốc độ nhỏ nhất khi đang chạy; bánh không quay thì tăng |
| `lane_lost_stop_ms` | `400` | Mất 2 làn quá lâu thì dừng |
| `dev_sign` | `1` | `-1` nếu servo lắp ngược |
| `require_start` | `true` | `true` = chỉ chạy khi bấm SPACE trên dashboard |
| `start_timeout_ms` | `600` | Mất heartbeat `/autocar/run` quá mức này ⇒ dừng |
| `control_hz` | `100` | Tần suất gửi lệnh xuống ESP32 |
| `viz_hz` | `30` | Tần suất tối đa gửi ảnh cho dashboard |
| `status_hz` | `10` | Tần suất `/lane/status` |
| `enable_viz` | `true` | Tắt để nhẹ máy |
| `enable_lidar` | `true` | Bật driver LiDAR + subscribe `/scan` |
| `lidar_mount_offset_deg` | `-90.0` | Góc lệch lắp LiDAR, `0` = trước, `90` = trái (REP-103) |
| `log_level` | `info` | `debug` để in nhiều hơn |

## Detector 2 làn (cửa sổ trượt)

**Camera 1920x1080 MJPG** → tự giải mã bằng libjpeg-turbo ở **1/2 kích thước** (960x540,
~7 ms thay vì ~15 ms, và tắt cảnh báo `Corrupt JPEG data` mà camera này gây ra mỗi frame)
→ cắt ROI → xám → thu về **320 cột, giữ đúng tỉ lệ 16:9 (320x180)** → Gaussian blur(5,5)
→ **trừ nền** → Otsu + **ngưỡng trễ** trong hình thang → `CLOSE(5,15)` + `OPEN(3,3)` → sửa
đứt đoạn.

Khung làm việc giữ đúng tỉ lệ ảnh gốc (1920x1080 → 320x180, 640x480 → 320x240), nên
tiêu cự dọc = tiêu cự ngang và công thức pinhole đúng với mọi độ phân giải.

**Cửa sổ trượt** — 6 cửa sổ, 3 cách quét, lấy cách có điểm cao nhất:

| Cách quét | Khi nào thắng |
|---|---|
| **PRIOR** — tìm quanh vị trí vạch của frame trước, biên ±18 px | xe đang bám làn tốt (ưu tiên khi hoà điểm ⇒ không giật) |
| **XA → GẦN** — tìm cặp vạch ở cửa sổ xa, đoán vị trí theo phối cảnh | mới khởi động, vừa bắt lại làn |
| **GẦN → XA** | vào cua gắt, vạch xa đã ra khỏi khung |

Chọn cặp vạch theo bề rộng làn đã học (và cân đối quanh giữa ảnh), loại vạch rộng hơn
bề rộng băng keo theo phối cảnh, loại cửa sổ 2 vạch chéo nhau hoặc quá sát.

**Hình học camera (quan trọng):** camera trên xe cúi xuống ~13° nên chân trời nằm ở ~20%
chiều cao ảnh, không phải giữa ảnh. Bản trước giả định camera nhìn ngang (50%) ⇒ đổi
pixel → mét sai ⇒ cặp 2 băng keo thật bị coi là "làn quá rộng" và bị loại, detector ghép
vạch sàn bên ngoài với 1 băng keo ⇒ báo `1 LANE`, `dev` −147 px, servo đánh hết lái. Với
`horizon_frac = 0.20` và giới hạn bề rộng 1 vạch nới lên 12 cm (băng keo bản rộng), cả 3
ảnh chụp thật đều nhận đúng 2 băng keo.

**Chống lóa đèn trần — trừ nền (background subtraction):**

1. **Ảnh nền** = phép đóng hình thái học (giãn rồi co) cửa sổ 61×5 px: mọi vật tối hẹp hơn
   cửa sổ (vạch) bị "lấp" bằng màu sàn ngay cạnh, còn vệt lóa và độ sáng chung giữ nguyên.
2. `diff = nền − ảnh`: vạch (kể cả đoạn bị lóa tráng sáng) ⇒ diff dương; sàn, lóa, vùng
   sáng/tối dần ⇒ ~0; vật tối **rộng** hơn cửa sổ (bóng ghế) ⇒ nền tối theo ⇒ ~0, tự loại.
3. **Ngưỡng trễ (hysteresis):** đoạn vạch trong vùng lóa chỉ còn diff 15–35 trong khi đoạn
   ngoài lóa ~100, một ngưỡng Otsu sẽ cắt mất. Giữ pixel yếu (diff > 8) nếu nó **nối liền**
   với pixel mạnh (diff > ngưỡng Otsu) ⇒ đoạn vạch bị lóa được nối lại với phần vạch còn rõ;
   vân gỗ, vết bẩn rời rạc vẫn bị bỏ.

Thử trên ảnh tổng hợp có lóa phản xạ gương (độ tương phản còn 15%): bản cũ làm đứt vạch và
sinh vệt giả hình cung ở rìa vùng lóa; bản mới cho mask liền mạch, không vệt giả, 10/10
frame `2 LANES`. Đoạn vạch bị lóa trắng hoàn toàn (diff ≈ 0) vẫn được bước "sửa đứt đoạn"
nội suy lại.

**Ổn định hoá:**

1. **Fit loại điểm lệch** — fit đường tâm bậc 2, bỏ tối đa 2 điểm lệch > 7 px rồi fit lại
   (bóng đổ, vật lạ không kéo cả đường tâm).
2. **Chặn nhảy** — `dev` mới lệch giá trị đang lọc > 60 px (ảnh 640) thì giữ giá trị cũ;
   lệch như vậy 2 frame liền mới tin (vào cua thật). Overlay hiện dấu `*`.
3. **EMA 0.40** trên `dev`.
4. **Sửa đứt đoạn** (đèn trần rửa sáng vạch): nội suy vẽ lại tối đa 3 cửa sổ.

**Quy ước `dev_px`:** luôn tính theo ảnh **tham chiếu rộng 640 px**, bất kể camera chạy
độ phân giải nào. Firmware (deadzone 10 px, bão hoà 50 px) được chỉnh theo 640 px; bản
cũ nhân theo ảnh gốc nên ở 1920x1080 `dev` lớn gấp 3 ⇒ đánh lái quá tay, xe lắc.

Đo bề rộng làn ra cm bằng pinhole `W = w_px · h / (cos(pitch) · (y − horizon))`, không IPM.

Kiểm thử ngoại tuyến: `CameraLane::process(bgr, out)` xử lý 1 ảnh có sẵn không cần camera.
Trên chuỗi 40 ảnh tổng hợp 1920x1080 (nhiễu, vạch đứt, vật tối giữa làn) detector giữ
`2 LANES` mọi frame, bề rộng đo 54.2–54.6 cm (thật 55 cm), xử lý ~0.9 ms/frame.

## Giao thức UART (230400 8N1)

**Mini PC → ESP32 (11 byte)**
```
AB CD | DEV_H DEV_L | SPEED_X10 | EMG | 00 00 00 00 | XOR(byte 2..9)
```
**ESP32 → Mini PC (~50 Hz)** — driver nhận cả 2 dạng:
```
v1  7 byte : DC BA | float32 velocity_kmh | XOR(byte 2..5)
v2 10 byte : DC BB | float32 velocity_kmh | ESC_DEG | STEER_DEG | FLAGS | XOR(byte 2..8)
```
v2 (firmware hiện tại) báo **mức xung ESP32 thật sự đang phát** cho ESC/servo
(`ESC_DEG` 90 = dừng, > 90 = tiến) và cờ: bit0 EMG, bit1 watchdog (mất lệnh > 500 ms),
bit2 đang phanh, bit3 đã nhận gói lệnh. Dashboard hiện ở ô **ESC (ESP32)**; Mini PC gửi
lệnh chạy mà ESP32 vẫn phát 90 ⇒ trạng thái `NO THROTTLE` kèm lý do.

> ⚠️ **Dừng xe bằng EMG.** Firmware hiện tại coi `speed = 0` là dừng (bản cũ kẹp lên 95 và
> xe bò), nhưng EMG vẫn là tín hiệu dừng chính thức. Watchdog 500 ms phía ESP32 là lưới an
> toàn cuối.

**Vùng chết ESC.** `Servo.write(góc)` phát xung `1000 + góc·1000/180` µs: 90 → 1500 µs
(dừng), 95 → 1527, 97 → 1538, 100 → 1555 µs. ESC xe RC có vùng chết quanh 1500 µs (thường
±30–50 µs) nên xung 95–97 bị coi là dừng: **giao tiếp đúng nhưng bánh không quay**. Firmware
giờ không bao giờ ra ga dưới `ESC_START_FWD = 100` khi đang chạy, và "đề-pa" `ESC_KICK_FWD =
106` trong 300 ms khi bắt đầu lăn. Đo lại cho xe của bạn: đặt xe dưới sàn, chạy sketch
`motor_test`, tăng dần từ 1% cho tới khi bánh vừa quay, đổi ra góc `95 + (pct−1)·85/99` rồi
ghi vào `ESC_START_FWD`.

Driver Mini PC mở cổng ở chế độ raw non-blocking (không bao giờ chặn vòng điều khiển), hạ
DTR/RTS cùng lúc để ESP32 không bị reset, và **tự mở lại cổng mỗi giây** khi rút cáp.

## Vòng điều khiển

| Trạng thái | `dev` | `speed` | `emg` |
|---|---|---|---|
| chưa bấm SPACE / mất heartbeat dashboard | 0 | 0 | **1** |
| camera OK + đủ 2 làn | `dev_px × dev_sign` | `speed_x10 × speed_scale` (ramp) | 0 |
| camera OK, chỉ 1 vạch | `dev_px × dev_sign` | `min(speed_corner_x10, speed_x10 × speed_scale)` | 0 |
| mất cả 2 vạch < `lane_lost_stop_ms` | giữ hướng lái cuối | `speed_hold_x10` | 0 |
| mất cả 2 vạch ≥ `lane_lost_stop_ms` | 0 | 0 | **1** |
| camera mất frame (> 200 ms) | 0 | 0 | **1** |

Tốc độ **giảm ngay** khi cần, chỉ **tăng dần** theo `speed_ramp_x10`. Node tắt ⇒ gửi EMG
3 lần trước khi đóng serial.

## ESP32 firmware

> ⚠️ **Kiểm tra chân ESC.** Firmware xe dùng servo = GPIO **32**, ESC = GPIO **33**, nhưng
> 2 sketch thử `motor_test` / `servo_test` dùng ESC = GPIO **14**, servo = GPIO **16**.
> Dây tín hiệu ESC cắm chân 14 mà firmware phát ra chân 33 ⇒ ESC không bao giờ nhận lệnh.
> Ô **ESC (ESP32)** trên dashboard > 90 mà bánh không quay ⇒ kiểm tra dây/chân, nguồn ESC,
> mass chung giữa ESP32 và ESC, rồi mới tới tốc độ (`speed_min_x10`).

Chip là **ESP32 thường** (thư mục vẫn tên `esp32s3/` cho khớp lịch sử repo). Arduino IDE:
Board **ESP32 Dev Module**, **Core Debug Level = None** (log ESP-IDF in ra UART0 sẽ chen vào
khung nhị phân). `Serial` = UART0 = CP2102. Tắt `run.sh` trước khi nạp (cổng đang bận).

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 --build-path /tmp/esp32build \
  firmware/esp32s3/autonomous_vehicle
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32 --input-dir /tmp/esp32build
```

**Lái** (`calcSteer`): `góc = 90 ± STEER_KP · map(dev) + D`, với `map`: deadzone 10 px →
0°, 50 px → 30°; `STEER_KP = 0.8`; khâu D tính trên tốc độ đổi góc đã lọc, giới hạn ±6°;
servo quay tối đa 300°/s. Bản cũ tính `D = 1.5·Δgóc/0.01 s` nên mỗi frame camera mới làm
D vọt lên hàng chục độ ⇒ servo giật hết lái rồi mới về. Xe lắc qua lại thì giảm
`STEER_KP`, vào cua không đủ gắt thì tăng.

> Monitor serial **không** in chữ khi xe đang chạy: cổng đó mang protocol nhị phân.
> Muốn xem trạng thái thì dùng `/lane/status` hoặc dashboard.

## Checklist trước khi chạy xe thật

1. Kê bánh, có người cầm công tắc nguồn.
2. Dashboard: 4 đèn FUSION / CAMERA / LIDAR / ESP32 xanh, ô lớn hiện `2 LANES`.
3. Chưa bấm SPACE ⇒ trạng thái `STOPPED`, ESC ở neutral.
4. Bấm SPACE: xe đặt lệch phải ⇒ `dev` **âm** ⇒ servo **> 90** (lái trái về giữa). Sai
   chiều thì chạy lại với `dev_sign:=-1`.
5. Bấm SPACE lần nữa / ESC / đóng dashboard ⇒ xe dừng ngay.
6. Che camera ⇒ `EMERGENCY`, ESC về neutral, node không crash.
7. Rút cáp ESP32 ⇒ đèn ESP32 đỏ, node vẫn chạy; cắm lại ⇒ tự kết nối lại sau ~1 s.

## Lịch sử phát triển

29/09/2026 → 07/10/2026. Chi tiết từng lỗi đã sửa: `docs/FIX_REPORT.md`
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

### 07/10 — UART, dashboard SPACE, ổn định bám làn

| Nội dung |
|---|
| Giao tiếp ESP32 chuyển hẳn sang **UART** (CP2102, `/dev/ttyUSB0`): driver raw non-blocking, tự dò bằng gói telemetry, tự kết nối lại |
| Camera 1920x1080: tự giải mã MJPG 1/2 bằng libjpeg (17 → 26 fps), khung làm việc đúng tỉ lệ 16:9, `dev_px` chuẩn hoá về ảnh 640 |
| Detector: quét theo frame trước, fit loại điểm lệch, chặn nhảy; firmware bỏ khâu D gây giật servo, thêm giới hạn tốc độ servo |
| `fusion_node` đa luồng (nén ảnh không chặn điều khiển), chỉ phát ảnh khi có người xem, lệnh chạy/dừng `/autocar/run` |
| Dashboard mới: overlay C++, nút SPACE, đèn kết nối, bản đồ LiDAR theo khung xe (trước ↑); `tools/install_deps.sh` |

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