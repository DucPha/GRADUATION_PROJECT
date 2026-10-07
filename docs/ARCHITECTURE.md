# KIẾN TRÚC HỆ THỐNG

## 1. Packages

| Package | Loại | Vai trò |
|---|---|---|
| `camera_node` | thư viện C++ (OpenCV + libjpeg) | `CameraLane`: đọc + giải mã MJPG, nhận 2 làn bằng cửa sổ trượt, vẽ overlay. Không phụ thuộc rclcpp |
| `esp32s3_node` | thư viện C++ | `SerialESP32`: UART 11/7 byte, tự dò cổng bằng telemetry, tự kết nối lại |
| `sllidar_ros2_node` | thư viện C++ | `LidarModule` + `ObstacleAvoidance` (chưa nối vào phase 1) |
| `fusion_node` | node ROS 2 | node điều khiển duy nhất: nối 3 thư viện trên + timer + publisher |
| `src/gui/gui.py` | script Python | dashboard PySide6: hiển thị + nút SPACE chạy/dừng |
| `traffic_light_detector`, `turn_detector` | node Python | detector NCNN, chưa launch trong phase 1 |

Ba thư viện C++ được `ament_export_libraries` + `ament_export_include_directories`, nên
`fusion_node` chỉ cần `find_package(...)` rồi `ament_target_dependencies`.

## 2. Luồng dữ liệu

```
USB camera MJPG 1920x1080
      │  CameraLane::capture_loop()   luồng riêng: đọc gói MJPG → libjpeg 1/2 (960x540)
      │                               → detect() → (vẽ overlay 640x360 nếu có người xem)
      ▼
  LaneOutput { state, dev_px (ảnh 640), dev_cm, width, curvature, speed_scale, ... }
      │
      ├── control_tick()   100 Hz  ─► SerialESP32 ─► UART /dev/ttyUSB0 ─► ESP32 ─► servo + ESC
      │        ▲
      │        └── /autocar/run (Bool, 10 Hz từ dashboard: SPACE)
      ├── viz_tick()       ≤30 Hz  ─► /autocar/dbg/lane_{vis,roi,bin}/compressed, /lane/vis
      └── status_tick()     10 Hz  ─► /lane/status (key=value)

RPLIDAR A1 ─► rplidar_ros ─► /scan ─┬─► LidarModule::update() (khoảng cách 4 hướng, cảnh báo)
                                    └─► dashboard (bản đồ polar, khung xe)
```

Điều khiển đọc detector **trong cùng tiến trình** (không qua topic) nên không có độ trễ
ROS. Topic chỉ để quan sát và ghi rosbag.

## 3. Luồng và nhóm callback

`fusion_node` chạy `MultiThreadedExecutor` 2 luồng:

| Nhóm | Callback | Ghi chú |
|---|---|---|
| `ctrl_group_` | `control_tick` 100 Hz, `status_tick` 10 Hz, `/scan`, `/autocar/run` | cùng nhóm ⇒ dùng chung biến không cần khoá |
| `viz_group_` | `viz_tick` | nén JPEG/PNG ở luồng riêng ⇒ không làm trễ điều khiển |

`CameraLane` có thêm luồng camera riêng. Ảnh overlay chỉ được vẽ khi `viz_tick` vừa lấy
ảnh (cờ `vis_wanted_`), và `viz_tick` chỉ lấy ảnh khi topic ảnh có subscriber ⇒ không ai
xem thì không tốn CPU vẽ/nén.

## 4. Topic

| Topic | Kiểu | Tần suất | Dùng cho |
|---|---|---|---|
| `/lane/status` | `std_msgs/String` | 10 Hz | trạng thái key=value, dashboard đọc |
| `/autocar/run` | `std_msgs/Bool` | 10 Hz (dashboard) | lệnh chạy + heartbeat |
| `/autocar/dbg/lane_vis/compressed` | `CompressedImage` jpeg | = fps camera | overlay detector |
| `/autocar/dbg/lane_roi/compressed` | `CompressedImage` jpeg | = fps camera | vùng ROI |
| `/autocar/dbg/lane_bin/compressed` | `CompressedImage` png | = fps camera | mask nhị phân |
| `/autocar/dbg/cam_raw/compressed` | `CompressedImage` jpeg | = fps camera | ảnh chưa vẽ |
| `/lane/vis` | `sensor_msgs/Image` bgr8 | = fps camera | `rqt_image_view` |
| `/scan` | `sensor_msgs/LaserScan` | ~8 Hz | driver `rplidar_ros` |

## 5. Dashboard

- GUI **không xử lý ảnh**: chỉ giải mã ảnh và đọc số liệu C++ đã tính, nên luôn đồng bộ
  với bộ điều khiển.
- Luồng ROS (rclpy) giải mã ảnh, giữ bản **mới nhất** của mỗi nguồn (QoS best-effort,
  depth 1 ⇒ không xếp hàng, không trễ). Luồng Qt kiểm tra 60 lần/giây, chỉ vẽ lại khi có
  khung mới; số liệu cập nhật 10 Hz; bản đồ LiDAR vẽ bằng blit.
- An toàn: dashboard chỉ cho bấm chạy khi `/lane/status` còn sống; mất `/lane/status`
  > 1.5 s thì tự bỏ lệnh chạy; đóng dashboard gửi `false` rồi mới thoát.

## 6. An toàn

- **Dừng xe = `emg = 1`**, không phải `speed = 0`: firmware kẹp PWM ở `[95,180]`.
- Khởi động luôn ở trạng thái dừng; chỉ chạy khi có `/autocar/run = true` còn mới
  (≤ `start_timeout_ms`).
- Watchdog 500 ms trên ESP32 là lưới cuối khi Mini PC treo.
- Camera mất frame > `STALE_AGE_MS` ⇒ EMG ngay.
- Mất 2 làn ⇒ hạ tốc độ, quá `lane_lost_stop_ms` ⇒ EMG.
- Node tắt ⇒ gửi EMG 3 lần trước khi đóng serial.
- `ObstacleAvoidance` **không** được gọi ở phase 1.
