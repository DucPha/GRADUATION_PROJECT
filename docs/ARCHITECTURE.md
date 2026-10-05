# KIẾN TRÚC HỆ THỐNG

## 1. Packages

| Package | Loại | Vai trò |
|---|---|---|
| `camera_node` | thư viện C++ (OpenCV) | `CameraLane`: nhãn 2 làn bằng cửa sổ trượt. Không phụ thuộc rclcpp |
| `esp32s3_node` | thư viện C++ | `SerialESP32`: khung nhị phân 11/7 byte, tự dò cổng USB |
| `sllidar_ros2_node` | thư viện C++ | `LidarModule` + `ObstacleAvoidance` (chưa nối vào phase 1) |
| `fusion_node` | node ROS 2 | node điều khiển duy nhất: nối 3 thư viện trên + timer + publisher |
| `gui_matplotlib` | node Python | dashboard matplotlib + bản đồ polar LiDAR, chạy cùng `fusion_node` |
| `traffic_light_detector`, `turn_detector` | node Python | detector NCNN, chưa launch trong phase 1 |

Ba thư viện C++ được `ament_export_libraries` + `ament_export_include_directories`, nên
`fusion_node` chỉ cần `find_package(camera_node)` / `find_package(esp32s3_node)` /
`find_package(sllidar_ros2_node)` rồi `ament_target_dependencies`. Header đặt ngay gốc
package và được `install(FILES ...)` vào `include/` trong không gian cài đặt, nên
`fusion_node` include phẳng (`#include "camera_node.hpp"`) là đúng.

## 2. Luồng dữ liệu

```
USB camera 640x480
      │  CameraLane::capture_loop()      1 luồng: read → detect → lưu frame mới nhất
      ▼
  LaneOutput { left_pts, right_pts, two_lanes, dev_px, age_ms, stale, vis }
      │
      ├── fusion_node::control_tick()   100 Hz  → SerialESP32::send_command()
      │                                        → USB-OTG → ESP32-S3 → servo + ESC
      └── fusion_node::viz_tick()        10 Hz   → /lane/vis  (bgr8)
/scan ──► LidarModule::update()           10 Hz  → chỉ để log, không điều khiển
```

Điều khiển đọc detector **trong cùng tiến trình** (không qua topic) nên không có độ trễ
ROS. Topic chỉ để quan sát và ghi rosbag.

## 3. Topic

| Topic | Kiểu | Tần suố | Dùng cho |
|---|---|---|---|
| `/lane/status` | `std_msgs/String` | 5 Hz | trạng thái 1 dòng, dễ đọc bằng `ros2 topic echo` |
| `/lane/vis` | `sensor_msgs/Image` bgr8 | 10 Hz | overlay 2 làn để chỉnh detector |
| `/scan` | `sensor_msgs/LaserScan` | 10 Hz | do driver `rplidar_ros` phát |

Không dùng `cv_bridge` và không dùng `jsoncpp`: ảnh điền thẳng vào
`sensor_msgs::msg::Image` bằng `memcpy` từng hàng, trạng thái là chuỗi thuần.

## 4. Phân tách timer

| Timer | Tần suố | Việc |
|---|---|---|
| `control_tick` | 100 Hz | đọc frame mới nhất → tính lệnh → gửi serial |
| `viz_tick` | 10 Hz | copy ảnh overlay và publish |
| `status_tick` | 5 Hz | dựng chuỗi trạng thái, log 2 Hz |

`get_latest()` có tham số `copy_vis`: vòng 100 Hz không copy ảnh 640x480, chỉ vòng
10 Hz mới copy.

## 5. Detector

Blur(5,5) → CLAHE → mask ROI hình thang → Otsu → morphology → 5 cửa sổ trượt từ dưới
lên trên. Mỗi cửa sổ dùng `cv::reduce(REDUCE_SUM)` để lấy histogram cột, tìm đỉnh quanh
seed riêng của từng bên. Sau đó fit đường bậc 2 qua các điểm mỗi bên.

- Không IPM, không nhánh Hough, không `margin_search`, không deque 4 frame.
  Thứ tự các bước mượn từ bài Udacity nhưng hằng số thuộc về xe mô hình.
- `dev_px` là **pixel ảnh gốc** — giữ đúng đơn vị firmware đang dùng, nên firmware
  không phải sửa.
- Đo centimet bằng `W = w_px · CAMERA_HEIGHT_M / (y − HORIZON_Y)` thay cho IPM.
  Chỉ để báo cáo: `px_to_cm` trả `0` khi quá gần chân trời (không đo được).

## 6. An toàn

- **Dừng xe = `emg = 1`**, không phải `speed = 0`: firmware kẹp PWM ở `[95,180]` nên
  `speed = 0` thành 95 ≈ 1.55 km/h.
- Watchdog 500 ms trên ESP32 là lưới cuối khi Mini PC treo.
- Camera mất frame > `STALE_AGE_MS` ⇒ EMG ngay.
- Mất 2 làn ⇒ hạ tốc độ, quá `lane_lost_stop_ms` ⇒ EMG.
- Node tắt ⇒ gửi một lệnh EMG trước khi đóng serial.
- `ObstacleAvoidance` **không** được gọi ở phase 1: nó coi LiDAR mất dữ liệu là
  emergency stop, trong khi phase 1 chưa dùng LiDAR để tránh vật cản.

## 7. Phần còn lại

`traffic_light_detector`, `turn_detector` và `ObstacleAvoidance` đều còn trong repo và
build được, nhưng chưa nối vào đường điều khiển. Nối `ObstacleAvoidance` lại phải xử lý
điều kiện LiDAR stale trước, nếu không xe sẽ đứng yên mỗi lần `/scan` tụt.