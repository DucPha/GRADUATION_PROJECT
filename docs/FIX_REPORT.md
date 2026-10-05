# Báo cáo lỗi & đồng bộ toàn hệ thống

Ngày: 2026-10-01 · Phạm vi: `src/` (ROS 2) + `firmware/` (ESP32-S3) + `Dockerfile` + launch + docs

Trạng thái: **đã sửa toàn bộ** (A1–A20, B1–B14), build xanh, 10/10 test LiDAR pass.
Chưa commit — toàn bộ nằm trong working tree.

---

## A. LỖI CHÍT MẠNG (xe không chạy / chạy sai)

| # | File | Vấn đề | Hậu quả | Trạng thái |
|---|------|--------|----------|-----------|
| A1 | `lidar_module.cpp` | Vùng góc quét **lệch 90°** so với quy ước ROS (0° = phía trước). `ob_front_cm` lấy góc 70..110° → thực chất là **bên trái xe**, không phải phía trước | Xe **không bao giờ** thấy vật cản phía trước → không giảm tốc, không phanh, lao vào vật cản | ✅ Chuyển toàn bộ tính toán sang **khung xe** (REP-103) + tham số `lidar_mount_offset_deg` (mặc định `-90°`, giữ nguyên hành vi phần cứng cũ). Test `FrontObstacleHitsFrontSectorOnly` chốt hồi quy |
| A2 | `lidar_module.cpp` | `ob_right_rear` = `270..360 ∪ 0..69` và `ob_left_rear` = `111..270` (160° trùng nhau) | Điều kiện "đã lách qua vật cản" luôn sai → kẹt BYPASS vĩnh viễn | ✅ Biên vùng nửa mở `[start, end)` qua `in_sector()`; thêm `ob_rear_cm`; 4 vùng 45° + 4 vùng 90° không chồng lấn. Test `RearLeftAndRearRightAreDisjoint`, `EveryAngleMapsToConsistentSector` |
| A3 | `obstacle_avoidance.cpp` | `handle_return_lane_*` gọi `handle_normal(lidar, cmd.dev_final_px, 0.0f)` — truyền **±80 px hard-code** thay vì sai số camera thật | Sau khi trở lại làn, xe bẻ lái 80px về một bên **vĩnh viễn**, không bao giờ thẳng lại | ✅ Cache `dev_px_`/`dominant_slope_` mỗi `update()`; bù lái còn `RETURN_STEER_PX = 20` + kẹp `MAX_STEER_PX = 60` (firmware tự kẹp ở `CAM_MAX_DEV = 50`) |
| A4 | `obstacle_avoidance.cpp` | `handle_normal` trả `speed_control = 35` (3,5 km/h) | Camera tính 8,5 km/h bị `min()` xuống 3,5 km/h **mọi lúc** → xe chạy chậm như rùa | ✅ `SPEED_CEILING_NORMAL = 255` để camera quyết định ở NORMAL; tốc độ camera gộp bằng `min()` trong fusion node |
| A5 | `obstacle_avoidance.cpp` | `get_state_elapsed_ms()` dùng `system_clock` | Đồng hồ nhảy/lùi làm timeout sai | ✅ `steady_clock` qua `now_ms()` |
| A6 | `obstacle_avoidance.cpp` | ~20 `std::cout` không throttle, gọi ở 120 Hz | Tràn console, làm chậm vòng lặp điều khiển | ✅ Xoá hết; trạng thái quan sát qua HUD `bypass_state` |
| A7 | `fusion_viz_node.cpp` | **Không có watchdog LiDAR** | LiDAR rớt → giữ vật cản cuối cùng → xe chạy vào tường | ✅ `lidar_stale_timeout_s = 0.5`; `!lidar_stale` thành một điều kiện của `sensors_ok` |
| A8 | `fusion_viz_node.cpp` | **Không có watchdog camera** | Camera treo → giữ `dev_final_px` cũ → xe bẻ lái vào vạt | ✅ `camera_stale_timeout_s = 1.0`, phát hiện bằng `frame_id` đóng băng |
| A9 | `fusion_viz_node.cpp` | Quyết định `/turn_detector/decision` chỉ hiển thị, **không điều khiển** | Biển rẽ không được thi hành | ✅ Trộn thành bù lái (`turn_blend_px = ±30`) + trần tốc độ (`turn_speed_x10 = 40`) |
| A10 | `Autonomous_Vehicle.ino` | `Serial.printf` chẩn đoán 1 Hz **trên đúng UART mang protocol nhị phân** | Chèn ASCII vào luồng 7-byte → MiniPC mất gói, giảm độ trễ, có thể desync state machine | ✅ **Đã gỡ hẳn** toàn bộ `printDiagnostics()`/`Serial.printf` (không gate lại bằng cờ). Giữ lại UART chỉ dùng `Serial.write` nhị phân + ghi chú cấm in ở đúng chỗ |
| A11 | `Autonomous_Vehicle.ino` | `emg_started` chỉ phanh **đúng 1 lần** cho mỗi cờ `emg_stop` | Phanh 200 ms rồi thả; bánh còn quay mà không còn xung phanh nào | ✅ Bỏ `emg_started`; `checkSafety()` giữ phanh khi `car.cur_spd > STOPPED_KMH (0.3 km/h)` |
| A12 | `bringup.launch.py` | `cam_index`/`camera_fps` truyền **chuỗi** không ép kiểu | `declare_parameter("cam_index", 0)` nhận string → node **crash lúc khởi động** | ✅ `ParameterValue(..., value_type=int/float)` cho mọi tham số |
| A13 | `bringup.launch.py` | Đường dẫn rviz hard-code `~/Documents/GRADUATION_PROJECT/...` | RViz không chạy trên Mini PC (path khác) | ✅ `get_package_share_directory` |
| A14 | `traffic_light_ncnn.py` | `max_det = 1` + NMS **không theo lớp** | Biển "20" điểm cao hơn → xoá mất đèn đỏ → xe không dừng | ✅ NMS per-class |
| A15 | `traffic_light_ncnn.py` | `process_latest` xử lý **lại cùng một frame** ở 30 Hz | `red_frame_count >= 2` đạt từ **một** khung hình → báo đỏ giả liên tục | ✅ Định danh frame `frame_seq` / `last_inferred_seq`, chỉ infer frame mới |
| A16 | `traffic_light_ncnn.py` | `red` bị hạ xuống `yellow` khi HSV nghiêng vàng | Đèn đỏ bị đổi thành vàng → **xe chạy qua ngã tư đỏ** | ✅ Bỏ hoàn toàn nhánh hạ cấp |
| A17 | `serial_esp32.cpp` | Không reconnect khi USB cổng serial bị rút/cắm lại | Xe chết vĩnh viễn sau một lần đụng dây | ✅ Cờ `link_down()` set trên `POLLERR/POLLHUP/POLLNVAL/EIO` và write ngắn; node đóng cổng và thử `open()` lại mỗi `SERIAL_RETRY_PERIOD_S = 3.0` |
| A18 | `camera_lane.cpp` | Không reopen camera khi USB camera rớt | Xe mất đường, vẫn giữ lái cũ | ✅ Sau `REOPEN_AFTER_READ_FAILS = 30` lần đọc hỏng liên tiếp thì mở lại; `apply_camera_settings()` gom cấu hình vào một chỗ |
| A19 | `serial_esp32.cpp` | `select(5ms)` + `sleep_for(5ms)` không điều kiện | Trễ 2× vòng poll, đốt CPU | ✅ `poll(..., 20ms)` |
| A20 | `fusion_viz_node.cpp` | Publish `/image_raw` (768 KB) ở 120 Hz | 92 MB/s, bão hòa DDS, AI node nghẽn | ✅ Giới hạn theo `raw_image_hz = 5.0` |

---

## B. LỆCH ĐỒNG BỘ (contract)

| # | Vấn đề | Sửa | Trạng thái |
|---|--------|-----|-----------|
| B1 | `obstacle_avoidance` hard-code `/traffic_light/decision`; node Python hard-code topic của nó | Tham số `traffic_light_topic` / `turn_detector_topic` hai chiều, launch truyền chung | ✅ |
| B2 | `ai_timeout_s` khai báo trong launch nhưng code C++ hard-code `2.0` | Node đọc tham số, launch truyền xuống | ✅ |
| B3 | `baudrate` khai báo trong launch nhưng C++ hard-code `230400` | Tham số `baudrate` cho C++ | ✅ |
| B4 | Hai launch file `bringup` và `minipc` trùng lặp, mặc định khác nhau (120 Hz vs 30 Hz) | Một nguồn: `bringup.launch.py`; `minipc.launch.py` là wrapper `IncludeLaunchDescription` | ✅ |
| B5 | `package.xml` khai `ament_cmake_auto` nhưng CMakeLists dùng `ament_cmake` | Khớp lại | ✅ (`ament_cmake` + `ament_cmake_gtest`, `ament_index_python`, `rviz2`) |
| B6 | README ghi `ros-humble-*` / `ROS 2 Humble/Iron`; máy đang chạy **Jazzy** | Ghi đúng, hướng dẫn theo `$ROS_DISTRO` | ✅ |
| B7 | `CLAUDE.md` mô tả `firmware/legacy/`, `launch/` ở gốc — không tồn tại | Sửa lại cây thư mục + mục protocol | ✅ |
| B8 | `Dockerfile` `pip3 install opencv-python` cạnh `python3-opencv` | Gây lỗi ABI numpy; bỏ khỏi pip | ✅ Thêm `ROS_DISTRO` build arg (mặc định `jazzy`), copy `rosdep.yaml` vào `src/` |
| B9 | `docker-entrypoint.sh` gọi `sudo` trong container không có sudo, chạy non-root | Bỏ `sudo` | ✅ |
| B10 | `rosdep.yaml` dùng key không tồn tại (`ncnn-vulkan`, `python3-ncnn-vulkan`) → `rosdep install` fail | Chỉ giữ key rosdep hợp lệ; ncnn chuyển hoàn toàn sang `pip` | ✅ |
| B11 | `FWD_THRESHOLD_PX = 15` ở C++ vs `CAM_DEADZONE = 10` ở ESP32 | Thống nhất 1 ngưỡng | ✅ `FWD_THRESHOLD_PX = 10` + ghi chú ràng buộc với `CAM_DEADZONE` |
| B12 | `BYPASS_MIN_SPACE` / `BYPASS_COMPLETE_DISTANCE` khai ở `LidarModule` nhưng không ai dùng | Bỏ bản sao ở `LidarModule`; đặt ngưỡng ở đúng chỗ dùng | ✅ `MIN_BYPASS_SPACE_CM`, `BYPASS_REAR_CLEAR_CM`, `BYPASS_SIDE_OPEN_DELTA_CM`, `BYPASS_SETTLE_MS` trong `ObstacleAvoidance`, thay cho số 30.0/5.0/500 rời rạc |
| B13 | Topic AI: AI node không đổi được topic đầu vào | Thêm tham số `image_topic` cho cả 2 node Python | ✅ |
| B14 | AI publish `String` 30 Hz kể cả khi không đổi | Chỉ publish khi giá trị đổi | ✅ Có **heartbeat 0.5 s** — watchdog `ai_timeout_s` (2 s) sẽ xoá quyết định nếu im lặng hoàn toàn |

---

## C. KIẾN TRÚC / AN TOÀN BỔ SUNG (đã triển khai)

- Watchdog LiDAR (`lidar_stale_timeout_s`, 0.5 s) + camera (`camera_stale_timeout_s`, 1.0 s) có tham số hóa. `sensors_ok = cam_valid && !lidar_stale && serial_link_ok` → ép `speed_control = 0` **và** `emergency_stop = true`.
- `lidar_mount_offset_deg` tham số để hiệu chỉnh góc lắp LiDAR (mặc định `-90°` = giữ nguyên hành vi cũ).
- Reconnect serial (3 s) + reopen camera (30 lần đọc hỏng).
- `steady_clock` cho toàn bộ state machine.
- Nhịp publish `/image_raw` tách riêng (`raw_image_hz`).
- Quyết định rẽ (TURN_LEFT/RIGHT) nay **điều khiển lái xe**.
- ESP32: UART chỉ còn protocol nhị phân; phanh lặp lại khi còn trượt (`STOPPED_KMH`).
- Bảng tra `ESC_LUT` + hằng số bánh xe tập trung, có ghi chú hiệu chỉnh.

## D. Hợp đồng không được đổi lặng lẽ

- **Serial**: PC→ESP32 11 byte `AB CD | dev_hi dev_lo | speed(km/h×10) | emg | 00×4 | XOR(2..9)`; ESP32→PC 7 byte `DC BA | float velocity | XOR(2..5)`. Baud 230400 hai chiều. Không thêm byte, không đổi thứ tự.
- **Dấu góc lái**: `dev_final_px < 0` = TRÁI, `> 0` = PHẢI (khớp `calcSteerPID`). `CAM_DEADZONE = 10`, `CAM_MAX_DEV = 50`.
- **Trộn tốc độ**: `speed_control = min(camera theo độ cong, trần của obstacle avoidance)`.
- `speed_control` và `dev_final_px` đều là **km/h × 10** và **pixel**, không scale lại.

---

## E. GHI CHÚ PHẦN CỨNG (cần bạn xác nhận, KHÔNG tự ý đổi)

- `PIN_STEER = 33` trên ESP32-S3: nếu module có **PSRAM octal**, GPIO 33–37 bị module dùng cho PSRAM → servo sẽ không chạy. Cần module PSRAM 2MB/quad hay đổi chân.
- Thông số bánh: `WHEEL_DIA_M = 0.090`, `PULSES_PER_REV = 4`, `GEAR_RATIO = 37/13`. Sai một trong ba là tốc độ đo sai.
- `MIN_BYPASS_SPACE_CM = 60 cm` là khoảng trống bên tối thiểu để lách; xe thật cần đo lại.
- `CAM_MAX_DEV = 50 px` là trần góc lái camera. Firmware tự kẹp ở đây, nên `MAX_STEER_PX = 60` phía PC là giá trị "chết" phía cuối — giữ để dễ đọc log.

---

## F. KIỂM CHỨNG

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select autonomous_vehicle --cmake-args -DBUILD_TESTING=ON
./build/autonomous_vehicle/test_lidar_module     # 10/10 PASSED
```

Test phủ: vùng trước không lẫn vùng bên, sau-trái/sau-phải không chồng, mọi góc 0..360° ánh xạ nhất quán, offset lắp đặt dịch vùng đúng, mức cảnh báo, scan rỗng, giá trị NaN/inf bị bỏ.

**Chưa kiểm chứng được trên máy:** hành vi thật của LiDAR/camera/động cơ, và toàn bộ đường điều khiển lái–tốc độ–phanh. Cần chạy thử có người giám sát, bánh kê gạt, mặt đất bằng phẳng.