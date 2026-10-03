# KIẾN TRÚC HỆ THỐNG

## 1. Packages
- `autonomous_vehicle`: ROS2 node chính (CameraLane + LiDAR + ObstacleAvoidance + Serial). Publish `/autocar/dbg/*` (JPEG compressed + JSON).
- `autonomous_vehicle_gui`: Qt5 dashboard độc lập, subscribe `/autocar/dbg/*`, layout y hệt MATLAB.

## 2. Topics Debug
| Topic | Type | Dùng cho |
|---|---|---|
| `/autocar/dbg/status` | `std_msgs/String` (JSON) | Toàn bộ trạng thái (lidar/lane/oa/esp/ai/pid) |
| `/autocar/dbg/cam_raw/compressed` | `CompressedImage` JPEG | RAW camera |
| `/autocar/dbg/lane_vis/compressed` | `CompressedImage` JPEG | Overlay làn |
| `/autocar/dbg/lane_bin/compressed` | `CompressedImage` JPEG | Binary mask (320x200) |
| `/autocar/dbg/lane_roi/compressed` | `CompressedImage` JPEG | Vùng ROI thật (`roi_y0`) |

## 3. Tối ưu RAM/CPU
- JPEG Q=80 (thay vì raw), compressed image.
- Timer tách: `control_hz` (100), `viz_hz` (10), `dbg_rate_hz` (10). Có thể hạ dbg xuống 5–8.
- `enable_dbg` có thể tắt hoàn toàn khi không dùng GUI.
- Buffer frame 2 (`FRAME_BUFFER_COUNT=2`), Latest-Frame, không queue sâu.
- Qt vẽ QPainter (ít allocation), không Qwt.

## 4. Quan trọng
- `roi_y0` từ detector (không suy lại) → ROI hiển thị chính xác.
- Ma trận 3x3 polyfit sửa A[8], A[6], A[7] (Sy⁴, Sy², Sy³).
- Default: `enable_light_lanes=false`, `bonnet_enable=false` (phòng).
- Stale handling: camera/lidar tuổi dữ liệu → chế độ an toàn.
