# HƯỚNG DẪN HIỆU CHỈNH NHANH

## 1. ROI
Thay đổi `roi_factor_dual` / `roi_factor_single` (0–1, mép trên=xa, mép dưới=gần).

```bash
ros2 param set /autonomous_vehicle_node roi_factor_dual 0.60
ros2 param set /autonomous_vehicle_node roi_factor_single 0.75
```

## 2. Mask nhanh
Chỉ chỉnh khi `edge_quality` thấp hoặc mất blob.

Các tham số quan trọng trong `lane_mask::Options` (sẽ được expose qua params sau): `dark_band`, `p_low/p_high`, `min_area`, `min_height`, `require_edge`, `edge_peak_ratio`, `enable_light_lanes`.

## 3. Debug log
Trong `camera_lane.hpp`: `ENABLE_DEBUG_LOG=true`, `DEBUG_LOG_EVERY=15`. Quan sát `roi`, `p_lo/hi`, `sobel_p99`, `blobs`, `dual`, `conf`.

## 4. Ẩn GUI/Debug
Tiết kiệm tài nguyên:
```bash
# tắt debug
ros2 run autonomous_vehicle autonomous_vehicle_node --ros-args -p enable_dbg:=false

# chạy chỉ core
ros2 launch autonomous_vehicle minipc.launch.py
```
