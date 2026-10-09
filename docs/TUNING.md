# HƯỚNG DẪN HIỆU CHỈNH NHANH

Mọi hằng số detector nằm trong `src/camera_node/camera_node.hpp`. Sửa ở đó
rồi build lại. Tham số nào tiện thì expose qua launch (`ros2 launch fusion_node
fusion.launch.py <tên>=<giá trị>` hoặc `./run.sh <tên>:=<giá trị>`).

## 0. Trước hết: nhìn `/lane/vis`

Ảnh overlay cho biết ngay detector đang dừng ở bước nào:

- **Ô BEV góc trên trái** = ảnh chiếu xuống mặt sàn (xe ở đáy, mũi tên trắng hướng đi):
  mask vạch màu xám sáng, phần bị lọc là đồ vật màu **tím**, ngoài tầm nhìn xanh đậm
- **Vạch trái xanh lá**, **vạch phải cam**; vạch **xám** = đang dự đoán (mất vài frame) hoặc
  1 vạch chưa xác minh (chưa từng là 1 cặp làn hợp lệ ⇒ không chạy theo)
- Vạch **xám mảnh** = vạch ứng viên không được chọn (làn bên cạnh, vết bẩn...)
- **Đường vàng** = tâm làn (vạch dời nửa làn theo pháp tuyến, góc gập nối kiểu mitre)
- **Vòng tròn vàng** = điểm ngắm pure pursuit, **dấu X tím** = góc gập phía trước
- Chữ: `2 LANES` / `1 LANE (L|R)` / `LOST`, dấu `*` = frame bị chặn nhảy
- Thanh trên: `dev` (px ảnh 640), `w` bề rộng làn đã học, `v` hệ số tốc độ, `pitch` góc
  cúi (`?` = chưa tự hiệu chỉnh), `thr` ngưỡng tối, `CUA >>`/`CUA <<` khi thấy góc gập

## 1. Bám vạch (`camera_node.hpp`)

| Hằng số | Mặc định | Khi nào chỉnh |
|---|---|---|
| `TAPE_MAX_THICK_M` / `BLOB_THICK_M` | 0.10 / 0.18 | phần dày hơn mà không thuôn dài / dày hơn hẳn = đồ vật. Băng keo bản rộng bị tô tím ⇒ tăng |
| `TRACK_GATE_M` | 0.07 | vạch mới lệch xa vị trí dự đoán hơn mức này ⇒ không phải vạch cũ |
| `COAST_SEC` | 0.5 | mất vạch ngắn hơn: giữ vạch dự đoán theo odometry |
| `MIN_KINK_TAIL_M` | 0.10 | đoạn sau góc gập ngắn hơn ⇒ không phải góc cua (rẽ vào vết bẩn) |
| `CORNER_KINK_DEG` | 30 | tâm làn gập ≥ bấy nhiêu độ thì coi là góc cua |
| `RADIAL_DEG` / `RADIAL_NEAR_GAP_M` | 8 / 0.12 | lọc chân bàn/ghế (vệt chĩa thẳng về camera, không chạm mép dưới tầm nhìn) |

## 1b. Hình học camera — chỉnh ĐẦU TIÊN

Detector chiếu ảnh xuống **mặt sàn** (BEV, 1 cm/ô, `src/camera_node/lane_bev.cpp`) nên mọi
phép đổi pixel ↔ mét dựa vào 3 số trong launch:

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `camera_pitch_deg` | 42.0 | góc cúi camera. Đo trên ảnh thật (log Python 08/10): 2 vạch thẳng kéo dài gặp nhau ở hàng −106 của ảnh 320×240 ⇒ ~42°, bề rộng làn đo lại ra đúng 42 cm |
| `camera_height_m` | 0.30 | tâm ống kính cách sàn |
| `camera_vfov_deg` | 51.0 | góc nhìn **dọc**. Chế độ 1920×1080 và 640×480 của cam này cùng góc dọc (4:3 chỉ cắt 2 bên) |
| `camera_auto_pitch` | false | tự hiệu chỉnh góc cúi khi chạy thẳng thấy 2 vạch (±8° quanh `camera_pitch_deg`). Mặc định TẮT: thử vòng kín, ước lượng nhảy 42 → 44.5 → 42.7° dù góc thật cố định, mỗi lần đổi > 2° còn xoá vạch đang bám. Bật để đo 1 lần: log in `Tu hieu chinh goc cui camera: ... -> X` ⇒ điền X vào `camera_pitch_deg` rồi tắt lại |
| (bỏ `lane_width_m`) | — | bề rộng làn không cài đặt: đo mỗi frame thấy 2 vạch (0.28–0.70 m), đường tâm = giữa 2 vạch đo được; chỉ 1 vạch thì dời nửa bề rộng vừa đo gần nhất |

Kiểm tra: đặt xe giữa 2 vạch thẳng, overlay phải hiện `2 LANES` và `w` ≈ bề rộng làn thật.
Camera cúi ~42° nên **không thấy chân trời** trong ảnh; mặt sàn nhìn thấy từ ~0.13 m tới
~1.1 m trước chân camera (ô BEV góc trên trái overlay).

## 2. Vùng làm việc

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `roi_top_frac` (launch) | 0.0 | bỏ qua phần trên cùng của ảnh (mặc định dùng hết, ~1.1 m) |
| `roi_bottom_frac` (`CameraProfile`) | 0.98 | đáy ROI; nếu thấy cản/mũi xe trong ảnh thì giảm |

Log khởi động in `geometry: ... BEV 160xN (1 cm), Z gần..xa m` = vùng mặt sàn nhìn thấy.

## 3. Lái theo `dev_px` của camera (chỉ dùng khi `steer_mode:=camera`)

| Hằng số | Mặc định | Ý nghĩa |
|---|---|---|
| `PP_LOOKAHEAD_M` | 0.55 | khoảng nhìn trước tính từ trục sau; xe lắc / ôm cua sớm ⇒ tăng, vào cua muộn ⇒ giảm |
| `PP_LOOKAHEAD_CORNER_M` | 0.80 | thấy góc gập phía trước: nhìn xa hơn để bẻ lái sớm |
| `DEV_REF_DIST_M` | 0.65 | thang đo `dev_px` (đừng đổi trừ khi chỉnh lại cả firmware) |
| `JUMP_GATE_REF_PX` | 90 | `dev` nhảy quá mức này (px ảnh 640) thì giữ giá trị cũ 1 frame |
| `EMA_ALPHA_TWO` / `EMA_ALPHA_ONE` | 0.70 / 0.50 | lọc `dev` khi 2 vạch / 1 vạch; lớn = nhạy |
| `HOLD_MS` | 500 | mất vạch: giữ `dev` cuối bao lâu |

## 4. Tốc độ

| Tham số (launch) | Mặc định | Ý nghĩa |
|---|---|---|
| `speed_x10` | 70 | đủ 2 vạch + đường thẳng (7.0 km/h ≈ ESC 103). Cua nhẹ rơi về ESC 102, cua gắt ESC 101 |
| `speed_corner_x10` | 50 | 1 vạch, hoặc 2 vạch ở cua gắt nhất (5.0 km/h ≈ ESC 101 = `ESC_CURVE` 1561 µs) |
| `speed_hold_x10` | 50 | mất cả 2 vạch, đang giữ hướng |
| `speed_min_x10` | 50 | sàn tốc độ khi đang chạy — BLDC dưới mức này kêu cọt kẹt |
| `speed_ramp_x10` | 30 | tăng tốc 3 km/h mỗi giây; giảm tốc luôn tức thì |

Đủ 2 vạch: `tốc độ = corner + (speed − corner) × speed_scale`. `speed_scale` (camera) = 1
trên đường thẳng, về 0 khi hướng đường tâm ở đầu xa lệch ≥ 50° (`CURVE_HEADING_*_DEG`),
độ cong ≥ 1.5 1/m (`CURVE_K_*`), hoặc góc gập cách trục sau ≤ 0.7 m (`CORNER_SLOW_*`).

Bảng ESC firmware rất dốc: 100 ≈ 3.6, 101 ≈ 5.1, 102 ≈ 6.6, 104 ≈ 7.8, 105 ≈ 8.3 km/h.
Muốn nhanh hơn trên đường thẳng: `speed_x10:=75` (ESC 104). Cua đã ở sàn ESC 101: firmware ghi
ESC theo độ nguyên nên mọi lệnh < ~6.1 km/h đều ra ESC 101; chậm hơn nữa phải sửa firmware.

## 5. Mask vạch (giống `lane_bev.py`)

Trên ảnh BEV: nền = phép đóng 13 cm; pixel là vạch khi tối hơn nền ≥ `dark_ratio` (tự chỉnh
0.10–0.35 theo độ tương phản của chính vạch đang bám, hiện ở `thr`), ≥ `DARK_MIN_ABS` mức
xám, và tối hơn **mức sàn** xung quanh (phân vị 40% khối 40×20 cm) ≥ `FLOOR_DARK_RATIO`.

| Triệu chứng | Chỉnh |
|---|---|
| Vạch trong vùng tối / lóa bị mất | giảm `FLOOR_DARK_RATIO` (0.06) hoặc `DARK_RATIO_MIN` (0.08) |
| Khe gạch / vân sàn hiện thành vạch | tăng `FLOOR_DARK_RATIO` (0.14) hoặc `DARK_RATIO_MIN` (0.12) |
| Vạch bản rộng bị khoét rỗng giữa | tăng `BG_KERNEL_M` (0.16) |
| Vạch dính vào ghế / vật có màu bên cạnh | lọc màu: pixel bão hoà > `COLOR_MAX_SAT` (100) không phải vạch. Vạch đen bị mất ⇒ tăng (130) |

Đo trên ảnh thật (2 dải cao su đen trên bàn, 08/10): cao su S 40–70, ghế cam S 130–150,
mặt bàn S < 10. Góc cúi: xem §1b (`camera_pitch_deg`, tự hiệu chỉnh).

## 6. Hướng lái (firmware)

- `dev` luôn tính theo ảnh **tham chiếu 640 px** tại 0.65 m (~1.5 mm/px). `> 0` = lái phải.
- `CAM_DEADZONE = 4` px, bão hoà `CAM_MAX_DEV = 45` px → 30°.
- `STEER_KP = 1.0`: xe lắc qua lại ⇒ giảm (0.8); cua gắt không đủ lái thì giữ 1.0 và giảm
  `PP_LOOKAHEAD_M` phía camera.
- `ALPHA_STEER = 0.5`: lọc nhẹ `dev` trên ESP32 (Mini PC đã lọc chính).
- `STEER_KD = 0.03` s, giới hạn ±`STEER_D_MAX` = 6°: giảm chấn, không gây giật.
- `STEER_RATE_DEG_S = 500`: tốc độ quay tối đa của lệnh servo.
- `ESC_START_FWD = 101`: mức ga thấp nhất khi đang chạy (dưới mức này BLDC kêu).
- Xe đẩy sang phải mà bánh càng lái càng xa ⇒ chạy lại với `dev_sign:=-1`.
- Dashboard ước lượng góc servo bằng cùng công thức (`fusion_viz_node.cpp`, `FW_*`): đổi
  firmware thì đổi luôn ở đó.

## 7. Lái theo đường đã nhớ (`steer_mode:=track`, mặc định) — ĐO 3 SỐ NÀY TRƯỚC

| Tham số (launch) | Mặc định | Cách đo / khi nào chỉnh |
|---|---|---|
| `wheelbase_m` | 0.26 | thước: tâm trục trước → tâm trục sau |
| `cam_to_rear_axle_m` | 0.18 | đo trên xe ~18 cm. Thả dây dọi từ camera xuống sàn, đo dọc từ điểm đó tới tâm trục sau. **Ảnh hưởng lớn nhất**: thử vòng kín khai báo 0.12 mà thật 0.30 (hoặc ngược lại) ⇒ xe chạm vạch 30–50 % thời gian. (car_config.py ghi 0.12 — sai) |
| `steer_ratio` | 0.6 | quay servo hết lái (dashboard SERVO ≈ 60 hoặc 120): đo góc bánh lệch so với thẳng, chia cho 30. Chạy thật với 0.8 rồi 0.65: xe bắt về phía ngoài cua, cua nhẹ bẻ không đủ ⇒ hạ 0.6. Khai báo CAO hơn thật ⇒ lệch ra ngoài cua; THẤP hơn thật ⇒ cắt vào trong / lắc |
| `single_search_m` / `single_search_rate` | 0.08 / 0.15 | chỉ thấy 1 vạch: dời dần tâm bám về phía vạch bị mất (tối đa m, tốc độ m/s) ⇒ xe lái vào trong tìm lại vạch kia, ôm cua khi vạch trong ra khỏi khung (`SINGLE_LINE_SEARCH_PX` bản Python). Thấy lại 2 vạch thì trả dần về 0 |
| `xte_gain` | 2.0 | phản hồi lệch ngang tại chân camera: `atan(xte_gain·e / (v + 0.5))`. Pure pursuit thuần lệch 5 cm chỉ bẻ ~3°. Xe lắc qua lại trên đường thẳng ⇒ giảm (1.0); hay lệch ra ngoài cua ⇒ tăng (2.5) |
| `xte_ki` / `xte_i_max_deg` | 50 / 6 | tích phân lệch ngang (độ bánh / m / s): bù trim servo, camera lắp lệch/xoay vài độ (`STEER_KI` / `STEER_I_LIMIT` bản Python). `/lane/status`: `xte=` lệch (cm), `xi=` phần tích phân (độ) |
| `lookahead_min_m` / `lookahead_gain_s` / `lookahead_max_m` | 0.35 / 0.20 / 0.80 | tầm nhìn = min + gain·v. Xe lắc trên đường thẳng ⇒ tăng `min` (0.45); cắt cua ⇒ giảm `min` (0.30) |
| `actuator_latency_s` | 0.08 | xe vào cua trễ, văng ra ngoài ⇒ tăng (0.12) |
| `camera_latency_s` | 0.04 | tương tự, cho trễ camera |
| `car_half_width_m` / `line_margin_m` | 0.10 / 0.04 | rào chắn vạch: nửa bề ngang xe / khoảng hở tối thiểu |
| `odom_speed_scale` | 1.0 | tốc độ thật / tốc độ lệnh. Đo: cho chạy 2 m thẳng, bấm giờ |
| `lost_memory_ms` | 1500 | mất cả 2 vạch: chạy tiếp theo đường đã nhớ tối đa bao lâu |

Kiểm tra trên bản đồ nhỏ góc phải ảnh overlay:
- Đi thẳng: đường vàng nằm giữa 2 vạch xanh, kéo dài xuống tới khung xe (vùng mù được lấp).
- Vào cua: vạch trong chuyển màu **tím** (vạch ảo) khi ra khỏi khung hình, vẫn nằm đúng chỗ.
- Đường vàng / vạch bị "cong lệch" dần khi xe quay ⇒ `steer_ratio` hoặc `wheelbase_m` sai.
- `/lane/status`: `mode=track turn=R mem=0.85 la=0.63 wheel=+18.0 guard=+0.0`.
  `guard` ≠ 0 thường xuyên ⇒ xe hay sát vạch ⇒ kiểm tra `steer_ratio`, giảm tốc cua.

Muốn quay lại cách lái cũ để so sánh: `./run.sh steer_mode:=camera`.
