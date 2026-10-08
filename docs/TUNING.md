# HƯỚNG DẪN HIỆU CHỈNH NHANH

Mọi hằng số detector nằm trong `src/camera_node/camera_node.hpp`. Sửa ở đó
rồi build lại. Tham số nào tiện thì expose qua launch (`ros2 launch fusion_node
fusion.launch.py <tên>=<giá trị>` hoặc `./run.sh <tên>:=<giá trị>`).

## 0. Trước hết: nhìn `/lane/vis`

Ảnh overlay cho biết ngay detector đang dừng ở bước nào:

- Khung **xanh nhạt** = ROI (hình chữ nhật, phủ hết bề ngang ảnh)
- **Vạch trái xanh lá**, **vạch phải xanh dương**, kèm các ô vuông xám dọc theo vạch
- Vạch **xám mảnh** = vạch ứng viên không được chọn (vạch làn bên cạnh, vết bẩn...)
- **Đường vàng** = đường tâm làn (vạch dời vào trong nửa bề rộng làn)
- **Vòng tròn vàng** = điểm ngắm pure pursuit (cách xe 0.70 m), nối với mũi xe
- **Vạch tím** = chân trời
- Chữ: `2 LANES` / `1 LANE (L|R)` / `LOST`, dấu `*` = frame bị chặn nhảy
- Thanh trên: `dev` (px ảnh 640 + cm tương đương), `w` bề rộng làn đã học, `v` hệ số tốc
  độ, `k` độ cong (1/m), `ctr` độ tương phản mask, `lines` số vạch ứng viên

## 1. Bám vạch

| Hằng số | Mặc định | Khi nào chỉnh |
|---|---|---|
| `TAPE_MAX_M` | 0.12 | bề dày tối đa 1 vạch (m); băng keo bản rộng bị loại thì tăng |
| `MIN_LINE_LEN_M` | 0.15 | vạch (sau khi nối) ngắn hơn thì bỏ; vết bẩn ngắn bị nhận là vạch thì tăng |
| `MIN_SEG_LEN_M` | 0.06 | đoạn vạch ngắn hơn thì bỏ trước khi nối |
| `LINK_GAP_M` / `LINK_ANGLE_DEG` | 0.35 / 40 | nối đoạn đứt do lóa: khoảng hở / lệch hướng tối đa |
| `LANE_W_MIN_M` / `LANE_W_MAX_M` | 0.35 / 0.85 | bề rộng làn chấp nhận được |
| `PRIOR_MATCH_M` | 0.12 | vạch cách vạch frame trước ít hơn mức này thì giữ nhãn trái/phải cũ |
| `MIN_BLOB_PX` | 10 | thành phần liên thông nhỏ hơn thì coi là nhiễu |

## 1b. Chân trời — chỉnh ĐẦU TIÊN

`horizon_frac` (launch, mặc định `0.20`) là hàng chân trời theo tỉ lệ chiều cao ảnh. Mọi
phép đổi pixel ↔ mét (bề rộng làn, bề dày vạch, toạ độ mặt đất) dựa vào nó; sai ⇒ bề rộng
làn sai ⇒ hay rớt về `1 LANE`, đường tâm lệch.

Cách chỉnh: đặt xe giữa 2 vạch **thẳng**, nhìn ảnh overlay, kéo dài 2 vạch lên trên — chỗ
chúng gặp nhau là chân trời; vạch tím phải nằm ngang qua đúng chỗ đó. Vạch tím thấp hơn
điểm gặp ⇒ giảm `horizon_frac`, cao hơn ⇒ tăng. Ví dụ: `./run.sh horizon_frac:=0.18`.

Sau đó kiểm tra ô LANE WIDTH trên dashboard ra gần bề rộng làn thật; lệch nhiều thì chỉnh
`camera_height_m` (cm đo được tỉ lệ thuận với nó).

## 2. Vùng làm việc

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `roi_top_frac` (launch) | 0.48 | đỉnh ROI; nhỏ hơn = thấy xa hơn, báo cua sớm hơn |
| `roi_bottom_frac` (`CameraProfile`) | 0.95 | đáy ROI; nếu thấy cản/mũi xe trong ảnh thì giảm |
| `ROI_MIN_DY` | 10 | đỉnh ROI không cao hơn chân trời + 10 hàng |

ROI luôn phủ **hết bề ngang** ảnh. Log khởi động in `geometry: ... ROI rows a..b (full
width) = Zgần..Zxa m` để biết vùng nhìn thật trên mặt đất.

## 3. Lái theo `dev_px` của camera (chỉ dùng khi `steer_mode:=camera`)

| Hằng số | Mặc định | Ý nghĩa |
|---|---|---|
| `PURSUIT_L_M` | 0.70 | khoảng nhìn trước; nhỏ hơn = bám sát hơn nhưng dễ lắc, lớn hơn = mượt nhưng cắt cua |
| `LAT_GAIN` | 0.4 | kéo về giữa làn theo độ lệch ngang ngay trước xe. Xe lệch mà về giữa chậm ⇒ tăng (0.6); lắc trên đường thẳng ⇒ giảm (0.2) |
| `DEV_REF_DIST_M` | 0.65 | thang đo `dev_px` (đừng đổi trừ khi chỉnh lại cả firmware) |
| `JUMP_GATE_REF_PX` | 90 | `dev` nhảy quá mức này (px ảnh 640) thì giữ giá trị cũ 1 frame |
| `EMA_ALPHA_TWO` / `EMA_ALPHA_ONE` | 0.70 / 0.50 | lọc `dev` khi 2 vạch / 1 vạch; lớn = nhạy |
| `HOLD_MS` | 500 | mất vạch: giữ `dev` cuối bao lâu |

## 4. Tốc độ

| Tham số (launch) | Mặc định | Ý nghĩa |
|---|---|---|
| `speed_x10` | 75 | đủ 2 vạch + đường thẳng (7.5 km/h ≈ ESC 104) |
| `speed_corner_x10` | 55 | 1 vạch, hoặc 2 vạch ở cua gắt nhất (5.5 km/h ≈ ESC 101) |
| `speed_hold_x10` | 50 | mất cả 2 vạch, đang giữ hướng |
| `speed_min_x10` | 50 | sàn tốc độ khi đang chạy — BLDC dưới mức này kêu cọt kẹt |
| `speed_ramp_x10` | 30 | tăng tốc 3 km/h mỗi giây; giảm tốc luôn tức thì |

Đủ 2 vạch: `tốc độ = corner + (speed − corner) × speed_scale`. `speed_scale` (camera) = 1
trên đường thẳng, về 0 khi hướng đường tâm ở đầu xa lệch ≥ 50° (`CURVE_HEADING_*_DEG`)
hoặc độ cong ≥ 1.5 1/m (`CURVE_K_*`).

Bảng ESC firmware rất dốc: 100 ≈ 3.6, 101 ≈ 5.1, 102 ≈ 6.6, 104 ≈ 7.8, 105 ≈ 8.3 km/h.
Muốn nhanh hơn trên đường thẳng: `speed_x10:=85`; cua không kịp xử lý: `speed_corner_x10:=50`.

## 5. Ngưỡng ảnh (trừ nền + bù lóa + hysteresis)

Detector ước lượng ảnh nền (phép đóng 45×25), lấy `diff = (nền − ảnh) × gain(nền)`, Otsu
trên `diff`, rồi giữ thêm pixel yếu nối liền với pixel mạnh. Nhìn panel **BINARY MASK**:

| Triệu chứng | Chỉnh |
|---|---|
| Vạch sát xe (bản rộng) bị khoét rỗng giữa | tăng `BG_KERNEL_W` (55) |
| Vạch nằm ngang ở cua gắt bị khoét rỗng | tăng `BG_KERNEL_H` (31) |
| Vạch trong vùng lóa vẫn đứt | tăng `GLARE_GAIN_MAX` (3.0) hoặc giảm `BG_WEAK_DIFF` (5) |
| Vùng lóa sinh vệt giả | giảm `GLARE_GAIN_MAX` (1.8) |
| Vân sàn / mép bàn hiện thành vạch | tăng `BG_MIN_DIFF` (16) hoặc `BG_WEAK_DIFF` (10) |
| Mask trống dù thấy vạch | giảm `MIN_CONTRAST` (8) |
| Vạch dính vào ghế / vật có màu bên cạnh | lọc màu: pixel bão hoà > `COLOR_MAX_SAT` (100) không phải vạch. Vạch đen bị mất ⇒ tăng (130) |
| Vạch dính vào khối tối lớn (chân máy, bóng, mép bàn) | `THICK_REMOVE = true`: khoét vùng có lõi dày hơn `TAPE_MAX_M`. Vạch bản rộng bị khoét ⇒ tăng `TAPE_MAX_M` |

Đo trên ảnh thật (2 dải cao su đen trên bàn, 08/10): cao su S 40–70, ghế cam S 130–150,
mặt bàn S < 10. Chân trời đo từ 2 dải ≈ `0.155` (2 dải kéo dài gặp nhau ở 15.5 % chiều cao
ảnh) — chạy `./run.sh horizon_frac:=0.155` rồi kiểm tra ô LANE WIDTH khớp bề rộng thật.

## 6. Hướng lái (firmware)

- `dev` luôn tính theo ảnh **tham chiếu 640 px** tại 0.65 m (~1.5 mm/px). `> 0` = lái phải.
- `CAM_DEADZONE = 4` px, bão hoà `CAM_MAX_DEV = 45` px → 30°.
- `STEER_KP = 1.0`: xe lắc qua lại ⇒ giảm (0.8); cua gắt không đủ lái thì giữ 1.0 và giảm
  `PURSUIT_L_M` phía camera.
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
| `cam_to_rear_axle_m` | 0.30 | thả dây dọi từ camera xuống sàn, đo ngang từ điểm đó tới tâm trục sau |
| `steer_ratio` | 1.0 | quay servo hết lái (dashboard SERVO ≈ 60 hoặc 120): đo góc bánh lệch so với thẳng, chia cho 30. Sai 20 % là đủ để chạm vạch ở cua gắt |
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
