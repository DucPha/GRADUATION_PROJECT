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
| `camera_pitch_deg` | 35.5 | góc cúi camera, CỐ ĐỊNH. Đo: chụp 2 dải thẳng song song, kéo dài gặp nhau ở hàng v ⇒ pitch = atan((H/2 − v)/f), f = (H/2)/tan 25.5°. 08/10 ~42°; tối 09/10 camera ngửa lên: 35.5° (2 dải cách 50 cm trên bàn đo ra 49.5 cm) |
| `camera_height_m` | 0.30 | tâm ống kính cách sàn |
| `camera_vfov_deg` | 51.0 | góc nhìn **dọc**. Chế độ 1920×1080 và 640×480 của cam này cùng góc dọc (4:3 chỉ cắt 2 bên) |
| (bỏ `lane_width_m`) | — | bề rộng làn không cài đặt: đo mỗi frame thấy 2 vạch (0.28–0.70 m), đường tâm = giữa 2 vạch đo được; chỉ 1 vạch thì dời nửa bề rộng vừa đo gần nhất |

Kiểm tra: đặt xe giữa 2 vạch thẳng, overlay phải hiện `2 LANES` và `w` ≈ bề rộng làn thật.
Camera cúi 35.5° nên **không thấy chân trời** trong ảnh; mặt sàn nhìn thấy từ ~0.14 m tới
~1.6 m trước chân camera (ô BEV góc trên trái overlay).

## 2. Vùng làm việc

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `roi_top_frac` (launch) | 0.0 | bỏ qua phần trên cùng của ảnh (mặc định dùng hết, ~1.1 m) |
| `roi_bottom_frac` (`CameraProfile`) | 0.98 | đáy ROI; nếu thấy cản/mũi xe trong ảnh thì giảm |

Log khởi động in `geometry: ... BEV 160xN (1 cm), Z gần..xa m` = vùng mặt sàn nhìn thấy.

## 3. Lái theo `dev_px` của camera (chỉ khi PathTracker chưa có / mất đường nhớ)

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
| `speed_x10` | 65 | đủ 2 vạch + đường thẳng (lệnh 6.5 km/h ≈ 1562 µs) |
| `speed_one_x10` | 60 | 1 vạch, đường thẳng, xe giữa làn (≈ 1559 µs) |
| `speed_corner_x10` | 58 | cua gắt (cả khi đang ở chế độ bám vạch ngoài), 1 vạch, hoặc 2 vạch cong gắt nhất (≈ 1557.5 µs; 52 vẫn kêu két) |
| `coast_decel_mps2` / `corner_margin_m` | 0.45 / 0.20 | giảm tốc trước cua theo khoảng cách tới cua (ESC chỉ nhả ga, xe trôi). Đường thẳng dài rồi vẫn vọt ra ngoài cua ⇒ giảm `coast_decel_mps2` (0.5) |
| `speed_hold_x10` | 58 | mất cả 2 vạch, đang giữ hướng |
| `speed_min_x10` | 58 | sàn tốc độ lệnh khi đang chạy — BLDC dưới mức này kêu cọt kẹt |
| `speed_ramp_x10` / `speed_down_x10` | 30 / 20 | tăng tốc khi ra cua (58 → 65 ~0.25 s) / giảm đều khi vào cua (~0.35 s), x10 mỗi giây |

Đủ 2 vạch: `tốc độ = corner + (speed − corner) × speed_scale`. `speed_scale` (camera) = 1
trên đường thẳng, về 0 khi hướng đường tâm ở đầu xa lệch ≥ 50° (`CURVE_HEADING_*_DEG`),
độ cong ≥ 1.5 1/m (`CURVE_K_*`), hoặc góc gập cách trục sau ≤ 0.7 m (`CORNER_SLOW_*`).

Firmware ghi ESC theo µs: `1543 + (v − 3.58) / 2.98 × 19.5` (v = km/h lệnh), sàn 1550 µs,
cộng tới +3 µs khi hết lái. Mỗi đơn vị x10 ≈ 0.65 µs.

**Tốc độ thật** (đo từ ảnh ghi 09/10): lệnh 4.8 km/h ⇒ ~0.42 m/s (×0.32) và chậm dần khi
pin yếu. Ở 0.42 m/s xe đi 1.7 cm mỗi frame (25 fps) ⇒ xử lý ảnh không bị ảnh hưởng; mô
phỏng cũng cho thấy giảm tốc đường thẳng 62 → 55 không đổi gì việc bám làn (giới hạn là góc
lái). BLDC kêu ⇒ tăng `speed_min_x10` / `speed_corner_x10` (54, 56…), đừng giảm.

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
- `CAM_DEADZONE = 4` px, độ dốc 30° / 41 px, bão hoà `CAM_MAX_DEV ≈ 52` px → 35° (servo 55..125).
- `STEER_KP = 1.0`: xe lắc qua lại ⇒ giảm (0.8); cua gắt không đủ lái thì giữ 1.0 và giảm
  `PP_LOOKAHEAD_M` phía camera.
- `ALPHA_STEER = 0.5`: lọc nhẹ `dev` trên ESP32 (Mini PC đã lọc chính).
- `STEER_KD = 0.03` s, giới hạn ±`STEER_D_MAX` = 6°: giảm chấn, không gây giật.
- `STEER_RATE_DEG_S = 500`: tốc độ quay tối đa của lệnh servo.
- `ESC_START_FWD = 101`: mức ga thấp nhất khi đang chạy (dưới mức này BLDC kêu).
- Xe đẩy sang phải mà bánh càng lái càng xa ⇒ chạy lại với `dev_sign:=-1`.
- Dashboard ước lượng góc servo bằng cùng công thức (`fusion_viz_node.cpp`, `FW_*`): đổi
  firmware thì đổi luôn ở đó.

## 7. Lái theo đường đã nhớ (PathTracker) — ĐO 3 SỐ NÀY TRƯỚC

| Tham số (launch) | Mặc định | Cách đo / khi nào chỉnh |
|---|---|---|
| `wheelbase_m` | 0.26 | thước: tâm trục trước → tâm trục sau |
| `cam_to_rear_axle_m` | 0.18 | đo trên xe ~18 cm. Thả dây dọi từ camera xuống sàn, đo dọc từ điểm đó tới tâm trục sau. **Ảnh hưởng lớn nhất**: thử vòng kín khai báo 0.12 mà thật 0.30 (hoặc ngược lại) ⇒ xe chạm vạch 30–50 % thời gian. (car_config.py ghi 0.12 — sai) |
| `steer_ratio` | 0.6 | góc bánh / góc servo khai báo. Đo từ ảnh 09/10: thật ~0.42 (servo hết lái 30° ⇒ xe quay 21°/s ở 0.42 m/s ⇒ bánh ~12.7°). GIỮ 0.6: thử vòng kín, đặt đúng 0.42 (kèm `odom_speed_scale` 0.32) lại tệ hơn vì odometry ước lượng thừa đang vô tình giúp bẻ lái sớm |
| `single_search_m` / `single_search_rate` | 0.08 / 0.30 | chỉ thấy 1 vạch: dời dần tâm bám về phía vạch bị mất (tối đa m, tốc độ m/s) ⇒ xe lái vào trong tìm lại vạch kia, ôm cua khi vạch trong ra khỏi khung (`SINGLE_LINE_SEARCH_PX` bản Python). Thấy lại 2 vạch thì trả dần về 0 |
| `xte_gain` | 2.0 | phản hồi lệch ngang tại chân camera: `atan(xte_gain·e / (v + 0.5))`. Pure pursuit thuần lệch 5 cm chỉ bẻ ~3°. Xe lắc qua lại trên đường thẳng ⇒ giảm (1.0); hay lệch ra ngoài cua ⇒ tăng (2.5) |
| `xte_ki` / `xte_i_max_deg` | 50 / 6 | tích phân lệch ngang (độ bánh / m / s): bù trim servo, camera lắp lệch/xoay vài độ (`STEER_KI` / `STEER_I_LIMIT` bản Python). `/lane/status`: `xte=` lệch (cm), `xi=` phần tích phân (độ) |
| `lookahead_min_m` / `lookahead_gain_s` / `lookahead_max_m` | 0.35 / 0.20 / 0.80 | tầm nhìn = min + gain·v. Xe lắc trên đường thẳng ⇒ tăng `min` (0.45); cắt cua ⇒ giảm `min` (0.30) |
| `actuator_latency_s` | 0.08 | xe vào cua trễ, văng ra ngoài ⇒ tăng (0.12) |
| `camera_latency_s` | 0.04 | tương tự, cho trễ camera |
| `car_half_width_m` / `tape_half_m` / `line_margin_m` | 0.125 / 0.035 / 0.02 | rào chắn vạch (bánh trước và 0.25 m trước đó): nửa bề ngang xe / nửa bề rộng băng keo / khoảng hở tối thiểu tới MÉP băng keo |
| `corner_gain` | 1.15 | trong cua nhân góc lái pure pursuit (1 = tắt) |
| `lookahead_corner_scale` | 0.8 | trong cua (độ cong phía trước 0.4 → 1.0 1/m) nhân khoảng nhìn trước với hệ số này: điểm ngắm ~0.51 m thay vì ~0.64 m, không rơi lên đoạn thẳng SAU cua ⇒ bớt cắt góc đè vạch trong. 0.6 đã thử trên bản ghi: điểm ngắm sát đầu đường, lái giật ±21°. `1` = tắt |
| `kink_radius_m` | 1.0 | đường đua là các đoạn thẳng nối GÓC GẤP. Pure pursuit ngắm ~0.65 m nên thấy đỉnh góc là bẻ ngay ⇒ vào sớm, cắt góc. Ở đây chỉ bẻ khi trục sau còn cách đỉnh R·tan(góc/2), trước đó đi thẳng theo đoạn trước góc (chỉ khi thân xe song song đoạn đó, lệch < 12°; đang quay dở qua góc trước thì không giữ). Vẫn vào sớm / đè vạch trong ⇒ giảm (0.8); vọt ra vạch ngoài ⇒ tăng (1.3); `0` = tắt |
| `corner_outer_gap_m` / `corner_outer_max_m` | 0.08 / 0.15 | CHỈ Ở CUA GẮT: bám theo vạch NGOÀI, giữ mép xe cách mép băng keo vạch ngoài đúng `gap`. Khoảng cách đo TRỰC TIẾP từ điểm ngắm tới vạch ngoài (trong cua thường không thấy vạch trong nên bề rộng làn chỉ là ước lượng); quá sát thì dời vào trong (tối đa 5 cm). Chạm vạch ngoài ⇒ tăng (0.10–0.12); vẫn đè vạch trong ⇒ giảm (0.05); `-1` = tắt. `/lane/status`: `cob=` (cm, > 0 = sang phải), `sharp=L/R/-`, `od=` (cm, điểm ngắm → vạch ngoài) |
| `corner_sharp_k` | 1.0 | điều kiện vào chế độ cua gắt: độ cong phía trước ≥ mức này VÀ bánh đã bẻ ≥ 8° cùng chiều trong 0.15 s; giữ tới khi bánh về < 6° (0.2 s). Không giữ theo độ cong vì giữa cua gấp khúc đoạn thẳng kế tiếp nằm chéo trước mặt ⇒ độ cong ~0 dù bánh hết lái. Cua lớn không bật ⇒ giảm (0.8) |
| `odom_speed_scale` | 1.0 | tốc độ thật / tốc độ lệnh khai báo. Đo thật ~0.32 nhưng giữ 1.0 (xem `steer_ratio`) |
| `lost_memory_ms` | 1500 | mất cả 2 vạch: chạy tiếp theo đường đã nhớ tối đa bao lâu |

Kiểm tra trên bản đồ nhỏ góc phải ảnh overlay:
- Đi thẳng: đường vàng nằm giữa 2 vạch xanh, kéo dài xuống tới khung xe (vùng mù được lấp).
- Vào cua: vạch trong chuyển màu **tím** (vạch ảo) khi ra khỏi khung hình, vẫn nằm đúng chỗ.
- Đường vàng / vạch bị "cong lệch" dần khi xe quay ⇒ `steer_ratio` hoặc `wheelbase_m` sai.
- `/lane/status`: `mode=track turn=R mem=0.85 la=0.63 wheel=+18.0 guard=+0.0`.
  `guard` ≠ 0 thường xuyên ⇒ xe hay sát vạch ⇒ kiểm tra `steer_ratio`, giảm tốc cua.

