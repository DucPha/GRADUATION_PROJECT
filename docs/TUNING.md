# HƯỚNG DẪN HIỆU CHỈNH NHANH

Mọi hằng số detector nằm trong `src/camera_node/lane_detector.hpp`. Sửa ở đó
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

## 1. Bám vạch (`lane_detector.hpp`)

| Hằng số | Mặc định | Khi nào chỉnh |
|---|---|---|
| `TAPE_MAX_THICK_M` / `BLOB_THICK_M` | 0.10 / 0.18 | phần dày hơn mà không thuôn dài / dày hơn hẳn = đồ vật. Băng keo bản rộng bị tô tím ⇒ tăng |
| `TRACK_GATE_M` | 0.07 | vạch mới lệch xa vị trí dự đoán hơn mức này ⇒ không phải vạch cũ |
| `COAST_SEC` | 0.5 | mất vạch ngắn hơn: giữ vạch dự đoán theo odometry |
| `MIN_KINK_TAIL_M` | 0.10 | đoạn sau góc gập ngắn hơn ⇒ không phải góc cua (rẽ vào vết bẩn) |
| `CORNER_KINK_DEG` | 30 | tâm làn gập ≥ bấy nhiêu độ thì coi là góc cua |
| `RADIAL_DEG` / `RADIAL_NEAR_GAP_M` | 8 / 0.12 | lọc chân bàn/ghế (vệt chĩa thẳng về camera, không chạm mép dưới tầm nhìn) |
| `RADIAL_LINE_DEG` / `RADIAL_LINE_MIN_M` / `RADIAL_KINK_DEG` | 12 / 0.12 / 30 | lọc chân bàn/ghế trên từng VẠCH: đoạn đầu vạch chĩa về camera (chân ghế dính bóng thành chữ V) ⇒ bỏ vạch; đoạn sau góc gập ≥ 30° chĩa về camera (đi dọc băng keo rồi rẽ vào chân ghế đứng sát) ⇒ cắt bỏ đoạn đó |
| `ZIGZAG_DEG` / `ZIGZAG_GAP_M` | 35 / 0.12 | 2 góc gập ngược chiều sát nhau (răng cưa: mép bóng, rẽ vào hộp giấy) ⇒ cắt vạch tại đó |
| `LINE_MIN_CONTRAST_FRAC` | 0.75 | độ tối trung vị dọc vạch phải ≥ 0.75 × độ tối của chính vạch đó ở frame trước (vạch mới: so với vạch đang bám nhạt hơn) ⇒ không bám sang vệt bóng. Vạch thật bị bỏ khi đi qua vùng tối ⇒ giảm (0.6) |
| `LANE_W_MIN_M` / `LANE_W_MAX_M` | 0.50 / 0.85 | khoảng bề rộng làn chấp nhận khi ghép cặp — **tuyệt đối**, không theo bề rộng đã học (làn dán tay rộng hẹp từng đoạn) |
| `LANE_W_LOCAL_ALPHA` / `LANE_W_LOCAL_HOLD_SEC` | 0.35 / 3 | bề rộng làn **tại chỗ** (đo mỗi frame thấy 2 vạch); chỉ thấy 1 vạch thì dời tâm nửa bề rộng này |

Vạch trái / phải không bao giờ được trùng hay đổi chỗ cho nhau: vạch bám lại được phải nằm
đúng phía vạch kia; mâu thuẫn thì giữ vạch đã bám lâu hơn (trước đây một vệt bóng nhận làm
vạch trái có thể "bám" sang băng keo phải ⇒ vạch phải thật bị bỏ, xe lái ra ngoài).

## 1b. Hình học camera — chỉnh ĐẦU TIÊN

Detector chiếu ảnh xuống **mặt sàn** (BEV, 1 cm/ô, `src/camera_node/lane_detector.cpp`) nên mọi
phép đổi pixel ↔ mét dựa vào 3 số trong launch:

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `camera_pitch_deg` | 42.0 | góc cúi camera. Đo trên ảnh thật (log Python 08/10): 2 vạch thẳng kéo dài gặp nhau ở hàng −106 của ảnh 320×240 ⇒ ~42°, bề rộng làn đo lại ra đúng 42 cm |
| `camera_height_m` | 0.30 | tâm ống kính cách sàn |
| `camera_vfov_deg` | 51.0 | góc nhìn **dọc**. Chế độ 1920×1080 và 640×480 của cam này cùng góc dọc (4:3 chỉ cắt 2 bên) |
| `camera_auto_pitch` | false | tự hiệu chỉnh góc cúi khi chạy thẳng thấy 2 vạch (±8° quanh `camera_pitch_deg`). Mặc định TẮT: thử vòng kín, ước lượng nhảy 42 → 44.5 → 42.7° dù góc thật cố định, mỗi lần đổi > 2° còn xoá vạch đang bám. Bật để đo 1 lần: log in `Tu hieu chinh goc cui camera: ... -> X` ⇒ điền X vào `camera_pitch_deg` rồi tắt lại |
| `lane_width_m` | 0.60 | bề rộng làn **ban đầu** (tâm vạch – tâm vạch). Không cố định: mỗi frame thấy 2 vạch đo lại bề rộng tại chỗ, ghép cặp chấp nhận 0.50–0.85 m |

Kiểm tra: đặt xe giữa 2 vạch thẳng, overlay phải hiện `2 LANES` và `w` ≈ bề rộng làn thật.
Camera cúi ~42° nên **không thấy chân trời** trong ảnh; mặt sàn nhìn thấy từ ~0.13 m tới
~1.1 m trước chân camera (ô BEV góc trên trái overlay).

## 2. Vùng làm việc

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `WORK_W` (`lane_detector.hpp`) | 960 | bề ngang **khung làm việc** (= bề ngang ảnh giải mã 1920/2 nên **không thu nhỏ thêm**, mép vạch sắc hơn). Giảm về 640 / 320 nếu Mini PC yếu — hình học scale-invariant, **không đổi** kết quả đo mét |
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
| `speed_x10` | 70 | MỨC THẲNG: đường thẳng hẳn + xe giữa làn ⇒ 1562.5 µs. CHỈ 2 MỨC (15/10): còn lại (cua, sắp tới cua, xe lệch) là `speed_corner_x10` 44 ⇒ 1548 µs, không nội suy. 15/10: 62 → 70 (MPC có tích phân LQI giữ giữa làn tốt hơn). Muốn nhanh hơn: 78–90 |
| `speed_one_x10` | 70 | chỉ thấy 1 vạch, đường thẳng, xe giữa làn (= mức thẳng 1562.5 µs) |
| `speed_corner_x10` | 44 | 1 vạch, hoặc 2 vạch ở cua gắt nhất (≈ ESC 101) |
| `speed_hold_x10` | 44 | mất cả 2 vạch, đang giữ hướng |
| `speed_min_x10` | 44 | sàn tốc độ khi đang chạy — BLDC dưới mức này kêu cọt kẹt |
| `speed_ramp_x10` | 8 | tăng tốc 0.8 km/h mỗi giây; giảm tốc luôn tức thì |
| `coast_decel_mps2` | 0.7 | xe nhả ga (ESC không phanh) giảm tốc cỡ bấy nhiêu m/s². Dùng để **giảm tốc trước cua** và cho odometry khi giảm tốc. Vẫn vọt cua ⇒ giảm (0.5) |
| `corner_margin_m` | 0.20 | phải xuống tốc độ cua trước khi tới cua bấy nhiêu m |

`tốc độ = corner + (speed − corner) × hệ số` (`speed` = `speed_x10` khi 2 vạch, `speed_one_x10`
khi 1 vạch). Hệ số = nhỏ nhất của: `speed_scale` camera, độ cong đường đã nhớ phía trước, và
**xe giữa làn** (lệch ≤ 3 cm ⇒ 1, ≥ 7 cm ⇒ 0: xe 25 cm trong làn 0.60 m còn ~14 cm mỗi bên).
`speed_scale` (camera) = 1
trên đường thẳng, về 0 khi hướng đường tâm ở đầu xa lệch ≥ 50° (`CURVE_HEADING_*_DEG`),
độ cong ≥ 1.5 1/m (`CURVE_K_*`), hoặc góc gập cách trục sau ≤ 0.7 m (`CORNER_SLOW_*`).

**Giảm tốc trước cua**: PathTracker tìm điểm đầu tiên dọc đường tâm mà hướng đường đã quay
≥ 15° so với hướng tại bánh trước (`corner_dist_m`, `corner=` trong `/lane/status`; −1 = chưa
thấy cua). Tốc độ bị chặn thêm `v = √(v_cua² + 2·coast_decel_mps2·(d − corner_margin_m))`:
thấy cua là nhả ga ngay, không đợi độ cong trung bình (cả đoạn 1.2 m) lên cao như trước. Camera
chỉ thấy tới ~1 m trước bánh trước nên cua R 0.8 được phát hiện khi còn cách ~0.5–0.8 m; ESC chỉ
nhả ga (không phanh) nên tốc độ đường thẳng vẫn là giới hạn chính. Odometry dùng tốc độ ước
lượng (`vest=`): lệnh giảm thì xe trôi chậm dần theo `coast_decel_mps2` chứ không tụt ngay.

Bảng ESC firmware rất dốc: 100 ≈ 3.6, 101 ≈ 5.1, 102 ≈ 6.6, 104 ≈ 7.8, 105 ≈ 8.3 km/h.
Muốn nhanh hơn nữa trên đường thẳng: `speed_x10:=90` (ESC 106) — chưa thử. Cua đã ở sàn ESC 101: firmware ghi
ESC theo độ nguyên nên mọi lệnh < ~6.1 km/h đều ra ESC 101; chậm hơn nữa phải sửa firmware.

## 5. Mask vạch (giống `lane_bev.py`)

Trên ảnh BEV: nền = phép đóng 13 cm; pixel là vạch khi tối hơn nền ≥ `dark_ratio` (tự chỉnh
0.10–0.42 = 0.6 × độ tương phản của chính vạch đang bám, hiện ở `thr`; log thật: băng keo tối
hơn nền ~0.66, bóng chân bàn/ghế chỉ 0.25–0.40 ⇒ `thr` ≈ 0.40 loại bóng), ≥ `DARK_MIN_ABS` mức
xám, và tối hơn **mức sàn** xung quanh (phân vị 40% khối 40×20 cm) ≥ `FLOOR_DARK_RATIO`.

| Triệu chứng | Chỉnh |
|---|---|
| Vạch trong vùng tối / lóa bị mất | giảm `FLOOR_DARK_RATIO` (0.06) hoặc `DARK_RATIO_MIN` (0.08) |
| Bóng chân bàn/ghế vẫn hiện thành vạch | tăng `CONTRAST_FRAC` (0.7) / `CONTRAST_THR_MAX` (0.48) |
| Vạch xa / vạch nhạt bị đứt sau khi bám được vài giây | giảm `CONTRAST_FRAC` (0.5) |
| Khe gạch / vân sàn hiện thành vạch | tăng `FLOOR_DARK_RATIO` (0.14) hoặc `DARK_RATIO_MIN` (0.12) |
| Vạch bản rộng bị khoét rỗng giữa | tăng `BG_KERNEL_M` (0.16) |
| Vạch dính vào ghế / vật có màu bên cạnh | lọc màu: pixel bão hoà > `COLOR_MAX_SAT` (100) không phải vạch. Vạch đen bị mất ⇒ tăng (130) |

Đo trên ảnh thật (2 dải cao su đen trên bàn, 08/10): cao su S 40–70, ghế cam S 130–150,
mặt bàn S < 10. Góc cúi: xem §1b (`camera_pitch_deg`, tự hiệu chỉnh).

## 6. Hướng lái (firmware)

- `dev` luôn tính theo ảnh **tham chiếu 640 px** tại 0.65 m (~1.5 mm/px). `> 0` = lái phải.
- Servo 55..125 (±35°, 09/10; trước ±30°). `CAM_DEADZONE = 4` px, bão hoà
  `CAM_MAX_DEV` ≈ 52 px → 35°. Độ dốc giữ 30° / 41 px như bản cũ ⇒ Mini PC mới chạy được
  với firmware cũ (chỉ bão hoà ở 30°). Servo gồng / kêu rè khi đánh hết lái = chạm cữ cơ
  khí ⇒ giảm `STEER_MIN`/`STEER_MAX` (57/123); cữ còn rộng thì có thể nới 50/130.
  Đổi phạm vi phải sửa cả `FW_STEER_RANGE` trong `fusion_viz_node.cpp`.
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
| `steer_ratio` | 0.6 | quay servo hết lái (dashboard SERVO ≈ 55 hoặc 125): đo góc bánh lệch so với thẳng, chia cho 35. Chạy thật với 0.8 rồi 0.65: xe bắt về phía ngoài cua, cua nhẹ bẻ không đủ ⇒ hạ 0.6. Khai báo CAO hơn thật ⇒ lệch ra ngoài cua; THẤP hơn thật ⇒ cắt vào trong / lắc |
| `single_search_m` / `single_search_rate` / `single_search_view_m` | 0.08 / 0.30 / 0.25 | chỉ thấy 1 vạch trên **đường thẳng** mà vạch kia (dự đoán = vạch đang thấy + bề rộng làn) **nằm ngoài khung** (xe lệch về phía vạch đang thấy): dời dần tâm bám về phía vạch bị mất (tối đa m, tốc độ m/s) ⇒ xe lái mạnh về phía đó tìm lại vạch kia (`SINGLE_LINE_SEARCH_PX` bản Python). Vạch kia lẽ ra nằm trong khung ≥ `single_search_view_m` mà không thấy = vạch mờ / lóa ⇒ KHÔNG dời (thử vòng kín: dời 8 cm ở đây kéo xe lệch 10 cm). Thấy lại 2 vạch thì trả dần về 0. Trong cua (độ cong phía trước 0.3 → 0.7 1/m) tắt dần: vạch trong ra khỏi khung là do góc nhìn hẹp, tâm suy từ vạch ngoài + bề rộng làn đã đúng; dời thêm 8 cm làm bánh sau cắt lên vạch trong (xe 25 cm, mỗi bên chỉ còn ~5 cm) |
| `corner_gain` | 1.15 | trong cua (độ cong phía trước 0.4 → 1.0 1/m) nhân góc lái pure pursuit với hệ số này. Vẫn văng ra ngoài cua ⇒ tăng (1.25); cắt vào trong ⇒ giảm (1.0 = tắt) |
| `xte_gain` | 2.0 | phản hồi lệch ngang tại chân camera: `atan(xte_gain·e / (v + 0.5))`. Pure pursuit thuần lệch 5 cm chỉ bẻ ~3°. Xe lắc qua lại trên đường thẳng ⇒ giảm (1.0); hay lệch ra ngoài cua ⇒ tăng (2.5) |
| `xte_ki` / `xte_i_max_deg` | 50 / 6 | tích phân lệch ngang (độ bánh / m / s): bù trim servo, camera lắp lệch/xoay vài độ (`STEER_KI` / `STEER_I_LIMIT` bản Python). `/lane/status`: `xte=` lệch (cm), `xi=` phần tích phân (độ) |
| `lookahead_min_m` / `lookahead_gain_s` / `lookahead_max_m` | 0.35 / 0.20 / 0.80 | tầm nhìn = min + gain·v. Xe lắc trên đường thẳng ⇒ tăng `min` (0.45); cắt cua ⇒ giảm `min` (0.30) |
| `actuator_latency_s` | 0.08 | xe vào cua trễ, văng ra ngoài ⇒ tăng (0.12) |
| `camera_latency_s` | 0.04 | tương tự, cho trễ camera |
| `car_half_width_m` / `tape_half_m` / `line_margin_m` | 0.125 / 0.035 / 0.02 | rào chắn vạch: nửa bề ngang xe (xe rộng 25 cm) / nửa bề rộng băng keo / khoảng hở tối thiểu tới MÉP băng keo. Xe ở giữa làn 0.60 m còn ~14 cm mỗi bên nên rào chắn bắt đầu đẩy khi xe lệch ~12 cm |
| `odom_speed_scale` | 1.0 | tốc độ thật / tốc độ lệnh. Đo: cho chạy 2 m thẳng, bấm giờ |
| `lost_memory_ms` | 1500 | mất cả 2 vạch: chạy tiếp theo đường đã nhớ tối đa bao lâu |

Kiểm tra trên bản đồ nhỏ góc phải ảnh overlay:
- Đi thẳng: đường vàng nằm giữa 2 vạch xanh, kéo dài xuống tới khung xe (vùng mù được lấp).
- Vào cua: vạch trong chuyển màu **tím** (vạch ảo) khi ra khỏi khung hình, vẫn nằm đúng chỗ.
- Đường vàng / vạch bị "cong lệch" dần khi xe quay ⇒ `steer_ratio` hoặc `wheelbase_m` sai.
- `/lane/status`: `mode=track turn=R mem=0.85 la=0.63 wheel=+18.0 guard=+0.0`.
  `guard` ≠ 0 thường xuyên ⇒ xe hay sát vạch ⇒ kiểm tra `steer_ratio`, giảm tốc cua.

Muốn quay lại cách lái cũ để so sánh: `./run.sh steer_mode:=camera`.

## 8. MPC giữ làn + tích phân LQI (`steer_mode:=track`, mặc định)

Khi có đường tâm phủ tới trục sau, `lane_keeping` lái bằng **MPC** (mô hình xe đạp tương đối
với làn: `e_y, e_psi, d_act`, xem trước độ cong 1.2 s, bù trễ servo) thay cho pure pursuit +
phản hồi lệch ngang. Pure pursuit / `xte_*` chỉ còn là **dự phòng** khi MPC không chạy được.
Tham số trong launch (tiền tố `mpc_`):

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `mpc_enable` | true | bật MPC. false = lái như bản 09/10 (pure pursuit + lệch ngang) |
| `mpc_tau_s` | 0.15 | trễ servo + lọc (s) trong mô hình |
| `mpc_horizon_s` | 1.2 | tầm dự đoán (s) |
| `mpc_zone_m` | 0.05 | \|e_y\| từ mức này coi là "xa tâm": ưu tiên kéo về tâm; gần tâm ưu tiên giữ hướng cho êm |
| `mpc_rd` | 30 | phạt thay đổi góc lái (êm). Lái giật ⇒ tăng; vào cua chậm ⇒ giảm |
| `mpc_g0` / `mpc_b0_deg` | 0.70 / 0.0 | tỉ số lái thật và độ lệch lái thật ban đầu; oracle tự học tiếp |
| `mpc_learn` | true | học `g` / `b` online từ độ cong đo bằng camera |
| **`mpc_qi`** | **60** | **LQI: trọng số tích phân lệch ngang** `s = ∫e_y dt`. MPC thường (không tích phân) để **lệch dư tại trục** khi mô hình sai (`b`, `g`, deadband servo, camera lệch). **0 = tắt (MPC cũ)** |
| **`mpc_i_max`** | **0.06** | chặn tích phân (m·s) — chống wind-up khi vừa mất làn / lệch lớn |

**Vì sao thêm LQI (15/10):** mô phỏng cho thấy MPC cũ đứng lệch dư ~3.9 cm khi `b` lệch 3° chưa
học, ~4.9 cm khi tỉ số lái sai (0.42 so với 0.70) — đúng hiện tượng "xe không bám giữa làn /
lệch dần". Thêm tích phân (`mpc_qi≈60`) đưa lệch về ~0 trong mọi trường hợp, hội tụ < 2 s, vẫn
ổn định khi có nhiễu detector và trong cua. Vì tốc độ đã tự hạ khi `xte` > 3 cm, giữ giữa làn
tốt cũng làm xe **chạy nhanh hơn** (ít khi rớt về mức cua).

Chỉnh nhanh: còn lệch dư một bên ⇒ tăng `mpc_qi` (80–120); xe dao/lắc quanh tâm ⇒ giảm
(30–40) hoặc giảm `mpc_i_max`. `mpc_qi:=0` để quay lại MPC cũ.

> `xte_ki` (§7) chỉ tác dụng khi MPC **không** chạy; lúc MPC chạy, tích phân do `mpc_qi` đảm
> nhiệm (không cộng dồn hai lần).
