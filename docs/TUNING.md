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
| `RADIAL_LINE_DEG` / `RADIAL_LINE_MIN_M` / `RADIAL_KINK_DEG` | 12 / 0.12 / 30 | lọc chân bàn/ghế trên từng VẠCH: đoạn đầu vạch chĩa về camera (chân ghế dính bóng thành chữ V) ⇒ bỏ vạch; đoạn sau góc gập ≥ 30° chĩa về camera (đi dọc băng keo rồi rẽ vào chân ghế đứng sát) ⇒ cắt bỏ đoạn đó |
| `ZIGZAG_DEG` / `ZIGZAG_GAP_M` | 35 / 0.12 | 2 góc gập ngược chiều sát nhau (răng cưa: mép bóng, rẽ vào hộp giấy) ⇒ cắt vạch tại đó |
| `LINE_MIN_CONTRAST_FRAC` | 0.75 | độ tối trung vị dọc vạch phải ≥ 0.75 × độ tối của chính vạch đó ở frame trước (vạch mới: so với vạch đang bám nhạt hơn) ⇒ không bám sang vệt bóng. Vạch thật bị bỏ khi đi qua vùng tối ⇒ giảm (0.6) |
| `LANE_W_MIN_M` / `LANE_W_MAX_M` | 0.28 / 0.70 | khoảng bề rộng làn chấp nhận khi ghép cặp — **tuyệt đối**, không theo bề rộng đã học (làn dán tay rộng hẹp từng đoạn) |
| `LANE_W_LOCAL_ALPHA` / `LANE_W_LOCAL_HOLD_SEC` | 0.35 / 3 | bề rộng làn **tại chỗ** (đo mỗi frame thấy 2 vạch); chỉ thấy 1 vạch thì dời tâm nửa bề rộng này |

Vạch trái / phải không bao giờ được trùng hay đổi chỗ cho nhau: vạch bám lại được phải nằm
đúng phía vạch kia; mâu thuẫn thì giữ vạch đã bám lâu hơn (trước đây một vệt bóng nhận làm
vạch trái có thể "bám" sang băng keo phải ⇒ vạch phải thật bị bỏ, xe lái ra ngoài).

## 1b. Hình học camera — chỉnh ĐẦU TIÊN

Detector chiếu ảnh xuống **mặt sàn** (BEV, 1 cm/ô, `src/camera_node/lane_bev.cpp`) nên mọi
phép đổi pixel ↔ mét dựa vào 3 số trong launch:

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `camera_pitch_deg` | 35.9 | góc cúi camera. 08/10 (log Python): ~42°. 09/10 camera được ngửa lên, đo lại trên ảnh live 1920×1080: điểm tụ 4 mép của 2 vạch thẳng ⇒ 35.6–36.0°, bộ tự hiệu chỉnh của detector ra 35.89°, bề rộng làn không đổi theo khoảng cách ⇒ 35.9°. **Chỉnh camera lại thì phải đo lại** (cách đo: dòng `camera_auto_pitch` bên dưới) |
| `camera_height_m` | 0.30 | tâm ống kính cách sàn |
| `camera_vfov_deg` | 51.0 | góc nhìn **dọc**. Chế độ 1920×1080 và 640×480 của cam này cùng góc dọc (4:3 chỉ cắt 2 bên) |
| `camera_auto_pitch` | false | tự hiệu chỉnh góc cúi khi chạy thẳng thấy 2 vạch (±8° quanh `camera_pitch_deg`). Mặc định TẮT: thử vòng kín, ước lượng nhảy 42 → 44.5 → 42.7° dù góc thật cố định, mỗi lần đổi > 2° còn xoá vạch đang bám. Bật để đo 1 lần: log in `Tu hieu chinh goc cui camera: ... -> X` ⇒ điền X vào `camera_pitch_deg` rồi tắt lại |
| `lane_width_m` | 0.42 | bề rộng làn **ban đầu** (tâm vạch – tâm vạch). Không cố định: mỗi frame thấy 2 vạch đo lại bề rộng tại chỗ, ghép cặp chấp nhận 0.28–0.70 m |

Kiểm tra: đặt xe giữa 2 vạch thẳng, overlay phải hiện `2 LANES` và `w` ≈ bề rộng làn thật.
Camera cúi ~36° nên **không thấy chân trời** trong ảnh; mặt sàn nhìn thấy từ ~0.17 m tới
~1.6 m trước chân camera (cúi 42° cũ: 0.13–1.1 m) (ô BEV góc trên trái overlay).

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
| `speed_x10` | 66 | đủ 2 vạch + đường thẳng + xe giữa làn: 1563 µs = đúng xung đường thẳng bản cũ (70 / 78 cũ đều ra 1562.5 µs, xem §4b). Cua / xe lệch nội suy **liên tục** về `speed_corner_x10` |
| `speed_one_x10` | 66 | chỉ thấy 1 vạch, đường thẳng, xe giữa làn (60 cũ cũng ra 1562.5 µs). Trước đây 1 vạch luôn chạy tốc độ cua ⇒ đường thẳng mà 1 vạch bị mờ thì xe bò chậm |
| `speed_tight_x10` | 50 | tốc độ ở cua GẮT. PathTracker đo `corner_k` = độ cong lớn nhất của đường tâm từ bánh trước trở đi (độ cong phải cao liên tục ≥ 15 cm mới tính, giữ đỉnh 0.4 s; log `ck=`). `ck` ≤ `corner_k_easy` (0.9, R ≥ 1.1 m) ⇒ `speed_corner_x10`; ≥ `corner_k_tight` (1.25, R ≤ 0.8 m) ⇒ `speed_tight_x10`. Thử vòng kín: R 0.8 ở 55 vọt 10 cm (lái gần hết), ở 50 còn 7 cm |
| `steer_slow_start_deg` / `steer_slow_full_deg` | 4 / 10 | góc bánh (đã lọc) ≥ 4° bắt đầu, ≥ 10° ghim hẳn tốc độ về tốc độ cua. Nửa sau cua tầm nhìn đã thấy đường thẳng nên trước đây xe tăng tốc khi còn đang bẻ lái ⇒ văng ra ngoài ở lối ra cua (nguyên nhân vọt lố lớn nhất trong sim) |
| `speed_corner_x10` | 55 | 1 vạch, hoặc 2 vạch ở cua gắt nhất: 1556 µs (bản cũ 1543 µs, BLDC kêu; 50 = 1552) + firmware bù tới 3 µs khi hết lái |
| `speed_hold_x10` | 50 | mất cả 2 vạch, đang giữ hướng |
| `speed_min_x10` | 50 | sàn tốc độ khi đang chạy (1552 µs). Vẫn kêu ⇒ tăng 2–3 (mỗi đơn vị ≈ 0.65 µs), không cần nạp lại ESP32 |
| `speed_ramp_x10` | 30 | tăng tốc 3 km/h mỗi giây; giảm tốc luôn tức thì |
| `coast_decel_mps2` | 0.7 | xe nhả ga (ESC không phanh) giảm tốc cỡ bấy nhiêu m/s². Dùng để **giảm tốc trước cua** và cho odometry khi giảm tốc. Vẫn vọt cua ⇒ giảm (0.5) |
| `corner_margin_m` | 0.20 | phải xuống tốc độ cua trước khi tới cua bấy nhiêu m |

`tốc độ = corner + (speed − corner) × hệ số` (`speed` = `speed_x10` khi 2 vạch, `speed_one_x10`
khi 1 vạch). Hệ số = nhỏ nhất của: `speed_scale` camera, độ cong đường đã nhớ phía trước, và
**xe giữa làn** (lệch ≤ 3 cm ⇒ 1, ≥ 7 cm ⇒ 0: xe 25 cm trong làn 0.42 m chỉ còn ~5 cm mỗi bên).
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

### 4b. Xung ESC thật (đo 09/10)

ESP32Servo mặc định timer **10 bit** ⇒ xung chỉ có bước 20000/1024 = **19.53 µs**. Bản cũ
`Servo.write(góc)` nên các "góc" thực ra ra cùng xung:

| Góc cũ | 90 | 95 | 100, **101** | 102, 103, 104 | 105, 106 |
|---|---|---|---|---|---|
| Xung thật | 1484.4 | 1523.4 | **1543.0** | 1562.5 | 1582.0 µs |

⇒ sàn "101" = 100 (BLDC kêu), tốc độ chỉ nhảy 2 bậc 1543 ↔ 1562.5 µs (cua ↔ thẳng). Firmware
mới đặt ESC sang timer 16 bit (0.3 µs/bước), **giữ đúng xung thật cũ** cho neutral, phanh, đề-pa
và các điểm `ESC_LUT`, chỉ nội suy mịn ở giữa: 50 → 1552, 55 → 1556, 60 → 1559, 66 → 1563,
70 → 1567, 78 → 1576 µs. Sàn cứng `ESC_START_US = 1550`, bù tải `ESC_STEER_BOOST_US = 3` µs khi
hết lái. Telemetry `esc=` giờ là góc tương đương xung THẬT (1543 → 98, 1552 → 99, 1562.5 → 101).
Node mới vẫn chạy được với firmware cũ (66 → ESC 102, 50 → ESC 101: y như trước).

## 5. Mask vạch (giống `lane_bev.py`)

**Vạch chạy vào đồ vật (09/10).** Khi đi dọc vạch, detector đo thêm **mặt cắt ngang** vạch trên
mask đã lọc. Chỗ nào rộng hơn 2 lần bề rộng của chính vạch đó và rộng hơn 8 cm thì đó là đồ vật
(tờ giấy, vùng bóng, dây cáp), không phải băng keo: dừng vạch tại đó (`WIDE_FRAC`, `WIDE_MIN_M`
trong `camera_node.hpp`). Đuôi vạch bẻ ≥ 45° mà ngắn < 10 cm cũng bị cắt (`TAIL_BEND_DEG`).
So trên log thật 08/10 (2870 frame) và ảnh live: bỏ 20 "góc cua" giả (kiểm tra từng frame bằng
mắt, đều là giấy / bóng / mép ảnh), **không mất thêm frame nào**, góc cua thật vẫn nhận; live:
báo cua nhầm 19.6 % → 3.3 %, giật `dev` 3.2 → 0.9 px/frame. 1.6× / 6 cm hoặc đuôi 35° bắt đầu
làm mất vạch thật ⇒ không dùng.

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
- `ESC_START_US = 1550`: xung ga thấp nhất khi đang chạy (bản cũ 1543 µs: BLDC kêu), xem §4b.
- `ESC_STEER_BOOST_US = 3`: cộng thêm tới 3 µs khi servo hết lái (bánh lái bè gây tải, BLDC tụt vòng và kêu). 0 = tắt.
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
| `corner_gain` | 1.25 | (09/10: 1.15 → 1.25 khi tăng tốc độ cua, đánh lái dứt khoát hơn; hệ số giờ theo max(độ cong phía trước, độ cong đang lái) để không tắt ở nửa sau cua) trong cua (độ cong phía trước 0.4 → 1.0 1/m) nhân góc lái pure pursuit với hệ số này. Vẫn văng ra ngoài cua ⇒ tăng (1.25); cắt vào trong ⇒ giảm (1.0 = tắt) |
| `steer_filter_s` | 0.12 | lọc góc bánh trên đường thẳng (Python 0.08). 09/10: 0.08 → 0.12, thử vòng kín rung servo trên đường thẳng −17 %, lệch đường thẳng −17 %. Xe lắc chậm qua lại trên đường thẳng ⇒ giảm về 0.08 |
| `steer_filter_corner_s` | 0.03 | lọc góc bánh trong cua: độ cong phía trước 0.4 → 1.0 1/m giảm dần hằng số lọc từ `steer_filter_s` (0.08 s) về mức này ⇒ bánh bẻ vào cua sớm hơn ~50 ms. Servo giật trong cua ⇒ tăng (0.05); = `steer_filter_s` để tắt |
| `xte_gain` | 2.0 | phản hồi lệch ngang tại chân camera: `atan(xte_gain·e / (v + 0.5))`. Pure pursuit thuần lệch 5 cm chỉ bẻ ~3°. Xe lắc qua lại trên đường thẳng ⇒ giảm (1.0); hay lệch ra ngoài cua ⇒ tăng (2.5) |
| `xte_ki` / `xte_i_max_deg` | 50 / 6 | tích phân lệch ngang (độ bánh / m / s): bù trim servo, camera lắp lệch/xoay vài độ (`STEER_KI` / `STEER_I_LIMIT` bản Python). `/lane/status`: `xte=` lệch (cm), `xi=` phần tích phân (độ) |
| `lookahead_min_m` / `lookahead_gain_s` / `lookahead_max_m` | 0.35 / 0.20 / 0.80 | tầm nhìn = min + gain·v. Xe lắc trên đường thẳng ⇒ tăng `min` (0.45); cắt cua ⇒ giảm `min` (0.30) |
| `lookahead_corner_scale` | 0.7 | **tầm nhìn tự thích ứng**: độ cong phía trước 0.4 → 1.0 1/m thì tầm nhìn co dần về 0.7 lần (≥ `min`) ⇒ bám sát cung cua. Thử vòng kín: chạm vạch 20 % → 17 %, vọt cua giảm, đường thẳng không đổi. Kéo DÀI tầm nhìn trên đường thẳng thì xe lệch tâm nhiều hơn (không dùng); BEV rộng ±1.1–1.4 m thay vì ±0.8 không cải thiện (bộ nhớ đường đã phủ đủ). GUI: chấm xanh lơ + số mét = điểm ngắm đang dùng |
| `actuator_latency_s` | 0.08 | xe vào cua trễ, văng ra ngoài ⇒ tăng (0.12) |
| `camera_latency_s` | 0.04 | tương tự, cho trễ camera |
| `car_half_width_m` / `tape_half_m` / `line_margin_m` | 0.125 / 0.035 / 0.02 | rào chắn vạch: nửa bề ngang xe (xe rộng 25 cm) / nửa bề rộng băng keo / khoảng hở tối thiểu tới MÉP băng keo. Xe ở giữa làn 0.42 m chỉ còn ~5 cm mỗi bên nên rào chắn bắt đầu đẩy khi xe lệch ~3 cm |
| `odom_speed_scale` | 1.0 | tốc độ thật / tốc độ lệnh. Đo: cho chạy 2 m thẳng, bấm giờ |
| `lost_memory_ms` | 1500 | mất cả 2 vạch: chạy tiếp theo đường đã nhớ tối đa bao lâu |

Kiểm tra trên bản đồ nhỏ góc phải ảnh overlay:
- Đi thẳng: đường vàng nằm giữa 2 vạch xanh, kéo dài xuống tới khung xe (vùng mù được lấp).
- Vào cua: vạch trong chuyển màu **tím** (vạch ảo) khi ra khỏi khung hình, vẫn nằm đúng chỗ.
- Đường vàng / vạch bị "cong lệch" dần khi xe quay ⇒ `steer_ratio` hoặc `wheelbase_m` sai.
- `/lane/status`: `mode=track turn=R mem=0.85 la=0.63 wheel=+18.0 guard=+0.0`.
  `guard` ≠ 0 thường xuyên ⇒ xe hay sát vạch ⇒ kiểm tra `steer_ratio`, giảm tốc cua.

Muốn quay lại cách lái cũ để so sánh: `./run.sh steer_mode:=camera`.
