# HƯỚNG DẪN HIỆU CHỈNH NHANH

Mọi hằng số detector nằm trong `src/camera_node/camera_node.hpp`. Sửa ở đó
rồi build lại. Tham số nào tiện thì expose qua launch (`ros2 launch fusion_node
fusion.launch.py <tên>=<giá trị>`).

## 0. Trước hết: nhìn `/lane/vis`

Ảnh overlay cho biết ngay detector đang dừng ở bước nào:

- Khung **xám** = cửa sổ dùng được, khung **đỏ** = cửa sổ bị loại
- **Trái xanh lá**, **phải xanh dương** = 2 làn tìm được
- Hai **vòng tròn vàng** = 2 điểm seed (2 vạch được tách ở cửa sổ thấp nhất)
- **Đường vàng dọc** = tâm 2 làn, chỉ vẽ khi `two_lanes = true`
- Chữ lớn: `2 LANES OK` / `NO 2 LANES`

## 1. Cửa sổ trượt và ổn định hoá

| Hằng số | Mặc định | Khi nào chỉnh |
|---|---|---|
| `N_WINDOWS` | 6 | ảnh nhiễu nặng thì giảm còn 5 |
| `WINDOW_MARGIN` | 30 | làn cong mạnh, mất vạch khi tìm mới thì tăng lên 40 |
| `TAPE_MAX_M` | 0.12 | bề rộng tối đa 1 vạch (m); vạch bản rộng bị loại thì tăng |
| `BG_KERNEL_W` × `BG_KERNEL_H` | 61 × 5 | cửa sổ ước lượng nền (trừ nền chống lóa); phải rộng hơn bề ngang vạch lớn nhất. Vạch sát xe bị mất ở mask thì tăng |
| `BG_MIN_DIFF` / `BG_WEAK_DIFF` | 12 / 8 | ngưỡng mạnh tối thiểu / ngưỡng yếu của hysteresis. Vân sàn bị nhận là vạch thì tăng; đoạn vạch trong lóa vẫn đứt thì giảm `BG_WEAK_DIFF` |
| `PRIOR_MARGIN` | 18 | biên tìm quanh vạch của frame trước; xe nhanh / cua gắt mà mất vạch thì tăng |
| `PRIOR_MAX_AGE_MS` | 250 | vị trí vạch cũ quá tuổi này thì bỏ, tìm lại từ đầu |
| `WINDOW_MIN_POINTS` | 2 | ảnh nhiễu nhiều thì tăng lên 3 |
| `MIN_MATCHED_WINDOWS` | 3 | detector hay rớt `2 LANES` thì hạ xuống 2 |
| `CENTRE_OUTLIER_PX` | 7.0 | điểm tâm lệch đường fit quá mức này bị loại (px khung 320) |
| `JUMP_GATE_REF_PX` | 60 | `dev` nhảy quá mức này (px ảnh 640) thì giữ giá trị cũ 1 frame |
| `EMA_ALPHA` | 0.40 | lái càng mềm càng giảm (0.25); càng nhạy càng tăng (0.55) |

Overlay hiện dấu `*` sau trạng thái khi frame bị chặn nhảy; `/lane/status` có `gate=1`.
Nếu thấy `*` liên tục khi vào cua thì tăng `JUMP_GATE_REF_PX`.

## 1b. Chân trời — chỉnh ĐẦU TIÊN

`horizon_frac` (launch, mặc định `0.20`) là hàng chân trời theo tỉ lệ chiều cao ảnh. Mọi
phép đổi pixel ↔ mét (bề rộng làn, bề rộng vạch) dựa vào nó; sai ⇒ detector ghép nhầm
cặp vạch (thường thấy: `1 LANE` dù nhìn rõ 2 vạch, `dev` rất lớn, `w=-1`).

Cách chỉnh: đặt xe giữa 2 vạch **thẳng**, nhìn ảnh overlay, kéo dài 2 vạch lên trên — chỗ
chúng gặp nhau là chân trời; vạch tím phải nằm ngang qua đúng chỗ đó. Vạch tím thấp hơn
điểm gặp ⇒ giảm `horizon_frac`, cao hơn ⇒ tăng. Ví dụ: `./run.sh horizon_frac:=0.18`.

Sau đó kiểm tra ô LANE WIDTH trên dashboard ra gần bề rộng làn thật; lệch nhiều thì chỉnh
`camera_height_m` (cm đo được tỉ lệ thuận với nó).

## 2. Vùng làm việc

> Camera 16:9 (1920x1080) ⇒ khung làm việc 320x180, chân trời tự tính ở ~hàng 90 (không
> phải 120 như khung 320x240 cũ). Log khởi động in `geometry: ... horizon_y=..., ROI rows ...`.

| Hằng số | Mặc định | Ý nghĩa |
|---|---|---|
| `ROI_TOP_FRAC` | 0.52 | bỏ phần trên ảnh; nhỏ hơn = thấy xa hơn (làn 2 vạch rộng) |
| `ROI_BOTTOM_FRAC` | 0.93 | chừa khoảng trống hai bên làn; cửa sổ 0 là nơi lấy seed |
| `ROI_MIN_DY` (camera_node.hpp) | 12 | đỉnh ROI không cao hơn chân trời + 12 hàng |

`ROI_TOP_FRAC` có thể chỉnh lúc chạy bằng tham số `roi_top_frac` mà không cần build lại.
`ROI_BOTTOM_FRAC` phải sửa trong header.

**Cách hiệu chỉnh `ROI_TOP_FRAC`:** nhìn `/lane/vis`. Nếu cửa sổ trên cùng vẽ đè lên
trời (không thấy vạch nào trong khung), tăng lên. Nếu 2 làn bị mất quá sớm ở giữa khung,
giảm xuống. Camera cao 0.30 m thì chân trời ở khoảng hàng 120 ⇒ với mặc định `0.52`
đỉnh thật bị kẹp ở hàng 132 (chân trời + `ROI_MIN_DY = 12`). Muốn thấy xa hơn nữa thì
giảm `ROI_MIN_DY` trong `camera_node.hpp`, nhưng dưới ~10 hàng thì vạch xa mỏng hơn
2 px và bị `MORPH_OPEN(3,3)` xóa mất.

Lưu ý: điểm rộng nhất của 2 làn **không** nằm ở đáy ảnh, nên đừng đặt
`ROI_BOTTOM_FRAC` quá gần 1.0.

## 2b. Đo centimet

| Hằng số | Mặc định | Ý nghĩa |
|---|---|---|
| `CAMERA_HEIGHT_M` | 0.30 | chiều cao camera trên mặt đường |
| `HORIZON_Y` | 120 | hàng chân trời trong khung 320x240 |
| `LOOKAHEAD_FRAC` | 0.58 | hàng lấy `dev`, tính trong vùng ROI |

Công thức: `W_cm = w_px · CAMERA_HEIGHT_M · 100 / (y − HORIZON_Y)`

- **Cách hiệu chỉnh `HORIZON_Y`:** dùng `/lane/vis`, tìm chỗ trời gặp mặt đường rồi
  đếm hàng tương ứng. Sai 5 px làm sai số cm khoảng 4 %.
- **Sai số cm không ảnh hưởng vị trí lái.** `dev_px` vẫn là pixel ảnh gốc nên firmware
  không đổi. Số cm chỉ để ghi vào báo cáo.
- Quá gần chân trời (`y − HORIZON_Y ≤ 1`) ⇒ `px_to_cm` trả `0`, tức là "không đo được".
  Trong `/lane/status` giá trị này hiện thành `-1`.

## 3. Bề rộng làn

| Hằng số | Mặc định |
|---|---|
| `LANE_WIDTH_MIN` | 50 px (ở khung 320x240) |
| `LANE_WIDTH_MAX` | 230 px |

Đo ở **cửa sổ 0** (gần xe nhất, cũng là điểm rộng nhất).

> **Hai hằng số này ràng buộc lẫn nhau — kéo `ROI_BOTTOM_FRAC` xuống mà quên nới
> `LANE_WIDTH_MAX` thì xe sẽ tự dừng.** Bề rộng làn ở cửa sổ 0 là:
>
> | `ROI_BOTTOM_FRAC` | hàng đáy | làn 30 cm | làn 50 cm | làn 60 cm |
> |---|---|---|---|---|
> | 0.92 | 221 | 101 px | 168 px | 202 px |
> | **0.93** (hiện tại) | **223** | — đo lại — | — đo lại — | — đo lại — |
> | 0.97 | 233 | 104 px | 173 px | 208 px |
> | 0.98 | 235 | 115 px | 192 px | 230 px |
>
> Còn `LANE_WIDTH_MAX = 230` nên chấp nhận làn tới 0.63 m. Muốn kéo xuống 0.98 thì
> phải nới `LANE_WIDTH_MAX` lên ~250.
>
> ⚠️ Các số px ở hàng 0.97 là số **đo thật** trên xe. Hàng đáy đã hạ từ 233 xuống 223 nên
> độ rộng px sẽ nhỏ lại một chút — cần đo lại ở 0.93 trước khi chạy thật, không dùng lại
> số cũ.

Đo thật bằng cách nhìn ảnh overlay rồi chọn khoảng bao quanh.

## 4. Lọc và phản ứng

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `STALE_AGE_MS` | 200 | quá tuổi thì coi như mất camera |
| `lane_lost_stop_ms` (launch) | 400 | mất 2 làn bao lâu thì dừng |
| `BRIDGE_MAX_BANDS` (camera_node.hpp) | 3 | vạch đứt đoạn do đèn trần: nội suy vẽ lại tối đa bao nhiêu cửa sổ |
| `speed_x10` (launch) | 30 | tốc độ đủ 2 làn & đi thẳng; cua 2 vạch camera nhân `speed_scale` (0.45..1.0); tăng dần theo ramp (km/h × 10) |
| `speed_hold_x10` (launch) | 15 | tốc độ khi mất cả 2 vạch (km/h × 10) |
| `speed_corner_x10` (launch) | 15 | tốc độ khi chỉ thấy 1 vạch / khúc cua (km/h × 10) |
| `speed_ramp_x10` (launch) | 8 | mức tăng tốc, x10 mỗi giây (8 = 0.8 km/h/s) |

## 5. Ngưỡng ảnh (trừ nền + hysteresis)

Không còn ngưỡng trên ảnh xám. Detector ước lượng ảnh nền (phép đóng 61×5), lấy
`diff = nền − ảnh` rồi Otsu trên `diff`, sau đó giữ thêm pixel yếu nối liền với pixel mạnh.
Nhìn panel **BINARY MASK** trên dashboard:

| Triệu chứng | Chỉnh |
|---|---|
| Vạch sát xe (bản rộng) bị khoét rỗng giữa | tăng `BG_KERNEL_W` (81) |
| Vạch trong vùng lóa vẫn đứt | giảm `BG_WEAK_DIFF` (6) |
| Vân sàn / mép bàn hiện thành vạch | tăng `BG_MIN_DIFF` (16) hoặc `BG_WEAK_DIFF` (10) |
| Mask trống dù thấy vạch | giảm `MIN_CONTRAST` (10) |

## 6. Hướng lái (firmware)

- `dev` luôn tính theo ảnh **tham chiếu 640 px** (bất kể camera 1920x1080 hay 640x480).
- Firmware: deadzone `CAM_DEADZONE = 10` px, bão hoà `CAM_MAX_DEV = 50` px → 30°.
- `STEER_KP = 0.8`: xe lắc qua lại ⇒ giảm (0.6); vào cua không đủ gắt ⇒ tăng (tối đa 1.0).
- `STEER_KD = 0.03` s, giới hạn ±`STEER_D_MAX` = 6°: thêm giảm chấn, không gây giật.
- `STEER_RATE_DEG_S = 300`: tốc độ quay tối đa của lệnh servo.
- `ALPHA_STEER = 0.25`: lọc nhẹ `dev` trên ESP32 (Mini PC đã lọc chính).
- Xe đẩy sang phải mà bánh càng lái càng xa ⇒ chạy lại với `dev_sign:=-1`.
