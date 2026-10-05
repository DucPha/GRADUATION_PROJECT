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

## 1. Cửa sổ trượt

| Hằng số | Mặc định | Khi nào chỉnh |
|---|---|---|
| `N_WINDOWS` | 5 | ảnh nhiễu nặng thì giảm còn 4 |
| `WINDOW_MARGIN` | 30 | làn cong mạnh, seed lạc mất đỉnh thì tăng lên 40 |
| `WINDOW_MIN_POINTS` | 8 | ảnh tối, vạch mờ thì giảm xuống 5 |
| `MAX_RUN_WIDTH` | 25 | bóng đổ bị nhận nhầm thành làn thì giảm xuống 18 |
| `MIN_LANE_GAP` | 30 | một khối bị tách 2 đỉnh thì tăng lên 40 |
| `MIN_MATCHED_WINDOWS` | 3 | detector rớt thì hạ xuống 2 |

## 2. Vùng làm việc

| Hằng số | Mặc định | Ý nghĩa |
|---|---|---|
| `ROI_TOP_FRAC` | 0.58 | bỏ phần trên ảnh (trời, chân trời) |
| `ROI_BOTTOM_FRAC` | 0.93 | chừa khoảng trống hai bên làn; cửa sổ 0 là nơi lấy seed |

`ROI_TOP_FRAC` có thể chỉnh lúc chạy bằng tham số `roi_top_frac` mà không cần build lại.
`ROI_BOTTOM_FRAC` phải sửa trong header.

**Cách hiệu chỉnh `ROI_TOP_FRAC`:** nhìn `/lane/vis`. Nếu cửa sổ trên cùng vẽ đè lên
trời (không thấy vạch nào trong khung), tăng lên. Nếu 2 làn bị mất quá sớm ở giữa khung,
giảm xuống. Camera cao 0.30 m thì chân trời ở khoảng hàng 120 ⇒ `0.58` là điểm khởi đầu.

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
| `EMA_ALPHA` | 0.35 | lái càng mềm càng giảm (0.2); càng nhạy càng tăng (0.5) |
| `STALE_AGE_MS` | 200 | quá tuổi thì coi như mất camera |
| `lane_lost_stop_ms` (launch) | 400 | mất 2 làn bao lâu thì dừng |
| `speed_x10` (launch) | 40 | tốc độ khi đủ 2 làn (km/h × 10) |
| `speed_hold_x10` (launch) | 20 | tốc độ khi mất 2 làn (km/h × 10) |

## 5. Ngưỡng ảnh

Đang dùng Otsu nên tự thích sáng phòng, không cần chỉnh tay. Nếu ảnh quá tối hoặc quá
chói, Otsu bị lệch thì chuyển `cv::threshold(gray, bin, 0, 255, ...)` trong
`camera_node.cpp` sang ngưỡng cố định:

```cpp
cv::threshold(gray, bin, 70, 255, cv::THRESH_BINARY_INV);
```

Nhớ đổi cả phép `|` thành `cv::THRESH_BINARY_INV | cv::THRESH_OTSU` → bỏ `| cv::THRESH_OTSU`.

## 6. Hướng lái

- Firmware giữ `dev` trong khoảng **-50..50 px** (ảnh gốc) và có vùng chết ±10 px.
- Lệch quá 50 px là bão hoà lái hết cỡ.
- Nếu xe đẩy sang phải mà bánh xe càng lái càng xa, chạy lại với `dev_sign:=-1`.
- Lệch ảnh nhỏ lắm (vài px) mà xe vẫn không hồi được về giữa: xem `EMA_ALPHA`.