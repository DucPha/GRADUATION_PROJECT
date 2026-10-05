# Progress

## Trạng thái
Giai đoạn 1 — sửa `camera_node` — **đã xong**.

## Đã xong
- Đọc `.agent/skills/karpathy-guidelines`, `.agent/skills/planning-with-files`,
  `.agent/skills/superpowers`, `README.md`.
- Số đo của người dùng: làn 30–50 cm, camera cao 30 cm, xe mô hình.
- Chốt bỏ IPM (đo cm bằng công thức pinhole), bỏ nhánh Hough, bỏ 14 topic debug.
- Tạo `docs/notes/task_plan.md`, `docs/notes/findings.md`.
- Thêm vào `detect()`: Gaussian blur, CLAHE, mask ROI hình thang, fit bậc 2,
  đo `lane_width_cm` / `dev_cm`, vẽ đường fit lên `/lane/vis`.
- Vẽ thêm lên `/lane/vis`: hình thang ROI, đường chân trời, hàng lấy `dev`, nhãn khoảng cách.
- Kéo `ROI_BOTTOM_FRAC` 0.88 → 0.92 → **0.97** (tới sát đầu xe theo yêu cầu).
- Nới `LANE_WIDTH_MAX` 190 → **230** (bắt buộc đi kèm, xem dưới).
- **Sửa lỗi**: bề rộng làn trước đây đo ở cửa sổ 4 (xa nhất, hẹp nhất) thay vì cửa sổ 0
  (gần xe, rộng nhất). Làn 30 cm ở cửa sổ 4 chỉ 32 px < `LANE_WIDTH_MIN` nên bị lo oi.
- `ROI_TOP_FRAC` 0.45 → 0.58.
- Thêm `fit_poly2()` + `px_to_cm()` vào `camera_node.cpp`.
- `LaneOutput` có thêm `dev_cm`, `lane_width_cm`, `fit_ok`.
- `fusion_viz_node` in thêm `w=` và `devm=` vào `/lane/status`.
- Cập nhật `README.md`, `docs/ARCHITECTURE.md`, `docs/TUNING.md`.

## Đã kiểm chứng
- `g++ -std=c++17 -fsyntax-only -Wall -Wextra` trên `camera_node.cpp`: exit 0,
  0 cảnh báo từ file của mình.
- Test toán học `fit_poly2` + `px_to_cm` (10 case): **10/10 pass**.
  Residual tối đa 0.171 px; lane thẳng không đổi lệch; có chặn độ cong vô lý,
  chặn < 3 điểm, chặn y-span < 4.
- Test `snprintf` `/lane/status`: 11 `%` khớp 11 đối số, chuỗi dài nhất 121 ký tự
  trong buffer 384.
- Test khoảng thi với ROI 0.58–0.97 (10 case): **10/10 pass**. ROI phủ
  **0.66 m → 2.14 m**; làn 50 cm nằm trong [50, 230] px ở cả 5 cửa sổ;
  làn 30 cm giữ được 3/5 cửa sổ; làn 90 cm bị lo.

## Vùng detector sau khi sửa (làn 50 cm, camera 0.30 m)

| Cửa sổ | Hàng | Bề rộng | Cách xe |
|---|---|---|---|
| 0 | 215–233 | 173 px | 0.66 m |
| 1 | 197–215 | 143 px | 0.80 m |
| 2 | 179–197 | 113 px | 1.01 m |
| 3 | 161–179 | 83 px | 1.37 m |
| 4 | 143–161 | 53 px | 2.14 m |

## Điều chưa làm / cần kiểm chứng sau
- `colcon build` — phải chạy trên Mini PC (máy này không có rclcpp).
- Chưa chạy thử trên ảnh thật của xe.
- `HORIZON_Y = 120` là **ước lượng**, chưa hiệu chỉnh. Sai số cm tỉ lệ thuận với nó;
  vị trí lái không bị ảnh hưởng.
- Mask ROI hình thang dùng `half_top = WORK_W/6` — chưa đo lại trên ảnh thật.
- Chưa đổi firmware. `dev_px` giữ nguyên đơn vị pixel nên protocol không đổi.
- Yêu cầu dọn code thừa trong `.ino` vẫn chưa làm (hoãn, ngoài phạm vi camera).