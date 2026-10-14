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

---

## 2026-10-15 — Nâng cấp pipeline detector (tăng cường ảnh + chống vật thể giả + độ phân giải)

Bối cảnh: vạch băng keo cao su **đen** bị **lóa đèn trần**, mất vạch ở cua, và các **mảng tối
(gầm bàn, chân ghế)** bị nhận nhầm thành vạch ⇒ xe lao vào vật cản. `GaussianBlur(5,5)` cũ làm
mờ vạch mỏng/xa.

### Thay đổi
- **`WORK_W` 320 → 960** (khung xử lý 960×540 cho 16:9, đúng bằng ảnh giải mã 1/2 của
  camera 1920×1080 nên không còn bước resample). Mọi hằng số pixel ghi theo ảnh tham
  chiếu 320 và tự nhân tỉ lệ qua `CameraLane::scaled_px()`; đặt lại 320/640 là quay về nhẹ hơn.
  Công thức pinhole không đổi (đã kiểm tra: `(x−cx)` và `(y−horizon)` cùng nhân tỉ lệ).
- **`enhance()` mới** thay `GaussianBlur` trong `detect()`: bilateral (giữ cạnh) → CLAHE
  (tương phản cục bộ) → unsharp (bù mờ hạ mẫu). Đồng thời sinh **mặt nạ màu** và **mặt nạ
  cháy sáng** (`GLARE_V_MIN`/`GLARE_S_MAX`).
- **Từ chối vật thể giả** trong `extract_segments()`: (a) bề dày trung vị > `TAPE_MAX_M`;
  (b) dài/thon `≥ TAPE_ASPECT_MIN`; (c) **bề dày mặt đất xa/gần ≤ `TAPE_THICK_RATIO_MAX`**
  → loại vật thể đứng (chân ghế); (d) **hai bên pháp tuyến sáng hơn lõi** ≥ `TAPE_SIDE_BRIGHT_MARGIN`
  trên ≥ `TAPE_SIDE_PASS_FRAC` số điểm → loại mép bóng / vùng tối rộng.
- **Nối qua vùng lóa**: `link_segments()` nhận mặt nạ cháy sáng; nếu điểm giữa khoảng hở nằm
  trong vùng cháy sáng thì cho hở ≤ `LINK_GAP_GLARE_M` (0.60 m) thay vì `LINK_GAP_M` (0.35 m).
- Overlay `/lane/vis` thêm **mảng tím mờ** = vùng cháy sáng, để phân biệt lỗi lóa với lỗi detector.

### Đã kiểm chứng
- `g++ -std=c++17 -fsyntax-only -Wall -Wextra` (thêm `-DJCS_EXT_BGR=JCS_RGB` vì libjpeg
  MinGW thiếu enum): `camera_node.cpp` exit 0, **0 cảnh báo** từ file của mình; `path_tracker.cpp` exit 0.
- Đo chi phí bằng `cv2` Python 640×220: đóng hình thái 90×50 = 0.17 ms, bilateral = 0.39 ms,
  CLAHE = 0.08 ms, `connectedComponentsWithStats` = 1.28 ms ⇒ pipeline mới ước ~3–5 ms/frame,
  còn xa ngưỡng 38 ms (26 fps).

### Cần kiểm chứng trên Mini PC
- `colcon build` + chạy ảnh thật; xem `/lane/vis` để chỉnh các ngưỡng mới (mục 1a/1b trong
  `docs/TUNING.md`). Nếu vạch thật bị loại oan ⇒ tăng `TAPE_THICK_RATIO_MAX` / giảm `TAPE_SIDE_PASS_FRAC`.
- Có thể đặt camera 640×480 để decode rẻ hơn (decode thẳng 640, không qua 960×540).
- Chưa thêm cue **Canny/cạnh tường minh** (hiện cạnh được tăng cường gián tiếp nhờ CLAHE +
  unsharp + phép thử hai bên); cân nhắc nếu vẫn thiếu vạch ở vùng tương phản thấp.