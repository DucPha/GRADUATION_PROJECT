# Task plan — Bổ sung các bước còn thiếu trong pipeline ảnh

## Vấn đề
Detector 2 làn hiện chỉ có: resize → xám → Otsu → morphology → cửa sổ trượt.
Thiếu khử nhiễu, cân bằng ánh sáng, mask ROI, và fit đường. Đồ án cần đủ các bước.

## Ràng buộc (skills/karpathy-guidelines)
- Giữ thay đổi nhỏ, ưu tiên cách đơn giản nhất, tránh refactor lớn.
- **Giữ nguyên protocol serial 11 byte** → `dev_px` vẫn là pixel ảnh gốc.
- Không sửa GPIO. Không chặn vòng lặp.

## Phạm vi (đã chốt)
Thêm đúng 5 thứ, bỏ qua những gì phức tạp mà chưa cần:

| # | Việc | Cỡ |
|---|---|---|
| 1 | GaussianBlur khử nhiễu | 1 dòng |
| 2 | CLAHE cân bằng bóng đổ | 3 dòng |
| 3 | ROI hình thang bằng fillPoly | ~10 dòng |
| 4 | Sửa ROI_TOP_FRAC 0.45 → 0.58 (đang lấy cả trời) | 2 hằng số |
| 5 | fit đường bậc 2 + đo bề rộng làn bằng cm | ~30 dòng |

## Cố ý KHÔNG làm
- **IPM / warpPerspective**: đo cm được bằng công thức pinhole 1 dòng, không cần
  biến đổi phối cảnh. Bỏ để giữ `dev_px` nguyên đơn vị pixel, không sửa firmware.
- **Nhánh Hough / Sobel / Canny / HSV-Lab**: nhánh cửa sổ trượt đã chạy được.
- **`margin_search`, deque 4 frame**: EMA sẵn có đã lọc rung.
- **14 topic debug**: vẽ đủ lên `/lane/vis` hiện có là đủ để chụp báo cáo.

## Công thức đo cm (thay IPM)
Camera cao `h`, chân trời ở hàng `cy`, bề rộng làn `w_px` đo ở hàng `y`:

```
W_m = w_px * h / (y - cy)
```

Chỉ cần 2 số: `h = 0.30 m`, `cy = 120`. Không cần ảnh chessboard.

## Tiêu chí nghiệm thu
- [ ] `g++ -std=c++17 -fsyntax-only -Wall -Wextra` sạch 0 lỗi 0 cảnh báo
- [ ] `two_lanes` và ngưỡng cũ giữ nguyên (không hồi quy)
- [ ] `dev_px` vẫn là pixel ảnh gốc → firmware không sửa
- [ ] `/lane/status` có thêm `w_cm` và `dev_cm`