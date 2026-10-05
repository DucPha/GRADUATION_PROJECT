# Findings — bằng chứng cho các quyết định

## Số đo đã xác nhận (người dùng)
- Bề rộng làn: nhỏ nhất ~**30 cm**, thực tế ~**50 cm**.
- Camera cao **30 cm** trên mặt đường.
- Xe là **mô hình** → tham khảo phương pháp, không sao chép hằng số xe thật.

## ROI 0.45 đang lấy cả trời
Quan sát được: `ROI_TOP_FRAC = 0.45` → hàng 108; `ROI_BOTTOM_FRAC = 0.88` → hàng 211.

Giả sử camera USB rộng ~70° ở khung 320x240 → `vf ≈ 228`, chân trời `cy = 120`.
Quan hệ khoảng cách: `d(y) = h * vf / (y - cy)`.

| Hàng | y=108 | y=120 | y=211 | y=240 |
|---|---|---|---|---|
| Khoảng cách | **vô hạn (trời)** | chân trời | 0.75 m | 0.57 m |

⇒ hàng 108 nằm **trên** đường chân trời. Cửa sổ trên cùng đang nhìn trời.

## 5 cửa sổ phủ quãng đường lệch 19 lần
`d(y) = 68.4 / (y - 120)` (h = 0.30 m, vf = 228)

| Cửa sổ | Hàng | Quãng đường | Bề rộng 50 cm hiện ra |
|---|---|---|---|
| 0 (đáy) | 192–211 | 0.20 m | 134 px |
| 1 | 174–192 | 0.32 m | 90 px |
| 2 | 156–174 | 0.63 m | 55 px |
| 3 | 138–156 | 1.90 m | 30 px |
| 4 (trên) | 120–138 | **≥ 3.80 m** | **≤ 19 px** |

Cửa sổ 4 có 2 vạch cách nhau < `MIN_LANE_GAP = 30` ⇒ **luôn bị loại**. Đây là lý do
thấy khung đỏ ở trên `/lane/vis`.

## Vì sao bỏ IPM được
IPM (`getPerspectiveTransform` + `warpPerspective`) để đo cm là cách nặng: thêm 2 điểm
hiệu chỉnh, đổi toàn bộ hệ toạ độ, và phải đổi đơn vị `dev` trên firmware.

Công thức pinhole cho đúng kết quả đó bằng **một biểu thức**:
`W_m = w_px * h / (y - cy)`.

Kiểm chứng: y=211, cy=120, h=0.30, w_px=134 → W = 134 × 0.30 / 91 = **0.442 m**.
Khớp với dự đoán 0.50 m ở 0.85 m phía trước.

## `cv::fitPoly` không tồn tại
`CV_VERSION_MAJOR 4 / MINOR 11` trong OpenCV local. `fitPoly` chỉ có từ 5.x.
⇒ tự viết bằng hệ 3 phương trình bình phương nhỏ nhất + `cv::solve(DECOMP_SVD)`.

## CLAHE có sẵn, CMake không cần sửa
`cv::createCLAHE` nằm trong `opencv2/imgproc.hpp`. `src/camera_node/CMakeLists.txt` dùng
`find_package(OpenCV REQUIRED)` không giới hạn COMPONENTS ⇒ mọi module đều có.

## Tham khảo (không sao chép nguyên xi)
Các tài liệu dưới đây đã bị xoá khỏi repo, chỉ còn ghi chú kết luận rút ra:
- Tài liệu "Advanced Lane Keeping" (Gaussian smoothing trước edge detect; ROI để tránh
  nhận nhầm cột trụ; trung tuyến = trung điểp 2 mép).
- Repo `MichiMaestre/Lane-Detection-for-Autonomous-Cars` — Sobel + ROI mask + Hough +
  chia trái/phải theo slope + least squares + vanishing point.
- Notebook `P4 - Advanced Lane Lines.ipynb` của nhóm `graduation_project-master` —
  `pers_transform`, `window_search` 9 cửa sổ margin 100 minpix 50,
  `Line(maxSamples=4)`, `validate_lane_update` 5 cổng, đo mét bằng
  `ym_per_pix`/`xm_per_pix`.

Hằng số trong notebook là cho xe thật 1280x720 (làn 3.7 m, nhìn 30 m) — không dùng được
cho xe mô hình, nên chỉ mượn **thứ tự các bước**.