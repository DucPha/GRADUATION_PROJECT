#pragma once

// ============================================================================
// LANE MASK  --  Tách riêng khỏi CameraLane để:
//   1) dùng lại được cho detector CNN (node Python hoặc C++ khác)
//   2) chạy offline trong tools/lane_tuner.py mà không cần camera/ROS
//   3) giữ đúng một bản logic ngưỡng, không phân tán hai chỗ
//
// VẤN ĐỀ GIẢI QUYẾT
// Bản cũ dùng ngưỡng tuyệt đối trên kênh R: inRange(R, 0, 110). Trong phòng
// tối, toàn bộ nền gạch kem cũng rơi vào 0..110 -> mask bão hoà thành một khối
// trắng duy nhất rộng hơn MAX_BLOB_WIDTH_PX -> mọi blob bị lo -> mất 2 làn.
// Bản mới không dùng ngưỡng tuyệt đối: lấy ngưỡng từ phân vị 5/95% của chính
// ảnh đó, nên tự thích nghi mọi mức ánh sáng phòng và không cần can thiệp.
//
// GIAO DIỆN
//   build_lane_mask(bgr_full, opts, mask_small, debug) -> bool
//     mask_small : CV_8UC1, kích thước MASK_W x MASK_H (đã resize về đây)
//     debug      : có thể rỗng; nếu khác rỗng sẽ nhận ảnh BGR để hiện thị
// ============================================================================

#include <opencv2/opencv.hpp>

#include <string>
#include <vector>

namespace lane_mask {

// ---------------------------------------------------------------------------
// Tham số. Toàn bộ nằm trong struct để tuner ghi đè được không cần sửa code.
// ---------------------------------------------------------------------------

struct Options {
    // --- Kích thước làm việc ---
    // Mask được tính ở kích thước nhỏ hơn ảnh gốc. Ở 320x200 thay vì 640x400
    // giảm ~4x số pixel cho toàn bộ chuỗi CLAHE/ngưỡng/Sobel, đồng thời giảm
    // nhiễu sensor khi ảnh tối (nhiễu cảm biến tỉ lệ nghịch với số pixel).
    int work_w = 320;
    int work_h = 200;

    // --- Cân bằng tương phản ---
    // CLAHE áp trên ảnh BGR 3 kênh một lượt thay vì tách kênh rồi CLAHE riêng.
    double clahe_clip = 3.0;
    int clahe_grid_x = 8;
    int clahe_grid_y = 8;

    // --- Ngưỡng thích nghi theo phân vị ---
    // p_lo / p_hi là phân vị của kênh L trong ảnh sau CLAHE.
    // Ngưỡng vạch TỐI  = p_lo + dark_band * (p_hi - p_lo)
    // Ngưỡng vạch SÁNG = p_hi - light_band * (p_hi - p_lo)
    // p_lo/p_hi loại ngoại trừ 5% tối nhất / sáng nhất nên đèn LED trắng và bóng
    // tối không kéo ngưỡng đi.
    double p_low = 0.05;
    double p_high = 0.95;
    double dark_band = 0.32;
    double light_band = 0.32;

    // Bật/tắt nhánh vạch sáng (đường tối có vạch trắng).
    //
    // MẶC ĐỊNH = false vì xe chạy trong phòng: vạch đen trên nền gạch kem.
    // Bật nhánh này thì threshold(l_channel, light_thr, BINARY) chọn trúng
    // chính nền kem (L rất cao) -> mask thành một khối trắng gần hết ảnh ->
    // connected components gộp hết thành 1 blob -> blob đó rộng hơn
    // max_blob_width nên bị lo -> KHÔNG còn làn nào. Đây là nguyên nhân khả
    // dĩ nhất khiến camera không nhận được làn trong phòng.
    // Chỉ bật lại khi chạy ngoài đường tối có vạch trắng.
    bool enable_light_lanes = false;

    // --- Chống nhiễu ---
    // Ngưỡng trên trục L không đủ, vạch phải có biên sắc. Điểm này kiểm
    // SobelX trên ảnh CLAHE, nhưng NGƯỠNG SO SÁNH TƯƠNG ĐỐI với p99 của
    // chính ảnh, không phải hằng số 60 như bản cũ (60 là nguyên nhân mất
    // toàn bộ blob trong phòng thiếu sáng).
    bool require_edge = true;
    double edge_peak_ratio = 0.16;

    int close_w = 5;
    int close_h = 15;
    int open_k = 3;

    // --- Lọc blob ---
    // Nhỏ hơn bản cũ vì mask đang ở 320x200: một vạch rộng ~20px ở 640x400
    // chỉ còn ~5px ở 320x200, và đoạn vạch xa chỉ dài vài chục hàng. Ngưỡng
    // diện tích cũ loại mất đúng các đoạn đó trước khi kịp nối bằng
    // MORPH_CLOSE.
    int min_area = 12;
    int min_height = 6;
    // Bề rộng blob tối đa tính theo bề rộng làn ước lượng, không cố định.
    // Bản cũ dùng MAX_BLOB_WIDTH_PX = 260 trong khi bề rộng làn thật ~230px
    // cộng thêm bề rộng vạch và độ nghiêng do MORPH_CLOSE -> blob hợp lệ rơi
    // đúng ngay ngưỡng loại.
    float max_blob_width_ratio = 1.9f;
    int max_blob_width_min = 40;
    int max_blob_width_max = 420;

    // --- Che nắp xe ---
    // Camera đặt trước xe, góc nhìn thấy nền từ 30-40cm phía trước bumper:
    // đáy ảnh là nền đất trống, KHÔNG có nắp/thân xe trong khung hình.
    // Bản cũ bật che tại x∈[31%,62%], y>=65% -> xoá đúng vùng giữa ảnh gần
    // xe, nơi hai làn vừa tách ra và đáng lẽ phải dễ nhận nhất.
    bool bonnet_enable = false;
    // Giữ tham số để bật lại được khi đổi camera/tháp cao hơn có che nắp.
    double bonnet_x0 = 0.3125;
    double bonnet_x1 = 0.6250;
    double bonnet_y0 = 0.6481;

    // --- Theo dõi thời gian (EMA) ---
    // Rung của hệ số fit là nguồn rung lái chính. EMA hai tốc độ: khi ảnh tốt
    // thì phản hồi nhanh, khi ảnh yếu thì mạnh hơn để không bị nhiễu.
    double ema_alpha_strong = 0.55;
    double ema_alpha_weak = 0.25;

    // Ngưỡng dưới để coi là "ảnh yếu" (độ khớp cạnh thấp), so với edge_quality.
    double weak_edge_ratio = 0.30;
};

struct Result {
    // Kênh L sau CLAHE, kích thước work_w x work_h.
    cv::Mat l_channel;
    // SobelX đã chuẩn hoá về 0..255 (uint8) để dùng cho minMaxLoc nhanh.
    cv::Mat sobel_x;

    // Ngưỡng thực tế đã tính ở frame này (hữu ích khi hiệu chỉnh).
    int dark_threshold = 0;
    int light_threshold = 0;
    // p99 của SobelX -> mốc cho edge_peak_ratio.
    double sobel_p99 = 0.0;
    // sobel_p99 / 255: chất lượng tương phản cạnh của frame, dùng để quyết
    // định EMA mạnh hay nhẹ. Tương đối nên không phụ thuộc ánh sáng phòng.
    double edge_quality = 0.0;
    // Số blob còn lại sau lọc.
    int kept_blobs = 0;
    // mean/stddev của kênh L, để chẩn đoán chất lượng ảnh.
    double luma_mean = 0.0;
    double luma_stddev = 0.0;
};

// Hàm chính. mask_out phải là CV_8UC1 rỗng hoặc đúng kích thước work.
bool build_lane_mask(
    const cv::Mat& bgr_full,
    const Options& opts,
    cv::Mat& mask_out,
    Result* debug = nullptr
);

// ---------------------------------------------------------------------------
// Tiện ích dùng chung cho tuner / detector CNN
// ---------------------------------------------------------------------------

// Phân vị theo histogram, O(1) không cần sort.
void percentiles_from_histogram(
    const cv::Mat& gray_u8,
    double p_low,
    double p_high,
    int& out_low,
    int& out_high
);

// Che vùng nắp xe theo tỉ lệ toàn khung, ghi trực tiếp 0 vào mask.
void mask_bonnet(cv::Mat& mask, double x0, double x1, double y0);

}  // namespace lane_mask
