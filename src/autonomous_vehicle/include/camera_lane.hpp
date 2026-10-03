#pragma once

// ============================================================================
// CAMERA LANE
// ============================================================================

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>

#include "lane_mask.hpp"

// ============================================================================
// CAMERA OUTPUT
// ============================================================================

// Mức độ tin cậy của kết quả detect. Node điều khiển dùng để giảm tốc thay
// vì tin tuyệt đối: LOW thì chỉ chạy chậm, NONE thì dừng.
enum class LaneConfidence : uint8_t {
    NONE = 0,
    WEAK,
    GOOD,
    STRONG
};

// Tên hiển thị. Cố tình KHÔNG đặt tên là to_string để không đụng độ với
// std::to_string ở bất kỳ file nào dùng `using namespace std`.
inline const char* lane_confidence_name(LaneConfidence c) {
    switch (c) {
        case LaneConfidence::NONE: return "NONE";
        case LaneConfidence::WEAK: return "WEAK";
        case LaneConfidence::GOOD: return "GOOD";
        case LaneConfidence::STRONG: return "STRONG";
    }
    return "NONE";
}

struct LaneOutput {
    std::vector<cv::Point> left;
    std::vector<cv::Point> right;
    std::vector<cv::Point> center;

    std::string camera_cmd = "STOP";

    int16_t dev_final_px = 0;
    uint8_t target_speed_x10 = 0;
    bool valid = false;

    cv::Scalar lane_color_bgr = {0, 0, 0};
    std::string lane_color_name = "black";

    int pixels_used = 0;
    cv::Mat vis;
    cv::Mat raw;

    // Mask nhị phân và bird view để tuner/offline tool dùng lại, không phải
    // chạy lại toàn bộ detector.
    cv::Mat mask;
    cv::Mat bird;

    float curve_angle_deg = 0.0f;
    float curvature = 0.0f;
    uint8_t speed_factor = 0;

    // Hàng đầu tiên của ROI thật sự dùng ở frame này ( tính trên ảnh gốc).
    // Node dashboard cắt ảnh theo giá trị này để hiện đúng vùng mà detector
    // thật sự nhìn vào, thay vì vẽ lại từ hằng số ROI.
    int roi_y0 = 0;

    uint64_t frame_id = 0;
    float processing_ms = 0.0f;

    // --- Tín hiệu chẩn đoán ---
    // 0 = nhánh dự phòng Canny/scanline, 1 = nhánh chính IPM.
    // Giữ 2 chỗ dự phòng cho detector CNN: 2 = CNN, 3 = CNN không đủ tin cậy
    // nên fallback sang OpenCV.
    uint8_t detector_mode = 0;
    bool is_dual_lane = false;
    LaneConfidence confidence = LaneConfidence::NONE;

    // Tuổi dữ liệu tính lúc get_latest(): vượt STALE_AGE_MS thì stale = true.
    uint32_t age_ms = 0;
    bool stale = true;
};

// ============================================================================
// CAMERA LANE
//
// THUẬT TOÁN
//   1. lane_mask::build_lane_mask()  -> mask nhị phân tự thích nghi sáng
//   2. IPM (bird's-eye) 320x240
//   3. Cột tích phân (col integral) -> histogram cửa sổ O(W) thay vì O(N)
//   4. Sliding window 9 cửa sổ -> điểm lane
//   5. Polyfit bậc 2, lọc ngoại lệ 2 lượt
//   6. Chiếu ngược về ảnh gốc
//   7. Median filter + EMA trên hệ số
//
// SO VỚI BẢN CŨ (git HEAD)
//   - Ngưỡng kênh R tĩnh 0..110 -> phân vị 5/95% (tự thích nghi sáng)
//   - AND với Sobel >= 30 -> chỉ giữ mép vạch -> nay giữ cả thân vạch
//   - peak Sobel >= 60 tuyệt đối -> >= 0.16 * p99 (tương đối)
//   - MAX_BLOB_WIDTH_PX = 260 cố định -> theo bề rộng làn ước lượng
//   - findNonZero + quét toàn bộ 36 lần/cửa sổ -> col integral O(1)/cửa sổ
//   - fit_poly dùng Mat::at -> mảng thô + giải trực tiếp
//   - Không có EMA -> rung lái
// ============================================================================

class CameraLane {

public:

    static constexpr int FRAME_W = 640;
    static constexpr int FRAME_H = 400;

    // Cơ chế Latest-Frame (không tích luỹ độ trễ)
    static constexpr size_t FRAME_BUFFER_COUNT = 2;

    static constexpr bool ENABLE_VISUALIZATION = true;
    // Tô mask lên ảnh để kiểm chứng ngưỡng. Tốn ~1.5 ms/frame nên mặc định
    // tắt; bật lên khi camera không nhận được làn. Nếu thấy gần như toàn
    // ảnh bị tô xanh thì ngưỡng đang chọn nhầm nền, không phải lỗi IPM.
    static constexpr bool ENABLE_MASK_OVERLAY = false;
    static constexpr bool ENABLE_DEBUG_LOG = false;
    static constexpr int DEBUG_LOG_EVERY = 30;
    static constexpr bool ENABLE_LANE_COLOR_SAMPLE = true;

    // Ngưỡng tuổi dữ liệu. Vượt ngưỡng -> node điều khiển dừng xe.
    static constexpr uint32_t STALE_AGE_MS = 200;

    // ------------------------------------------------------------------------
    // ROI THEO TỐC ĐỘ (hệ số theo chiều cao khung hình)
    // ------------------------------------------------------------------------
    static constexpr float ROI_FACTOR_SLOW = 0.70f;
    static constexpr float ROI_FACTOR_FAST_MIN = 0.55f;
    static constexpr float ROI_SPEED_START = 4.0f;
    static constexpr float ROI_SPEED_GAIN = 0.02f;
    static constexpr float SINGLE_ROI_FACTOR = 0.85f;

    // ------------------------------------------------------------------------
    // NHÁNH DỰ PHÒNG: CANNY + SCANLINE
    // ------------------------------------------------------------------------
    static constexpr int CANNY_LOW = 25;
    static constexpr int CANNY_HIGH = 90;
    static constexpr int BLUR_KERNEL = 5;
    static constexpr int SCAN_STEP = 4;
    static constexpr int MIN_PRELIM_POINTS = 15;
    static constexpr int MIN_COMMON_POINTS = 10;
    static constexpr int MIN_FINAL_POINTS = 5;

    // ------------------------------------------------------------------------
    // HÌNH HỌC LÀN (px tính theo FRAME_W)
    // ------------------------------------------------------------------------
    static constexpr float MIN_LANE_WIDTH_PX = 200.0f;
    static constexpr float MAX_LANE_WIDTH_PX = 500.0f;
    static constexpr float DEFAULT_LANE_WIDTH_PX = 230.0f;
    static constexpr float ALPHA_LANE_WIDTH = 0.20f;

    // ------------------------------------------------------------------------
    // BIRD'S-EYE (IPM)
    // Hình học chóp lấy từ bản gốc (960x540 -> tỉ lệ toàn khung).
    // ------------------------------------------------------------------------
    static constexpr int BIRD_W = 320;
    static constexpr int BIRD_H = 240;

    static constexpr float IPM_TL_X = 0.4479f;
    static constexpr float IPM_TR_X = 0.5521f;
    static constexpr float IPM_BL_X = 0.1146f;
    static constexpr float IPM_BR_X = 0.9271f;

    // ------------------------------------------------------------------------
    // CỬA SỔ TRƯỢT + FIT BẬC HAI (đơn vị px ảnh bird's-eye)
    // ------------------------------------------------------------------------
    static constexpr int N_WINDOWS = 9;
    static constexpr int WINDOW_MARGIN = 50;
    static constexpr int WINDOW_MIN_POINTS = 6;
    static constexpr int WINDOW_MAX_SHIFT = 30;
    static constexpr float WINDOW_RUN_RATIO = 0.25f;

    static constexpr float BIRD_HIST_TOP_FRAC = 0.50f;
    static constexpr float BIRD_MIN_COLUMN_RATIO = 0.15f;
    static constexpr float BIRD_VALLEY_RATIO = 0.60f;
    // Khoảng cách trọng tâm khối lane so với giữa bird view để coi là "hai vạch
    // đã dính". Rộng hơn 0.06 vì khi phòng tối nền kem dính vào vạch, khối
    // lane bị phình và trọng tâm lệch khá xa mức tuyệt đối 19px.
    static constexpr float BIRD_SIDE_TOLERANCE = 0.10f;
    static constexpr int BIRD_MIN_HALF_WIDTH = 20;
    static constexpr int BIRD_XMID_JUMP_MAX = 60;

    static constexpr float BIRD_MIN_LANE_WIDTH = 40.0f;
    static constexpr float BIRD_MAX_LANE_WIDTH = 300.0f;
    static constexpr float BIRD_MAX_FIT_RESIDUAL_PX = 10.0f;
    static constexpr int BIRD_MIN_FIT_POINTS = 5;
    static constexpr int BIRD_MIN_FIT_PIXELS = 60;

    // Bề rộng đo được luôn thấp hơn bề rộng thật (đo trong vùng cửa sổ),
    // nên kiểm tra theo tỉ lệ so với ước lượng thay vì ngưỡng tuyệt đối.
    static constexpr float IPM_MIN_WIDTH_RATIO = 0.60f;
    static constexpr float IPM_MAX_WIDTH_RATIO = 1.45f;

    // Bootstrap bề rộng làn khi chưa có frame nào đo được. Bản cũ để
    // bird_lane_width_ = 0 -> half_w = 20 -> 2 seed dồn vào 40px -> 2 làn
    // dính -> hàm abort -> KHÔNG BAO GIỜ hồi phục (deadlock khởi động).
    static constexpr float BIRD_BOOTSTRAP_WIDTH_FRAC = 0.45f;

    static constexpr float BIRD_SAMPLE_TOP_FRAC = 0.25f;
    static constexpr int BIRD_SAMPLE_STEP = 4;

    static constexpr int VIEW_BIRD_EYE = 0;
    static constexpr int VIEW_NORMAL = 1;

    // Băng làn tô trong ảnh gốc
    static constexpr int BIRD_LANE_BAND_MARGIN = 33;
    static constexpr float BIRD_LANE_BAND_ALPHA = 0.30f;

    // Bước lấy mẫu chung cho hai làn trong ảnh gốc
    static constexpr int IMAGE_SAMPLE_STEP = 4;

    // ------------------------------------------------------------------------
    // LỌC NGOẠI LỆ TRƯỚC KHI FIT
    // Bản cũ fit trên toàn bộ pixel thu được rồi mới kiểm residual trên 9
    // điểm trung tâm. Một cụm nhiễu nhỏ trong mask kéo cả đường fit lệch.
    // Nay loại ngoại lệ trước khi fit rồi mới kiểm residual.
    // ------------------------------------------------------------------------
    static constexpr float OUTLIER_MIN_RESIDUAL_PX = 2.5f;
    static constexpr float OUTLIER_SIGMA_K = 1.5f;

    // ------------------------------------------------------------------------
    // BÁNH LÁ
    // ------------------------------------------------------------------------
    static constexpr int FWD_THRESHOLD_PX = 15;

    static constexpr float SLOPE_OFFSET_START = 0.80f;
    static constexpr float SLOPE_OFFSET_END = 1.00f;
    static constexpr float SLOPE_OFFSET_MIN_PX = 50.0f;
    static constexpr float SLOPE_OFFSET_MAX_PX = 120.0f;

    // ------------------------------------------------------------------------
    // SPEED PLANNER
    // ------------------------------------------------------------------------
    static constexpr uint8_t SPEED_STRAIGHT_X10 = 85;
    static constexpr uint8_t SPEED_CURVE_X10 = 60;
    static constexpr uint8_t SPEED_SHARP_X10 = 45;

    static constexpr float ALPHA_TARGET_SPEED = 0.70f;

    static constexpr float CURVE_ENTER_SLOPE = 0.65f;
    static constexpr float CURVE_EXIT_SLOPE = 0.55f;
    static constexpr float SHARP_ENTER_SLOPE = 0.85f;
    static constexpr float SHARP_EXIT_SLOPE = 0.75f;

    static constexpr int CURVE_ENTER_DEV_PX = 50;
    static constexpr int CURVE_EXIT_DEV_PX = 40;
    static constexpr int SHARP_ENTER_DEV_PX = 80;
    static constexpr int SHARP_EXIT_DEV_PX = 65;

    static constexpr int MEDIAN_TAIL_POINTS = 5;
    static constexpr int MEDIAN_TAIL_POINTS_MAX = 16;

    enum class SpeedState : uint8_t {
        STRAIGHT,
        CURVE,
        SHARP
    };

    explicit CameraLane(
        int device_index = 0,
        int fps = 30,
        bool use_v4l2 = true
    );

    ~CameraLane();

    bool start();
    void stop();

    // Ghi đè tham số mask trước khi start(). Dùng cho tuner và để chỉnh
    // không cần sửa lại code.
    void set_mask_options(const lane_mask::Options& opts);
    lane_mask::Options mask_options() const;

    // Ghi đè ROI theo tỉ lệ chiều cao ảnh (0..1, tính từ mép trên).
    //
    // Quan trọng: mép TRÊN ảnh là tầm xa, mép DƯỚI là gần nhất. Với camera
    // nghiêng xuống, khoảng cách là hàm siêu tuyến theo hàng ảnh - vài
    // chục phần trăm hàng dưới cùng chỉ che vài chục cm. Đáy ảnh ở đây là
    // nền cách bumper 30-40cm, nên hằng số ROI cố định rất dễ chọn sai.
    // Cho phép chỉnh bằng tham số ROS để tìm ngay trên xe.
    void set_roi_factors(float dual, float single);

    bool get_latest(
        LaneOutput& out,
        float current_speed_kmh = 0.0f
    );

    size_t buffered_frames() const;

private:

    void capture_thread();
    void process_thread();

    void detect_lanes(
        const cv::Mat& bgr,
        float current_speed_kmh,
        LaneOutput& out
    );

    // Bản sao tham số mask an toàn để đọc từ process thread.
    lane_mask::Options opts() const;

    // --- Nhánh IPM ---
    bool update_ipm(int mask_w, int mask_h);
    bool detect_lanes_ipm(
        const cv::Mat& mask_work,
        int y0_dual,
        int y0_single,
        int y1,
        float width_scale,
        bool& is_dual_lane,
        LaneOutput& out
    );

    // Cột tích phân: col_sum[y][x] = số pixel khác 0 trong cột x từ hàng 0
    // tới hàng y. Cho phép histogram của một dải hàng bằng 2 phép trừ.
    void build_col_integral(const cv::Mat& bin);

    bool window_search(float& center_x, bool& dual);
    bool search_window(
        int y_low,
        int y_high,
        int y_mid,
        int side,
        int& seed
    );

    void build_curve_points(
        bool fit_ok,
        const float coef[3],
        int y0,
        int y1,
        std::vector<cv::Point>& out
    );
    static void resample_curve(
        const std::vector<cv::Point>& src,
        int y0_used,
        int y1,
        std::vector<cv::Point>& out
    );

    // Fit x = a.y^2 + b.y + c trên toạ độ chuẩn hoá y / BIRD_H.
    // Lọc ngoại lệ 2 lượt trước khi giải, trả residual trung bình của bộ
    // điểm đã lọc.
    static bool fit_poly(
        const std::vector<cv::Point>& pts,
        float coef[3]
    );
    static float poly_eval(const float coef[3], float y);
    static bool fit_residual_ok(
        const std::vector<cv::Point>& pts,
        const float coef[3]
    );
    bool fit_poly_filtered(
        const std::vector<cv::Point>& pts,
        const std::vector<cv::Point>& win_means,
        float coef[3],
        float& residual_out
    );

    // --- Nhánh dự phòng ---
    bool detect_lanes_scanline(
        const cv::Mat& roi,
        float width_scale,
        int y0_dual,
        int y0_single,
        int y1,
        bool& is_dual_lane
    );

    void draw_lane_band(cv::Mat& vis) const;

    static float calculate_slope(
        const std::vector<cv::Point>& lane
    );

    static int median_tail_x(
        const std::vector<cv::Point>& pts,
        int tail_count
    );

    static int median_value(
        const std::vector<int>& values
    );

    static void median_smooth(
        std::vector<cv::Point>& pts
    );

    static int16_t clamp_int16(
        int value
    );

    static float clamp_float(
        float value,
        float min_value,
        float max_value
    );

    static int calculate_slope_offset(
        float dominant_slope
    );

    uint8_t calculate_target_speed(
        float dominant_slope,
        int abs_deviation
    );

    static LaneConfidence grade_confidence(
        bool has_left,
        bool has_right,
        bool dual,
        size_t fit_points,
        float residual,
        bool ema_stable
    );

    // Hồi quy từ mảng thô, không cấp phát. Dùng cho bậc 2 đơn giản hoá.
    static bool solve3x3(
        const double A[9],
        const double b[3],
        double x[3]
    );

    int device_index_;
    int fps_;
    bool use_v4l2_;

    cv::VideoCapture cap_;

    std::atomic<bool> running_{false};
    std::thread capture_thread_;
    std::thread process_thread_;

    struct FrameSlot {
        cv::Mat image;
        uint64_t frame_id = 0;
    };

    std::array<FrameSlot, FRAME_BUFFER_COUNT> frame_pool_{};

    int latest_slot_ = -1;
    int processing_slot_ = -1;

    mutable std::mutex frame_mtx_;
    std::condition_variable frame_ready_cv_;
    std::condition_variable frame_free_cv_;

    mutable std::mutex output_mtx_;
    LaneOutput latest_output_{};
    bool output_ready_ = false;
    // Mốc thời gian steady_clock của frame mới nhất đã sẵn sàng đọc.
    std::chrono::steady_clock::time_point latest_output_time_{};

    std::atomic<float> current_speed_kmh_{0.0f};

    // --- Tham số mask (ghi trước start, đọc trong process thread) ---
    lane_mask::Options mask_opts_{};
    lane_mask::Result mask_result_{};

    // Kích thước khung hình thực tế của camera và kích thước ảnh làm việc.
    // Mọi phép chiếu ngược IPM đều phải đi qua 2 giá trị này, nếu không điểm
    // lane sẽ nằm trong góc trên-trái của ảnh gốc.
    int frame_w_ = FRAME_W;
    int frame_h_ = FRAME_H;

    int xmid_scan_ = FRAME_W / 2;
    float roi_factor_dual_ = ROI_FACTOR_SLOW;
    float roi_factor_single_ = SINGLE_ROI_FACTOR;
    float lane_width_est_px_ = DEFAULT_LANE_WIDTH_PX;
    float target_speed_x10_filtered_ = static_cast<float>(SPEED_STRAIGHT_X10);
    bool target_speed_initialized_ = false;
    SpeedState speed_state_ = SpeedState::STRAIGHT;

    // --- Buffer tái sử dụng ---
    cv::Mat gray_;        // ảnh xám dải ROI (nhánh dự phòng)
    cv::Mat blur_;
    cv::Mat edges_;
    cv::Mat mask_;        // mask nhị phân ở kích thước làm việc
    cv::Mat bird_;        // mask sau IPM, 320x240
    cv::Mat labels_;
    cv::Mat stats_;
    cv::Mat centroids_;
    cv::Mat m_ipm_;
    cv::Mat m_ipm_inv_;
    cv::Size bird_size_ = cv::Size(BIRD_W, BIRD_H);
    int ipm_w_ = 0;
    int ipm_h_ = 0;

    cv::Mat morph_kernel_;

    float bird_lane_width_ = 0.0f;
    float bird_center_prev_ = static_cast<float>(BIRD_W) * 0.5f;
    bool bird_valid_ = false;
    // 2 vạch dính thành một khối ở giữa bird view: vẫn trả kết quả một làn
    // ước lượng thay vì huỷ toàn bộ nhánh IPM như bản cũ.
    bool bird_merged_hint_ = false;

    // Cột tích phân (BIRD_H+1) x BIRD_W, uint32 để không tràn khi cột dày.
    std::vector<uint32_t> col_integral_;

    // Hệ số fit thô của frame này và hệ số đã EMA (dùng để vẽ).
    float left_fit_raw_[3] = {0.0f, 0.0f, 0.0f};
    float right_fit_raw_[3] = {0.0f, 0.0f, 0.0f};
    float left_fit_[3] = {0.0f, 0.0f, 0.0f};
    float right_fit_[3] = {0.0f, 0.0f, 0.0f};
    bool left_fit_init_ = false;
    bool right_fit_init_ = false;

    std::vector<int> bird_col_sum_;
    std::vector<int> bird_win_sum_;

    std::vector<cv::Point> leftPts_prelim_;
    std::vector<cv::Point> rightPts_prelim_;
    std::vector<cv::Point> leftPts_;
    std::vector<cv::Point> rightPts_;
    std::vector<int> prelim_centers_;

    // 9 trung tâm cửa sổ: dùng để kiểm residual của đường fit
    std::vector<cv::Point> left_win_means_;
    std::vector<cv::Point> right_win_means_;

    // Toàn bộ pixel lane thu được trong cửa sổ: đầu vào polyfit
    std::vector<cv::Point> left_fit_px_;
    std::vector<cv::Point> right_fit_px_;

    std::vector<cv::Point> left_raw_;
    std::vector<cv::Point> right_raw_;

    // Buffer tái sử dụng cho chiếu ngược IPM (tránh cấp phát mỗi frame).
    std::vector<cv::Point2f> bird_pts_buf_;
    std::vector<cv::Point2f> roi_pts_buf_;
    std::vector<cv::Point2f> bird_bottom_buf_;
    std::vector<cv::Point2f> roi_bottom_buf_;

    // Bộ điểm tạm cho lượt lọc ngoại lệ.
    std::vector<cv::Point> filtered_;

    LaneOutput work_output_{};
    uint64_t next_frame_id_ = 0;
    uint64_t debug_frame_count_ = 0;
};
