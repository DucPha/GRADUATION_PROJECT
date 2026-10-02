#pragma once

#include <opencv2/opencv.hpp>

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// ============================================================================
// LANE OUTPUT
// ============================================================================

struct LaneOutput {
    std::vector<cv::Point> left;
    std::vector<cv::Point> right;
    std::vector<cv::Point> center;

    int16_t dev_final_px = 0;
    std::string camera_cmd = "FWD";
    uint8_t target_speed_x10 = 0;

    bool valid = false;
    bool is_dual_lane = false;

    int pixels_used = 0;
    float curve_angle_deg = 0.0f;
    float curvature = 0.0f;
    uint8_t speed_factor = 0;
    float lane_width_est_px = 0.0f;

    uint64_t frame_id = 0;
    float processing_ms = 0.0f;
    int held_frames = 0;

    cv::Mat vis;
    cv::Mat raw;
    cv::Mat binary_mask;
    cv::Mat bird_mask;

    cv::Scalar lane_color_bgr{0, 0, 0};
    std::string lane_color_name = "black";

    uint8_t detector_mode = 0;
};

// ============================================================================
// CAMERA LANE
// ============================================================================

class CameraLane {
public:
    // ------------------------------------------------------------------------
    // HỆ TOẠ ĐỘ THAM CHIẾU (DESIGN SPACE)
    // ------------------------------------------------------------------------
    // MọI hằng số tính bằng PIXEL bên dưới (ngưỡng hình học làn, bán kính
    // blob, sai số lái...) đều được viết cho KHUNG 640x480. Khi camera chạy ở
    // độ phân giải khác, chúng được nhân với px_scale_ = W_thuc_te/640.
    //
    // LÝ DO PHẢI TÁCH HAI KHUNG NÀY: camera USB 2.0 "PC CAMERA" chỉ chạy được
    // ~6 FPS ở 640x480 (vượt băng thông isochronous USB 2.0, thậm chí rớt
    // luôn khỏi bus USB), nhưng chạy 63 FPS ổn định ở 352x288. Nếu hardcode
    // luôn 640x480 thì vừa chậm vừa mất ổn định; nếu hardcode luôn 352x288 thì
    // phải sửa lại hàng chục ngưỡng theo tỉ lệ 0.55 - dễ sai sót.
    // Tách design space + px_scale_ giữ được cả hai: đổi độ phân giải chỉ cần
    // đổi CAP_W/CAP_H, mọi ngưỡng tự co giãn theo.
    // ------------------------------------------------------------------------
    static constexpr int FRAME_W = 640;
    static constexpr int FRAME_H = 480;

    // ------------------------------------------------------------------------
    // ĐỘ PHÂN GIẢI THỰC TẾ YÊU CẦU TỪ CAMERA
    // ------------------------------------------------------------------------
    // Camera này chỉ hỗ trợ: 640x480, 352x288, 320x240, 176x144, 160x120 (YUYV).
    // Đo thực tế trên chính máy này:
    //     640x480 -> ~6 FPS, hay rớt USB (VIDIOC_REQBUFS errno=19)
    //     352x288 -> 63 FPS, 0 lỗi trong 30 giây
    //     320x240 -> ~5 FPS
    // Nên 352x288 là LỚN NHẤT chạy được, và dư sức cho 30 FPS mục tiêu.
    static constexpr int CAP_W = 352;
    static constexpr int CAP_H = 288;

    // Các độ phân giải dự phòng theo thứ tự thử (px_scale_ tự tính lại).
    struct Resolution {
        int w;
        int h;
    };
    static constexpr Resolution RES_FALLBACKS[] = {
        {352, 288},
        {320, 240},
        {176, 144},
    };
    static constexpr int RES_FALLBACK_COUNT =
        static_cast<int>(sizeof(RES_FALLBACKS) / sizeof(RES_FALLBACKS[0]));

    static constexpr bool ENABLE_VISUALIZATION = true;
    static constexpr bool ENABLE_DEBUG_LOG = false;
    static constexpr int DEBUG_LOG_EVERY = 15;
    static constexpr bool ENABLE_BIRD_DEBUG = false;
    static constexpr bool ENABLE_LANE_COLOR_SAMPLE = true;

    // ROI ADAPTIVE
    static constexpr float ROI_FACTOR_SLOW = 0.55f;
    static constexpr float ROI_FACTOR_FAST_MIN = 0.40f;
    static constexpr float ROI_SPEED_START = 3.0f;
    static constexpr float ROI_SPEED_GAIN = 0.025f;
    static constexpr float SINGLE_ROI_FACTOR = 0.70f;

    // BLACK LANE THRESHOLDS
    static constexpr int THRESH_BLACK = 70;
    static constexpr int CANNY_LOW = 25;
    static constexpr int CANNY_HIGH = 100;
    static constexpr int BLUR_KERNEL = 5;
    static constexpr int SCAN_STEP = 4;
    static constexpr int MIN_PRELIM_POINTS = 6;
    static constexpr int MIN_COMMON_POINTS = 5;
    static constexpr int MIN_FINAL_POINTS = 5;

    // LANE GEOMETRY
    static constexpr float MIN_LANE_WIDTH_PX = 160.0f;
    static constexpr float MAX_LANE_WIDTH_PX = 480.0f;
    static constexpr float DEFAULT_LANE_WIDTH_PX = 280.0f;
    static constexpr float ALPHA_LANE_WIDTH = 0.12f;

    // FILTERING (Chống chói đèn trần & khử ron gạch)
    static constexpr int SOBEL_MIN_THRESHOLD = 12;
    static constexpr int SOBEL_MAX_THRESHOLD = 255;
    static constexpr float SOBEL_X_PEAK_MIN = 20.0f;
    static constexpr int LANE_MIN_VALUE = 0;
    static constexpr int LANE_MAX_VALUE = 145;        // Chấp nhận vạch đen bị bóng đèn chiếu xám

    static constexpr float CLAHE_CLIP_LIMIT = 1.2f;   // Hạ xuống 1.2 để không làm đậm ron gạch
    static constexpr int CLAHE_GRID_X = 8;
    static constexpr int CLAHE_GRID_Y = 8;

    static constexpr bool BONNET_ENABLE = false;
    static constexpr float BONNET_X0 = 0.25f;
    static constexpr float BONNET_X1 = 0.75f;
    static constexpr float BONNET_Y0 = 0.55f;

    static constexpr int ADAPTIVE_BLOCK = 31;
    static constexpr int ADAPTIVE_C = 6;

    // Bộ lọc hình thái học cân bằng
    static constexpr int MASK_CLOSE_X = 5;
    static constexpr int MASK_CLOSE_KERNEL = 15;
    static constexpr int MASK_OPEN_KERNEL = 3;        // Giữ 3 để không gọt đứt vạch thật
    static constexpr int MIN_BLOB_AREA = 30;
    static constexpr int MIN_BLOB_HEIGHT = 8;
    static constexpr float MAX_BLOB_WIDTH_PX = 250.0f;

    // IPM BIRD'S-EYE
    static constexpr int BIRD_W = 320;
    static constexpr int BIRD_H = 240;

    static constexpr float IPM_TL_X = 0.38f;
    static constexpr float IPM_TR_X = 0.62f;
    static constexpr float IPM_BL_X = 0.05f;
    static constexpr float IPM_BR_X = 0.95f;

    static constexpr int N_WINDOWS = 9;
    static constexpr int WINDOW_MARGIN = 45;
    static constexpr int WINDOW_MIN_POINTS = 6;
    static constexpr int WINDOW_MAX_SHIFT = 25;
    static constexpr float WINDOW_RUN_RATIO = 0.20f;

    static constexpr float BIRD_HIST_TOP_FRAC = 0.45f;
    static constexpr float BIRD_MIN_COLUMN_RATIO = 0.12f;
    static constexpr float BIRD_VALLEY_RATIO = 0.55f;
    static constexpr float BIRD_SIDE_TOLERANCE = 0.05f;
    static constexpr int BIRD_MIN_HALF_WIDTH = 18;
    static constexpr int BIRD_XMID_JUMP_MAX = 50;

    static constexpr float BIRD_MIN_LANE_WIDTH = 35.0f;
    static constexpr float BIRD_MAX_LANE_WIDTH = 280.0f;
    static constexpr float BIRD_MAX_FIT_RESIDUAL_PX = 8.0f;
    static constexpr int BIRD_MIN_FIT_POINTS = 4;
    static constexpr int BIRD_MIN_FIT_PIXELS = 50;

    static constexpr float IPM_MIN_WIDTH_RATIO = 0.55f;
    static constexpr float IPM_MAX_WIDTH_RATIO = 1.50f;

    static constexpr float BIRD_SAMPLE_TOP_FRAC = 0.20f;
    static constexpr int BIRD_SAMPLE_STEP = 3;

    static constexpr int VIEW_BIRD_EYE = 0;
    static constexpr int VIEW_NORMAL = 1;

    static constexpr int BIRD_LANE_BAND_MARGIN = 28;
    static constexpr float BIRD_LANE_BAND_ALPHA = 0.25f;
    static constexpr int IMAGE_SAMPLE_STEP = 3;

    // CONTROL & STEERING
    static constexpr int FWD_THRESHOLD_PX = 12;

    static constexpr float SLOPE_OFFSET_START = 0.70f;
    static constexpr float SLOPE_OFFSET_END = 0.95f;
    static constexpr float SLOPE_OFFSET_MIN_PX = 30.0f;
    static constexpr float SLOPE_OFFSET_MAX_PX = 70.0f;

    static constexpr uint8_t SPEED_STRAIGHT_X10 = 60;
    static constexpr uint8_t SPEED_CURVE_X10 = 50;
    static constexpr uint8_t SPEED_SHARP_X10 = 40;

    static constexpr float ALPHA_TARGET_SPEED = 0.35f;

    static constexpr float CURVE_ENTER_SLOPE = 0.55f;
    static constexpr float CURVE_EXIT_SLOPE = 0.40f;
    static constexpr float SHARP_ENTER_SLOPE = 0.75f;
    static constexpr float SHARP_EXIT_SLOPE = 0.60f;

    static constexpr int CURVE_ENTER_DEV_PX = 55;
    static constexpr int CURVE_EXIT_DEV_PX = 35;
    static constexpr int SHARP_ENTER_DEV_PX = 80;
    static constexpr int SHARP_EXIT_DEV_PX = 60;

    static constexpr int MEDIAN_TAIL_POINTS = 5;
    static constexpr int MEDIAN_TAIL_POINTS_MAX = 16;

    static constexpr int MAX_LOST_FRAMES = 4;

    enum class SpeedState : uint8_t {
        STRAIGHT,
        CURVE,
        SHARP
    };

    // device_index = -1 nghĩa là TỰ DÒ. Bắt buộc phải dùng auto với camera
    // này: khi USB rớt và cắm lại, kernel cấp lại số /dev/video khác (video0
    // -> video1 -> video0 ...). Chỉ định cứng index sẽ mở nhầm hoặc mở hỏng.
    //
    // use_v4l2=true: bắt buộc, camera chỉ hỗ trợ V4L2.
    // auto_exposure/exposure: giữ nguyên cho tương thích, nhưng camera USB
    // này KHÔNG có control exposure_auto/exposure_absolute nên chúng không có
    // tác dụng (đã kiểm tra bằng v4l2-ctl --list-ctrls).
    explicit CameraLane(
        int device_index = -1,
        int fps = 30,
        bool use_v4l2 = true,
        bool auto_exposure = true,
        int exposure = -4
    );

    ~CameraLane();

    bool start();
    void stop();

    bool get_latest(
        LaneOutput& out,
        float current_speed_kmh = 0.0f
    );

    size_t buffered_frames() const;

    // Độ phân giải thực tế đang chạy (sau khi start()).
    int actual_width() const { return actual_w_; }
    int actual_height() const { return actual_h_; }
    double measured_fps() const { return actual_measured_fps_; }
    // Lý do camera không mở được (rỗng = không có lỗi).
    std::string last_error() const { return last_error_; }

private:
    void capture_thread();
    void process_thread();

    // Mở camera theo 1 cấu hình cụ thể (dùng cho cả start lẫn reconnect).
    bool open_device(int index, int want_w, int want_h);

    // Dò /dev/videoN tìm đúng camera UVC đang cắm. Trả về index, -1 nếu không
    // thấy. Phải probe thật (mở + đọc 1 frame) vì /dev/video1 của camera này
    // là node capture rỗng - mở được nhưng không có format nào.
    static int detect_device_index(int preferred_index);

    // Nhân một hằng số tính theo pixel của design space (640x480) sang khung
    // thực tế đang chạy. Mọi ngưỡng hình học viết trong không gian 640x480 đều
    // phải đi qua đây, nếu không sẽ sai khi đổi CAP_W/CAP_H.
    static int scale_px(float design_px, float scale);
    static float scale_pxf(float design_px, float scale);

    // Dựng lại các kernel morphology theo px_scale_ hiện tại. Gọi mỗi lần
    // open_device() thành công vì kích thước kernel phụ thuộc độ phân giải.
    void build_scaled_kernels();

    // Số lần liên tiếp capture thất bại; dùng để quyết định khi nào dò lại
    // /dev/video (camera hay rớt khỏi USB rồi cắm lại với index khác).
    std::atomic<int> capture_fail_count_{0};

    void detect_lanes(
        const cv::Mat& bgr,
        float current_speed_kmh,
        LaneOutput& out
    );

    bool sobel_color_thresholding(const cv::Mat& bgr);
    static void mask_bonnet(cv::Mat& mask);
    bool filter_lane_blobs(const cv::Mat& gray_src);

    bool transforming_view(const cv::Mat& src, int flag, cv::Mat& dst);
    bool update_ipm(int w, int h);
    bool detect_lanes_ipm(
        const cv::Mat& full_frame,
        int y0_dual,
        int y0_single,
        int y1,
        float width_scale,
        bool& is_dual_lane,
        cv::Mat& out_bird
    );

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
        int frame_w,
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

    static bool fit_poly(
        const std::vector<cv::Point>& pts,
        float coef[3]
    );
    static float poly_eval(const float coef[3], float y);
    static bool fit_residual_ok(
        const std::vector<cv::Point>& pts,
        const float coef[3]
    );

    bool detect_lanes_scanline(
        const cv::Mat& roi,
        float width_scale,
        int y0_dual,
        int y0_single,
        int y1,
        bool& is_dual_lane
    );

    void draw_lane_band(cv::Mat& vis) const;

    static float calculate_slope(const std::vector<cv::Point>& lane);
    static int median_tail_x(const std::vector<cv::Point>& pts, int tail_count);
    static int median_value(const std::vector<int>& values);
    static void median_smooth(std::vector<cv::Point>& pts);
    static int16_t clamp_int16(int value);
    static float clamp_float(float value, float min_value, float max_value);
    // offset trả về tính bằng pixel THỰC TẾ nên cần px_scale_ làm tham số.
    int calculate_slope_offset(float dominant_slope);
    uint8_t calculate_target_speed(float dominant_slope, int abs_deviation);

    int device_index_;
    int fps_;
    bool use_v4l2_;
    bool auto_exposure_;
    int exposure_;
    double actual_measured_fps_ = 0.0;

    // Trạng thái thật của luồng sau khi mở thành công.
    int actual_w_ = 0;
    int actual_h_ = 0;
    // Khung thực tế / design space. Mọi ngưỡng pixel nhân với hệ số này.
    float px_scale_ = 1.0f;
    std::string last_error_;

    cv::VideoCapture cap_;

    std::atomic<bool> running_{false};
    std::thread capture_thread_;
    std::thread process_thread_;

    // Triple-Buffer Synchronization (Chống Data Race với GUI)
    mutable std::mutex frame_mtx_;
    std::condition_variable frame_ready_cv_;
    cv::Mat shared_frame_;
    uint64_t shared_frame_id_ = 0;
    bool new_frame_available_ = false;

    mutable std::mutex output_mtx_;
    LaneOutput latest_output_{};
    bool output_ready_ = false;

    std::atomic<float> current_speed_kmh_{0.0f};

    int xmid_scan_ = FRAME_W / 2;
    float lane_width_est_px_ = DEFAULT_LANE_WIDTH_PX;
    float target_speed_x10_filtered_ = static_cast<float>(SPEED_STRAIGHT_X10);
    bool target_speed_initialized_ = false;
    SpeedState speed_state_ = SpeedState::STRAIGHT;

    int lost_frame_count_ = 0;
    int16_t last_valid_dev_ = 0;
    float last_valid_slope_ = 0.0f;

    cv::Ptr<cv::CLAHE> clahe_;
    cv::Mat gray_;
    cv::Mat gray_full_;
    cv::Mat blur_;
    cv::Mat edges_;
    cv::Mat eq_;
    cv::Mat sobel_x_;
    cv::Mat mask_;
    cv::Mat mask_tmp_;
    cv::Mat bird_;
    cv::Mat labels_;
    cv::Mat stats_;
    cv::Mat centroids_;
    cv::Mat m_ipm_;
    cv::Mat m_ipm_inv_;
    cv::Size bird_size_ = cv::Size(BIRD_W, BIRD_H);
    int ipm_w_ = 0;
    int ipm_h_ = 0;

    int bird_hood_y_ = BIRD_H;
    int bird_valid_rows_ = BIRD_H;
    static constexpr int BIRD_MIN_VALID_ROWS = 60;

    std::array<cv::Mat, 3> bgr_channels_{};
    std::vector<uint8_t> blob_keep_lut_;

    cv::Mat morph_kernel_;
    cv::Mat mask_close_kernel_;
    cv::Mat mask_open_kernel_;

    float bird_lane_width_ = 0.0f;
    float bird_center_prev_ = static_cast<float>(BIRD_W) * 0.5f;
    bool bird_valid_ = false;
    float left_fit_[3] = {0.0f, 0.0f, 0.0f};
    float right_fit_[3] = {0.0f, 0.0f, 0.0f};

    std::vector<int> bird_col_sum_;
    std::vector<int> bird_win_sum_;
    float track_run_w_[2] = {0.0f, 0.0f};

    std::vector<cv::Point> leftPts_prelim_;
    std::vector<cv::Point> rightPts_prelim_;
    std::vector<cv::Point> leftPts_;
    std::vector<cv::Point> rightPts_;
    std::vector<int> prelim_centers_;

    std::vector<cv::Point> left_win_means_;
    std::vector<cv::Point> right_win_means_;
    std::vector<cv::Point> left_fit_px_;
    std::vector<cv::Point> right_fit_px_;

    std::vector<cv::Point> left_raw_;
    std::vector<cv::Point> right_raw_;
    std::vector<cv::Point> bird_nonzero_;

    LaneOutput work_output_{};
    uint64_t next_frame_id_ = 0;
    uint64_t debug_frame_count_ = 0;
};