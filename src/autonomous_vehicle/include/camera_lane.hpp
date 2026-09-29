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
// CAMERA OUTPUT
// ============================================================================

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

    float curve_angle_deg = 0.0f;
    float curvature = 0.0f;
    uint8_t speed_factor = 0;

    uint64_t frame_id = 0;
    float processing_ms = 0.0f;
};

// ============================================================================
// CAMERA LANE
// ============================================================================

class CameraLane {

public:

    static constexpr int FRAME_W = 640;
    static constexpr int FRAME_H = 400;

    static constexpr int ROI_W = 320;
    static constexpr int ROI_H = 200;

    // Cơ chế Latest-Frame (Zero Latency Accumulation)
    static constexpr size_t FRAME_BUFFER_COUNT = 2;

    static constexpr bool ENABLE_VISUALIZATION = false;
    static constexpr bool ENABLE_DEBUG_LOG = false;

    // Ngưỡng Canny tối ưu riêng cho vạch làn màu đen
    static constexpr int CANNY_LOW = 30;
    static constexpr int CANNY_HIGH = 120;
    static constexpr int BLUR_KERNEL = 5;
    static constexpr int SCAN_STEP = 4;
    static constexpr int MEDIAN_TAIL_POINTS = 5;

    static constexpr float ROI_FACTOR_SLOW = 0.70f;
    static constexpr float ROI_FACTOR_FAST_MIN = 0.55f;
    static constexpr float ROI_SPEED_START = 4.0f;
    static constexpr float ROI_SPEED_GAIN = 0.02f;
    static constexpr float SINGLE_ROI_FACTOR = 0.85f;

    static constexpr int MIN_PRELIM_POINTS = 15;
    static constexpr int MIN_FINAL_POINTS = 5;
    static constexpr int MIN_COMMON_POINTS = 10;

    static constexpr float MIN_LANE_WIDTH_PX = 200.0f;
    static constexpr float MAX_LANE_WIDTH_PX = 500.0f;
    static constexpr float DEFAULT_LANE_WIDTH_PX = 230.0f;
    static constexpr float ALPHA_LANE_WIDTH = 0.20f;

    static constexpr int FWD_THRESHOLD_PX = 15;

    static constexpr float SLOPE_OFFSET_START = 0.80f;
    static constexpr float SLOPE_OFFSET_END = 1.00f;
    static constexpr float SLOPE_OFFSET_MIN_PX = 50.0f;
    static constexpr float SLOPE_OFFSET_MAX_PX = 120.0f;

    // ------------------------------------------------------------------------
    // SPEED PLANNER (TỐC ĐỘ CAO - BỐC LỬA)
    // Đường thẳng -> 8.5 km/h (85)
    // Cua vừa    -> 6.0 km/h (60)
    // Cua gắt    -> 4.5 km/h (45)
    // ------------------------------------------------------------------------
    static constexpr uint8_t SPEED_STRAIGHT_X10 = 85;
    static constexpr uint8_t SPEED_CURVE_X10 = 60;
    static constexpr uint8_t SPEED_SHARP_X10 = 45;

    static constexpr float SPEED_CURVE_SLOPE = 0.65f;
    static constexpr float SPEED_SHARP_SLOPE = 0.85f;
    static constexpr float ALPHA_TARGET_SPEED = 0.70f;

    // Hysteresis chống chập chờn tốc độ khi qua lại giữa các khúc cua
    static constexpr float CURVE_ENTER_SLOPE = 0.65f;
    static constexpr float CURVE_EXIT_SLOPE = 0.55f;
    static constexpr float SHARP_ENTER_SLOPE = 0.85f;
    static constexpr float SHARP_EXIT_SLOPE = 0.75f;

    static constexpr int CURVE_ENTER_DEV_PX = 50;
    static constexpr int CURVE_EXIT_DEV_PX = 40;
    static constexpr int SHARP_ENTER_DEV_PX = 80;
    static constexpr int SHARP_EXIT_DEV_PX = 65;

    enum class SpeedState : uint8_t {
        STRAIGHT,
        CURVE,
        SHARP
    };

    explicit CameraLane(
        int device_index = 0,
        int fps = 120,
        bool use_v4l2 = true
    );

    ~CameraLane();

    bool start();
    void stop();

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

    static float calculate_slope(
        const std::vector<cv::Point>& lane
    );

    static int median_tail_x(
        const std::vector<cv::Point>& pts,
        int tail_count
    );

    static int median_value(
        std::vector<int>& values
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

    std::atomic<float> current_speed_kmh_{0.0f};

    int xmid_scan_ = FRAME_W / 2;
    float lane_width_est_px_ = DEFAULT_LANE_WIDTH_PX;
    float target_speed_x10_filtered_ = static_cast<float>(SPEED_STRAIGHT_X10);
    bool target_speed_initialized_ = false;
    SpeedState speed_state_ = SpeedState::STRAIGHT;

    cv::Mat gray_;
    cv::Mat blur_;
    cv::Mat edges_;
    cv::Mat morph_kernel_;

    std::vector<cv::Point> leftPts_prelim_;
    std::vector<cv::Point> rightPts_prelim_;
    std::vector<cv::Point> leftPts_;
    std::vector<cv::Point> rightPts_;
    std::vector<int> prelim_centers_;

    LaneOutput work_output_{};
    uint64_t next_frame_id_ = 0;
};