#pragma once

#include <opencv2/opencv.hpp>

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// ============================================================================
// CAMERA LANE - Nhận diện làn đường bằng cửa sổ trượt (Tích hợp Fallback 1 làn)
// ============================================================================

// Trạng thái bám làn
enum class LaneState { 
    LOST = 0, 
    ONE_LINE = 1, 
    TWO_LINES = 2 
};

class CameraLane {
public:
    // ------------------------------------------------------------------------
    // Kich thuoc xu ly
    // ------------------------------------------------------------------------
    static constexpr int WORK_W = 320;
    static constexpr int WORK_H = 240;

    // ------------------------------------------------------------------------
    // Tham so cua so truot
    // ------------------------------------------------------------------------
    static constexpr int N_WINDOWS = 5;
    static constexpr int WINDOW_MARGIN = 30;

    // Đã giảm xuống 2 theo yêu cầu (để đếm pixel đúng khi chia 255)
    static constexpr int WINDOW_MIN_POINTS = 2;
    static constexpr int MAX_RUN_WIDTH = 25;
    static constexpr int MIN_LANE_GAP = 30;
    static constexpr int LANE_WIDTH_MIN = 50;
    static constexpr int LANE_WIDTH_MAX = 230;
    static constexpr int MIN_MATCHED_WINDOWS = 3;
    static constexpr int MAX_SEED_SEARCH = 3;
    
    // Tham số cho Fallback 1 vạch & Giữ giá trị (Hold Frames)
    static constexpr int MIN_ONE_SIDE_WINDOWS = 3;
    static constexpr int HOLD_FRAMES = 15;        // ≈ 0,5 s @30 fps

    // ------------------------------------------------------------------------
    // Tien xu ly anh
    // ------------------------------------------------------------------------
    static constexpr int BLUR_KSIZE = 5;
    static constexpr double CLAHE_CLIP = 2.0;
    static constexpr int CLAHE_TILE = 8;

    // ------------------------------------------------------------------------
    // Hinh hoc de do centimet
    // ------------------------------------------------------------------------
    static constexpr float CAMERA_HEIGHT_M = 0.30f;
    static constexpr int HORIZON_Y = 120;
    static constexpr float LOOKAHEAD_FRAC = 0.58f;

    // ------------------------------------------------------------------------
    // Vung lam viec
    // ------------------------------------------------------------------------
    static constexpr float ROI_TOP_FRAC = 0.58f;
    
    // Đã hạ xuống 0.93 (để chừa khoảng ±0.15m hai bên làn)
    static constexpr float ROI_BOTTOM_FRAC = 0.93f;

    // ------------------------------------------------------------------------
    // Loc va do tre
    // ------------------------------------------------------------------------
    static constexpr float EMA_ALPHA = 0.35f; // Có thể tăng lên 0.6 nếu xe vọt ở cua
    static constexpr int STALE_AGE_MS = 200;
    static constexpr int MAX_READ_FAIL = 25;

    // ------------------------------------------------------------------------
    // Ket qua mot lan nhan
    // ------------------------------------------------------------------------
    struct LaneOutput {
        std::vector<cv::Point> left_pts;
        std::vector<cv::Point> right_pts;

        bool two_lanes = false;
        LaneState state = LaneState::LOST; // Cập nhật trạng thái làn

        int dev_px = 0;
        float dev_cm = 0.0f;
        float lane_width_cm = 0.0f;
        
        // Chỉ báo độ cong (khác 0 khi vào cua -> dùng để giảm tốc)
        float curve_px = 0.0f;

        bool fit_ok = false;
        unsigned long age_ms = 0;
        bool stale = true;
        cv::Mat vis;
        double proc_ms = 0.0;
        long frame_id = 0;
    };

    explicit CameraLane(int camera_index = -1, int target_fps = 30);
    ~CameraLane();

    CameraLane(const CameraLane&) = delete;
    CameraLane& operator=(const CameraLane&) = delete;

    bool start();
    void stop();
    bool is_running() const;

    void get_latest(LaneOutput& out, bool copy_vis = false) const;
    void set_roi_top_frac(float frac);

private:
    bool open_camera();
    void capture_loop();
    bool detect(const cv::Mat& frame, LaneOutput& out);

    static bool seed_pair_from(const cv::Mat& col_sum, int& seed_left, int& seed_right);
    static bool peak_in(const cv::Mat& col_sum, int seed, int& peak);
    static bool fit_poly2(const std::vector<cv::Point>& pts, double coef[3], int& y0, int& y1);
    static float px_to_cm(float px, int y);

    // ------------------------------------------------------------------------
    // Trang thai
    // ------------------------------------------------------------------------
    int camera_index_;
    int target_fps_;

    std::atomic<float> roi_top_frac_{ROI_TOP_FRAC};
    cv::VideoCapture cap_;
    std::atomic<bool> running_{false};
    std::thread worker_;
    mutable std::mutex mtx_;

    LaneOutput latest_;

    int dev_ema_ = 0;
    
    // Biến lưu trữ phục vụ Fallback 1 làn và Hold Frames
    float lane_w_m_ = 0.55f;
    int lost_frames_ = 0;

    cv::Ptr<cv::CLAHE> clahe_;
    long frame_id_ = 0;
    std::chrono::steady_clock::time_point last_frame_time_{};
};