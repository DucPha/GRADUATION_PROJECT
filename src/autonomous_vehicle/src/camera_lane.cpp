#include "camera_lane.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>

// V4L2 control cho exposure. Toàn khối biên dịch bị loại trên nền tảng không
// có kernel headers, nên build không phụ thuộc videodev2.h.
#if defined(__linux__) && defined(__has_include)
#if __has_include(<linux/videodev2.h>)
#define CAMERA_LANE_HAS_V4L2_CTRL 1
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/videodev2.h>
#if __has_include(<linux/v4l2-ctrls.h>)
#include <linux/v4l2-ctrls.h>
#endif
#include <cerrno>
#endif
#endif

using namespace cv;

namespace {

#ifdef CAMERA_LANE_HAS_V4L2_CTRL

// Mở /dev/videoN để gửi lệnh control. OpenCV không lộ fd của backend V4L2.
int v4l2_open(int device_index) {
    char path[64];
    std::snprintf(path, sizeof(path), "/dev/video%d", device_index);
    return ::open(path, O_RDWR | O_NONBLOCK);
}

// Đọc giá trị lớn nhất của một control. Trả false nếu camera không có control
// đó, đang bị disable, hoặc ioctl lỗi.
bool v4l2_ctrl_max(int device_index, unsigned int cid, int& max_out) {
    const int fd = v4l2_open(device_index);
    if (fd < 0) return false;

    v4l2_queryctrl query{};
    query.id = cid;

    const bool ok =
        ::ioctl(fd, VIDIOC_QUERYCTRL, &query) == 0 &&
        (query.flags & V4L2_CTRL_FLAG_DISABLED) == 0;
    if (ok) max_out = static_cast<int>(query.maximum);

    ::close(fd);
    return ok;
}

// Gửi VIDIOC_S_CTRL. Trả false nếu driver từ chối (control không tồn tại,
// nằm ngoài dải, hoặc camera đang bị chiếm bởi tiến trình khác).
bool v4l2_set_ctrl(int device_index, unsigned int cid, int value) {
    const int fd = v4l2_open(device_index);
    if (fd < 0) return false;

    v4l2_control control{};
    control.id = cid;
    control.value = value;

    const bool ok = ::ioctl(fd, VIDIOC_S_CTRL, &control) == 0;
    ::close(fd);
    return ok;
}

#endif  // CAMERA_LANE_HAS_V4L2_CTRL

}  // namespace

// ============================================================================
// CONSTRUCTOR / DESTRUCTOR
// ============================================================================

CameraLane::CameraLane(
    int device_index,
    int fps,
    bool use_v4l2
)
    : device_index_(device_index),
      fps_(fps),
      use_v4l2_(use_v4l2),
      xmid_scan_(FRAME_W / 2) {

    leftPts_prelim_.reserve(128);
    rightPts_prelim_.reserve(128);
    leftPts_.reserve(128);
    rightPts_.reserve(128);
    prelim_centers_.reserve(128);

    left_win_means_.reserve(N_WINDOWS);
    right_win_means_.reserve(N_WINDOWS);
    left_fit_px_.reserve(8192);
    right_fit_px_.reserve(8192);
    left_raw_.reserve(256);
    right_raw_.reserve(256);
    filtered_.reserve(8192);

    const size_t bird_samples = static_cast<size_t>(
        (BIRD_H - static_cast<int>(static_cast<float>(BIRD_H) * BIRD_SAMPLE_TOP_FRAC)) /
        BIRD_SAMPLE_STEP + 1);
    bird_pts_buf_.reserve(bird_samples);
    roi_pts_buf_.reserve(bird_samples);
    bird_bottom_buf_.reserve(2);
    roi_bottom_buf_.reserve(2);

    bird_col_sum_.assign(BIRD_W, 0);
    bird_win_sum_.assign(BIRD_W, 0);

    // Cột tích phân: (BIRD_H + 1) hàng x BIRD_W cột
    col_integral_.assign(
        static_cast<size_t>(BIRD_H + 1) * static_cast<size_t>(BIRD_W), 0);

    work_output_.left.reserve(128);
    work_output_.right.reserve(128);
    work_output_.center.reserve(128);

    latest_output_.left.reserve(128);
    latest_output_.right.reserve(128);
    latest_output_.center.reserve(128);

    morph_kernel_ = getStructuringElement(MORPH_RECT, Size(3, 3));
}

CameraLane::~CameraLane() {
    stop();
}

void CameraLane::set_mask_options(const lane_mask::Options& opts) {
    std::lock_guard<std::mutex> lock(output_mtx_);
    mask_opts_ = opts;
}

lane_mask::Options CameraLane::mask_options() const {
    std::lock_guard<std::mutex> lock(output_mtx_);
    return mask_opts_;
}

lane_mask::Options CameraLane::opts() const {
    return mask_options();
}

void CameraLane::set_roi_factors(float dual, float single) {
    // Chặn trong (0,1): 0 sẽ kéo ROI lên sát đáy (không còn dữ liệu), >=1 sẽ
    // lấy tràn ra ngoài ảnh.
    roi_factor_dual_ = std::max(0.05f, std::min(dual, 0.95f));
    roi_factor_single_ = std::max(0.05f, std::min(single, 0.95f));
}

// ============================================================================
// START
// ============================================================================

bool CameraLane::start() {
    if (running_)
        return true;

    std::cout << "[CameraLane] Opening camera index " << device_index_ << "...\n";

    const int backend = use_v4l2_ ? CAP_V4L2 : CAP_ANY;

    if (!cap_.open(device_index_, backend)) {
        std::cerr << "[CameraLane] ERROR: Cannot open camera\n";
        return false;
    }

    cap_.set(CAP_PROP_FRAME_WIDTH, FRAME_W);
    cap_.set(CAP_PROP_FRAME_HEIGHT, FRAME_H);
    cap_.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap_.set(CAP_PROP_FPS, fps_);
    cap_.set(CAP_PROP_BUFFERSIZE, 1);

    // Khoá auto-exposure nếu camera hỗ trợ.
    //
    // Đây là một trong hai nguyên nhân gốc khiến bản cũ hỏng trong phòng:
    // auto-exposure mặc định làm độ sáng trôi theo đèn, trong khi ngưỡng mask
    // của bản cũ là hằng số tuyệt đối -> khi phòng tối, toàn bộ nền gạch kem
    // cũng lọt vào dải "vạch tối" -> mask thành một khối trắng duy nhất ->
    // mọi blob bị lo.
    //
    // OpenCV không lộ file descriptor của backend V4L2, nên phải mở thêm
    // /dev/videoN để gửi VIDIOC_S_CTRL. Với driver không hỗ trợ (USB rời,
    // driver CSI) thì im lặng bỏ qua: detector vẫn chạy được nhờ mask thích
    // nghi theo phân vị.
    const int buffer_w = static_cast<int>(
        std::lround(cap_.get(CAP_PROP_FRAME_WIDTH)));
    const int buffer_h = static_cast<int>(
        std::lround(cap_.get(CAP_PROP_FRAME_HEIGHT)));

    if (buffer_w < 100 || buffer_h < 100) {
        std::cerr << "[CameraLane] ERROR: camera returned " << buffer_w << "x"
                  << buffer_h << "\n";
        cap_.release();
        return false;
    }

#ifdef CAMERA_LANE_HAS_V4L2_CTRL
    {
        // V4L2_EXPOSURE_AUTO (=0) BẬT auto-exposure, nên để khoá tay phải ghi
        // V4L2_EXPOSURE_MANUAL (=1). Ghi sai giá trị thì lệnh set
        // V4L2_CID_EXPOSURE_ABSOLUTE ngay dưới đây bị driver bỏ qua.
        bool exposure_locked =
            v4l2_set_ctrl(device_index_, V4L2_CID_EXPOSURE_AUTO,
                          V4L2_EXPOSURE_MANUAL);

        int exposure_max = 0;
        if (exposure_locked &&
            v4l2_ctrl_max(device_index_, V4L2_CID_EXPOSURE_ABSOLUTE,
                          exposure_max) && exposure_max > 8) {
            // Khoảng 1/8 giá trị lớn nhất: đủ sáng để vạch đen khác biệt với
            // nền kem nhưng chưa đủ để nền bị cháy và rơi xuống dải tối.
            v4l2_set_ctrl(device_index_, V4L2_CID_EXPOSURE_ABSOLUTE,
                          std::max(1, exposure_max / 8));
        }

        std::cout << "[CameraLane] V4L2 exposure "
                  << (exposure_locked ? "locked (manual)" : "not supported")
                  << "\n";
    }
#endif

    const double actual_fps = cap_.get(CAP_PROP_FPS);

    std::cout << "[CameraLane] " << buffer_w << "x" << buffer_h << " @ " << actual_fps
              << " FPS | mask " << mask_opts_.work_w << "x" << mask_opts_.work_h
              << " | auto-level p" << static_cast<int>(mask_opts_.p_low * 100)
              << "/p" << static_cast<int>(mask_opts_.p_high * 100) << "\n";

    {
        std::lock_guard<std::mutex> lock(frame_mtx_);
        latest_slot_ = -1;
        processing_slot_ = -1;

        for (size_t i = 0; i < FRAME_BUFFER_COUNT; ++i) {
            frame_pool_[i].image.create(buffer_h, buffer_w, CV_8UC3);
            frame_pool_[i].frame_id = 0;
        }
    }

    {
        std::lock_guard<std::mutex> lock(output_mtx_);
        latest_output_ = LaneOutput{};
        latest_output_.valid = false;
        latest_output_.stale = true;
        output_ready_ = false;
        latest_output_time_ = std::chrono::steady_clock::now();
    }

    leftPts_prelim_.clear();
    rightPts_prelim_.clear();
    leftPts_.clear();
    rightPts_.clear();
    prelim_centers_.clear();
    left_win_means_.clear();
    right_win_means_.clear();
    left_fit_px_.clear();
    right_fit_px_.clear();
    filtered_.clear();
    left_raw_.clear();
    right_raw_.clear();

    gray_.release();
    blur_.release();
    edges_.release();
    mask_.release();
    bird_.release();
    m_ipm_.release();
    m_ipm_inv_.release();
    ipm_w_ = 0;
    ipm_h_ = 0;

    xmid_scan_ = buffer_w / 2;
    lane_width_est_px_ = DEFAULT_LANE_WIDTH_PX *
        (static_cast<float>(buffer_w) / static_cast<float>(FRAME_W));
    bird_lane_width_ = 0.0f;
    bird_center_prev_ = static_cast<float>(BIRD_W) * 0.5f;
    bird_valid_ = false;
    left_fit_init_ = false;
    right_fit_init_ = false;
    target_speed_x10_filtered_ = static_cast<float>(SPEED_STRAIGHT_X10);
    target_speed_initialized_ = false;
    speed_state_ = SpeedState::STRAIGHT;
    current_speed_kmh_.store(0.0f, std::memory_order_relaxed);
    next_frame_id_ = 0;
    debug_frame_count_ = 0;

    running_ = true;
    capture_thread_ = std::thread(&CameraLane::capture_thread, this);
    process_thread_ = std::thread(&CameraLane::process_thread, this);

    return true;
}

// ============================================================================
// STOP
// ============================================================================

void CameraLane::stop() {
    if (!running_) {
        if (capture_thread_.joinable()) capture_thread_.join();
        if (process_thread_.joinable()) process_thread_.join();
        if (cap_.isOpened()) cap_.release();
        return;
    }

    running_ = false;
    frame_ready_cv_.notify_all();
    frame_free_cv_.notify_all();

    if (capture_thread_.joinable()) capture_thread_.join();

    frame_ready_cv_.notify_all();
    if (process_thread_.joinable()) process_thread_.join();

    if (cap_.isOpened()) cap_.release();

    {
        std::lock_guard<std::mutex> lock(frame_mtx_);
        latest_slot_ = -1;
        processing_slot_ = -1;
    }

    std::cout << "[CameraLane] Stopped.\n";
}

// ============================================================================
// GET LATEST
// ============================================================================

bool CameraLane::get_latest(LaneOutput& out, float current_speed_kmh) {
    if (!std::isfinite(current_speed_kmh)) current_speed_kmh = 0.0f;
    current_speed_kmh = clamp_float(current_speed_kmh, 0.0f, 20.0f);
    current_speed_kmh_.store(current_speed_kmh, std::memory_order_relaxed);

    std::lock_guard<std::mutex> lock(output_mtx_);
    if (!output_ready_) return false;

    out = latest_output_;

    // Tính tuổi ngay tại đây, trong khi vẫn giữ lock, để node điều khiển
    // nhận được con số khớp với dữ liệu vừa lấy.
    const auto age = std::chrono::steady_clock::now() - latest_output_time_;
    auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(age).count();
    if (age_ms < 0) age_ms = 0;

    out.age_ms = static_cast<uint32_t>(age_ms);
    out.stale = (age_ms > static_cast<long long>(STALE_AGE_MS));

    return true;
}

size_t CameraLane::buffered_frames() const {
    std::lock_guard<std::mutex> lock(frame_mtx_);
    return latest_slot_ >= 0 ? 1u : 0u;
}

// ============================================================================
// CAPTURE THREAD (LATEST-FRAME POLICY)
// ============================================================================

void CameraLane::capture_thread() {
    while (running_) {
        int slot_index = -1;

        {
            std::unique_lock<std::mutex> lock(frame_mtx_);
            frame_free_cv_.wait(lock, [this]() {
                if (!running_) return true;
                for (size_t i = 0; i < FRAME_BUFFER_COUNT; ++i) {
                    const int index = static_cast<int>(i);
                    if (index != processing_slot_ && index != latest_slot_) {
                        return true;
                    }
                }
                return false;
            });

            if (!running_) break;

            for (size_t i = 0; i < FRAME_BUFFER_COUNT; ++i) {
                const int index = static_cast<int>(i);
                if (index != processing_slot_ && index != latest_slot_) {
                    slot_index = index;
                    break;
                }
            }

            if (slot_index < 0) continue;
        }

        cv::Mat& frame = frame_pool_[slot_index].image;
        if (!cap_.read(frame) || frame.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        const uint64_t frame_id = ++next_frame_id_;

        {
            std::lock_guard<std::mutex> lock(frame_mtx_);
            if (!running_) continue;

            if (latest_slot_ >= 0) {
                latest_slot_ = -1;
            }

            frame_pool_[slot_index].frame_id = frame_id;
            latest_slot_ = slot_index;
        }

        frame_ready_cv_.notify_one();
        frame_free_cv_.notify_one();
    }
}

// ============================================================================
// PROCESS THREAD
// ============================================================================

void CameraLane::process_thread() {
    while (true) {
        int slot_index = -1;

        {
            std::unique_lock<std::mutex> lock(frame_mtx_);
            frame_ready_cv_.wait(lock, [this]() {
                return !running_ || latest_slot_ >= 0;
            });

            if (latest_slot_ < 0) {
                if (!running_) break;
                continue;
            }

            slot_index = latest_slot_;
            latest_slot_ = -1;
            processing_slot_ = slot_index;
        }

        cv::Mat& frame = frame_pool_[slot_index].image;
        const uint64_t frame_id = frame_pool_[slot_index].frame_id;
        const float current_speed = current_speed_kmh_.load(std::memory_order_relaxed);

        const auto start_time = std::chrono::steady_clock::now();
        detect_lanes(frame, current_speed, work_output_);
        const auto end_time = std::chrono::steady_clock::now();

        work_output_.frame_id = frame_id;
        work_output_.processing_ms =
            std::chrono::duration<float, std::milli>(end_time - start_time).count();
        work_output_.age_ms = 0;
        work_output_.stale = false;

        {
            std::lock_guard<std::mutex> lock(output_mtx_);
            latest_output_ = work_output_;
            latest_output_time_ = std::chrono::steady_clock::now();
            output_ready_ = true;
        }

        {
            std::lock_guard<std::mutex> lock(frame_mtx_);
            processing_slot_ = -1;
        }

        frame_free_cv_.notify_one();
    }
}

// ============================================================================
// UTILITIES
// ============================================================================

float CameraLane::clamp_float(float value, float min_value, float max_value) {
    return std::max(min_value, std::min(value, max_value));
}

int16_t CameraLane::clamp_int16(int value) {
    if (value < -32768) return -32768;
    if (value > 32767) return 32767;
    return static_cast<int16_t>(value);
}

void CameraLane::median_smooth(std::vector<cv::Point>& pts) {
    if (pts.size() < 3) return;
    for (size_t i = 1; i + 1 < pts.size(); ++i) {
        const int a = pts[i - 1].x;
        const int b = pts[i].x;
        const int c = pts[i + 1].x;
        pts[i].x = std::max(std::min(a, std::max(b, c)), std::min(std::max(a, b), c));
    }
}

int CameraLane::median_tail_x(const std::vector<cv::Point>& pts, int tail_count) {
    if (pts.empty() || tail_count <= 0) return 0;
    std::array<int, MEDIAN_TAIL_POINTS_MAX> values{};
    const int count = tail_count;
    const int n = std::max(0, std::min(count, static_cast<int>(
        std::min(static_cast<int>(pts.size()), MEDIAN_TAIL_POINTS_MAX))));
    if (n == 0) return 0;

    for (int i = 0; i < n; ++i) {
        values[static_cast<size_t>(i)] =
            pts[pts.size() - static_cast<size_t>(n) + static_cast<size_t>(i)].x;
    }
    std::sort(values.begin(), values.begin() + n);
    return values[n / 2];
}

int CameraLane::median_value(const std::vector<int>& values) {
    if (values.empty()) return 0;
    std::vector<int> tmp(values);
    const size_t mid = tmp.size() / 2;
    std::nth_element(tmp.begin(), tmp.begin() + mid, tmp.end());
    return tmp[mid];
}

float CameraLane::calculate_slope(const std::vector<cv::Point>& lane) {
    if (lane.size() < 10) return 0.0f;
    double sum_x = 0.0, sum_y = 0.0, sum_xy = 0.0, sum_yy = 0.0;
    const double n = static_cast<double>(lane.size());

    for (const auto& p : lane) {
        sum_x += p.x;
        sum_y += p.y;
        sum_xy += static_cast<double>(p.x) * static_cast<double>(p.y);
        sum_yy += static_cast<double>(p.y) * static_cast<double>(p.y);
    }

    const double denominator = n * sum_yy - sum_y * sum_y;
    if (std::abs(denominator) < 1e-9) return 0.0f;

    return static_cast<float>((n * sum_xy - sum_x * sum_y) / denominator);
}

int CameraLane::calculate_slope_offset(float dominant_slope) {
    const float magnitude = std::fabs(dominant_slope);
    if (magnitude <= SLOPE_OFFSET_START) return 0;

    const float normalized = clamp_float(
        (magnitude - SLOPE_OFFSET_START) / (SLOPE_OFFSET_END - SLOPE_OFFSET_START),
        0.0f, 1.0f);
    const float offset = SLOPE_OFFSET_MIN_PX +
        normalized * (SLOPE_OFFSET_MAX_PX - SLOPE_OFFSET_MIN_PX);
    const int offset_i = static_cast<int>(std::round(offset));

    return dominant_slope > 0.0f ? -offset_i : offset_i;
}

uint8_t CameraLane::calculate_target_speed(float dominant_slope, int abs_deviation) {
    const float slope_mag = std::fabs(dominant_slope);

    const bool sharp_enter =
        slope_mag >= SHARP_ENTER_SLOPE || abs_deviation >= SHARP_ENTER_DEV_PX;
    const bool sharp_exit =
        slope_mag < SHARP_EXIT_SLOPE && abs_deviation < SHARP_EXIT_DEV_PX;
    const bool curve_enter =
        slope_mag >= CURVE_ENTER_SLOPE || abs_deviation >= CURVE_ENTER_DEV_PX;
    const bool curve_exit =
        slope_mag < CURVE_EXIT_SLOPE && abs_deviation < CURVE_EXIT_DEV_PX;

    switch (speed_state_) {
        case SpeedState::STRAIGHT:
            if (sharp_enter) speed_state_ = SpeedState::SHARP;
            else if (curve_enter) speed_state_ = SpeedState::CURVE;
            break;
        case SpeedState::CURVE:
            if (sharp_enter) speed_state_ = SpeedState::SHARP;
            else if (curve_exit) speed_state_ = SpeedState::STRAIGHT;
            break;
        case SpeedState::SHARP:
            if (sharp_exit) {
                if (curve_exit) speed_state_ = SpeedState::STRAIGHT;
                else speed_state_ = SpeedState::CURVE;
            }
            break;
    }

    switch (speed_state_) {
        case SpeedState::SHARP: return SPEED_SHARP_X10;
        case SpeedState::CURVE: return SPEED_CURVE_X10;
        case SpeedState::STRAIGHT:
        default: return SPEED_STRAIGHT_X10;
    }
}

LaneConfidence CameraLane::grade_confidence(
    bool has_left,
    bool has_right,
    bool dual,
    size_t fit_points,
    float residual,
    bool ema_stable
) {
    if (!has_left && !has_right) return LaneConfidence::NONE;
    if (!dual) return LaneConfidence::WEAK;

    // Ngưỡng điểm fit: 60 = mức tối thiểu, 400 = rất nhiều điểm (mask đặc).
    const float point_score = clamp_float(
        static_cast<float>(fit_points) / 400.0f, 0.0f, 1.0f);
    // Ngưỡng residual: 3px rất tốt, 10px = trần cho phép.
    const float residual_score = clamp_float(
        (BIRD_MAX_FIT_RESIDUAL_PX - residual) / (BIRD_MAX_FIT_RESIDUAL_PX - 3.0f),
        0.0f, 1.0f);

    const float score = 0.45f * point_score + 0.55f * residual_score;

    if (score >= 0.75f && ema_stable) return LaneConfidence::STRONG;
    if (score >= 0.45f) return LaneConfidence::GOOD;
    return LaneConfidence::WEAK;
}

// ============================================================================
// GIẢI HỆ 3x3 KHÔNG CẤP PHÁT
// ============================================================================

bool CameraLane::solve3x3(const double A[9], const double b[3], double x[3]) {
    // Gaussian elimination kèm partial pivoting trên bản sao cục bộ.
    double m[3][4];
    for (int i = 0; i < 3; ++i) {
        m[i][0] = A[i * 3 + 0];
        m[i][1] = A[i * 3 + 1];
        m[i][2] = A[i * 3 + 2];
        m[i][3] = b[i];
    }

    for (int col = 0; col < 3; ++col) {
        int pivot = col;
        double best = std::abs(m[col][col]);
        for (int r = col + 1; r < 3; ++r) {
            const double v = std::abs(m[r][col]);
            if (v > best) {
                best = v;
                pivot = r;
            }
        }

        if (best < 1e-12) return false;

        if (pivot != col) {
            for (int c = col; c < 4; ++c) {
                std::swap(m[col][c], m[pivot][c]);
            }
        }

        const double diag = m[col][col];
        for (int r = col + 1; r < 3; ++r) {
            const double factor = m[r][col] / diag;
            if (factor == 0.0) continue;
            for (int c = col; c < 4; ++c) {
                m[r][c] -= factor * m[col][c];
            }
        }
    }

    for (int i = 2; i >= 0; --i) {
        double sum = m[i][3];
        for (int c = i + 1; c < 3; ++c) {
            sum -= m[i][c] * x[c];
        }
        if (std::abs(m[i][i]) < 1e-12) return false;
        x[i] = sum / m[i][i];
        if (!std::isfinite(x[i])) return false;
    }

    return true;
}

// ============================================================================
// POLYNOMIAL FIT
// x = c[0] + c[1]*yn + c[2]*yn^2,  yn = y / BIRD_H
//
// Hai điểm khác biệt so với bản cũ:
//   1. Mảng thô thay cho cv::Mat + at<double>. Bản cũ gọi at<> 9 lần mỗi
//      điểm (kèm kiểm tra biên và kiểu), với ~8000 điểm x 2 làn fit là
//      ~1.3 triệu lệnh at<>. Ở đây là 6 phép nhân cộng thuần.
//   2. Lọc ngoại lệ TRƯỚC khi giải. Bản cũ fit toàn bộ pixel rồi mới kiểm
//      residual trên 9 điểm trung tâm, nên một cụm nhiễu nhỏ trong mask
//      (rất dễ xảy ra khi phòng tối) kéo cả đường fit lệch.
// ============================================================================

float CameraLane::poly_eval(const float coef[3], float y) {
    const float yn = y / static_cast<float>(BIRD_H);
    return coef[0] + coef[1] * yn + coef[2] * yn * yn;
}

bool CameraLane::fit_poly(const std::vector<cv::Point>& pts, float coef[3]) {
    if (pts.size() < static_cast<size_t>(BIRD_MIN_FIT_POINTS)) return false;

    double A[9] = {0.0};
    double b[3] = {0.0};

    for (const auto& p : pts) {
        const double yn = static_cast<double>(p.y) / static_cast<double>(BIRD_H);
        const double y2 = yn * yn;
        const double y3 = y2 * yn;
        const double x = static_cast<double>(p.x);

        // Ma trận cho v = [1, yn, yn^2], u = x:
        //   [ n    Sy    Sy2 ] [c0]   [Sx]
        //   [ Sy   Sy2   Sy3 ] [c1] = [Sxy]
        //   [ Sy2  Sy3   Sy4 ] [c2]   [Sxy2]
        // Chỉ nửa trên được tích luỹ trực tiếp, nửa dưới điền sau cho đối
        // xứng. Chỉ số là row*3 + col.
        //
        // Với cơ sở bậc 2 thì [2][2] là SUM(yn^4), không phải SUM(yn^3) -
        // nếu dùng y3 ở đây thì hàng cuối cùng của ma trận bị thiếu bậc
        // và hệ số c2 sai, kéo theo toàn bộ đường fit lệch.
        const double y4 = y2 * y2;
        A[0] += 1.0;   A[1] += yn;  A[2] += y2;
        A[4] += y2;    A[5] += y3;  A[8] += y4;

        b[0] += x;
        b[1] += x * yn;
        b[2] += x * y2;
    }

    // Ma trận đối xứng: điền nửa dưới từ nửa trên.
    // A[3] = A[1] = Sy, A[6] = A[2] = Sy2, A[7] = A[5] = Sy3.
    A[3] = A[1];
    A[6] = A[2];
    A[7] = A[5];

    if (std::abs(A[4]) < 1e-9) return false;

    double x3[3] = {0.0, 0.0, 0.0};
    if (!solve3x3(A, b, x3)) return false;

    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(x3[i])) return false;
        coef[i] = static_cast<float>(x3[i]);
    }

    return true;
}

bool CameraLane::fit_residual_ok(
    const std::vector<cv::Point>& pts,
    const float coef[3]
) {
    if (pts.empty()) return false;

    double sum = 0.0;
    for (const auto& p : pts) {
        sum += std::abs(static_cast<double>(
            poly_eval(coef, static_cast<float>(p.y)) - static_cast<double>(p.x)));
    }

    return (sum / static_cast<double>(pts.size())) <=
           static_cast<double>(BIRD_MAX_FIT_RESIDUAL_PX);
}

namespace {

int clamp_int(int v, int lo, int hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// Một lượt lọc ngoại lệ: giữ lại các điểm có |residual| <= max(px_min, k*sigma).
// Dùng std::nth_element để O(n) thay vì O(n log n).
bool reject_outliers(
    const std::vector<cv::Point>& in,
    std::vector<cv::Point>& out,
    const float coef[3],
    int bird_h,
    float px_min,
    float sigma_k,
    int min_keep
) {
    if (in.size() < 8) {
        out = in;
        return false;
    }

    const size_t n = in.size();
    std::vector<float> err(n);
    for (size_t i = 0; i < n; ++i) {
        const float yn = static_cast<float>(in[i].y) / static_cast<float>(bird_h);
        const float pred = coef[0] + coef[1] * yn + coef[2] * yn * yn;
        err[i] = std::fabs(pred - static_cast<float>(in[i].x));
    }

    // Ước lượng sigma bằng trung vị (robust với chính các điểm nhiễu).
    std::vector<float> sorted = err;
    const size_t mid = sorted.size() / 2;
    std::nth_element(sorted.begin(), sorted.begin() + mid, sorted.end());
    const float mad = sorted[mid];
    // 1.4826 đổi MAD thành ước lượng sigma của phân phối chuẩn.
    const float sigma = std::max(1e-3f, mad * 1.4826f);

    const float limit = std::max(px_min, sigma_k * sigma);

    out.clear();
    out.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        if (err[i] <= limit) out.push_back(in[i]);
    }

    // Không loại quá nhiều, nếu không thì bộ điểm không còn tin cậy.
    if (out.size() < static_cast<size_t>(min_keep) ||
        out.size() * 4 < n * 3) {
        out = in;
        return false;
    }

    return true;
}

}  // namespace

bool CameraLane::fit_poly_filtered(
    const std::vector<cv::Point>& pts,
    const std::vector<cv::Point>& win_means,
    float coef[3],
    float& residual_out
) {
    residual_out = BIRD_MAX_FIT_RESIDUAL_PX;

    if (pts.size() < static_cast<size_t>(BIRD_MIN_FIT_POINTS)) return false;

    // Lượt 1: fit toàn bộ để có đường tham chiếu.
    if (!fit_poly(pts, coef)) return false;

    // Lượt 2: loại ngoại lệ theo MAD rồi fit lại. reject_outliers tự trả về
    // toàn bộ điểm nếu việc loại bỏ quá nặn, nên filtered_ không bao giờ rỗng
    // khi đầu vào hợp lệ.
    filtered_.clear();
    reject_outliers(pts, filtered_, coef, BIRD_H,
                    OUTLIER_MIN_RESIDUAL_PX, OUTLIER_SIGMA_K,
                    BIRD_MIN_FIT_POINTS);
    if (filtered_.empty()) filtered_ = pts;

    if (!fit_poly(filtered_, coef)) {
        // Bộ điểm đã lọc bị khuất địa hình -> quay lại dùng kết quả lượt 1.
        filtered_ = pts;
        if (!fit_poly(pts, coef)) return false;
    }

    // Residual trung bình trên bộ điểm thực sự dùng để fit. Đây là con số đi
    // vào grade_confidence, nên phải luôn được gán kể cả khi bước cuối thất
    // bại (để caller biết vì sao).
    double sum = 0.0;
    for (const auto& p : filtered_) {
        sum += std::abs(static_cast<double>(
            poly_eval(coef, static_cast<float>(p.y)) - static_cast<double>(p.x)));
    }
    residual_out = static_cast<float>(sum / static_cast<double>(filtered_.size()));

    if (residual_out > BIRD_MAX_FIT_RESIDUAL_PX) return false;

    // Đối chiếu chéo với 9 trung tâm cửa sổ: một cụm nhiễu nhỏ có thể không
    // kéo residual trung bình vượt trần nhưng vẫn làm lệch đường fit so với
    // vị trí cửa sổ đã bám.
    if (!win_means.empty() && !fit_residual_ok(win_means, coef)) return false;

    return true;
}

// ============================================================================
// IPM
// ============================================================================

bool CameraLane::update_ipm(int mask_w, int mask_h) {
    if (mask_w < 32 || mask_h < 32) return false;

    if (m_ipm_.empty() || ipm_w_ != mask_w || ipm_h_ != mask_h) {
        const float fx = static_cast<float>(mask_w);
        const float fy = static_cast<float>(mask_h);

        const Point2f src[4] = {
            Point2f(IPM_TL_X * fx, 0.0f),
            Point2f(IPM_TR_X * fx, 0.0f),
            Point2f(IPM_BR_X * fx, fy - 1.0f),
            Point2f(IPM_BL_X * fx, fy - 1.0f)
        };

        const Point2f dst[4] = {
            Point2f(0.0f, 0.0f),
            Point2f(static_cast<float>(BIRD_W - 1), 0.0f),
            Point2f(static_cast<float>(BIRD_W - 1), static_cast<float>(BIRD_H - 1)),
            Point2f(0.0f, static_cast<float>(BIRD_H - 1))
        };

        m_ipm_ = getPerspectiveTransform(src, dst);
        m_ipm_inv_ = getPerspectiveTransform(dst, src);
        ipm_w_ = mask_w;
        ipm_h_ = mask_h;
    }

    return !m_ipm_.empty() && !m_ipm_inv_.empty();
}

// ============================================================================
// CỘT TÍCH PHÂN
// col_integral_[y][x] = số pixel khác 0 trong cột x, các hàng 0..y-1.
// Nhờ vậy histogram của một dải hàng chỉ cần 2 phép trừ cho mỗi cột.
//
// Bản cũ: findNonZero() rồi search_window() quét TOÀN BỘ vector điểm HAI
// LẦN cho mỗi cửa sổ, tức 9 cửa sổ x 2 phía x 2 = 36 lượt quét toàn bộ.
// Với mask nhiễu và 40k điểm đó là ~1.4 triệu vòng lặp mỗi frame.
// ============================================================================

void CameraLane::build_col_integral(const cv::Mat& bin) {
    const size_t stride = static_cast<size_t>(BIRD_W);
    // Hàng 0 luôn bằng 0
    std::fill(col_integral_.begin(), col_integral_.begin() + stride, 0u);

    for (int y = 0; y < BIRD_H; ++y) {
        const uchar* row = bin.ptr<uchar>(y);
        uint32_t* prev = col_integral_.data() + static_cast<size_t>(y) * stride;
        uint32_t* cur = prev + stride;

        for (int x = 0; x < BIRD_W; ++x) {
            cur[x] = prev[x] + (row[x] ? 1u : 0u);
        }
    }
}

// ============================================================================
// CỬA SỔ TRƯỢT
// ============================================================================

bool CameraLane::window_search(float& center_x, bool& dual) {
    left_win_means_.clear();
    right_win_means_.clear();
    left_fit_px_.clear();
    right_fit_px_.clear();
    center_x = static_cast<float>(BIRD_W) * 0.5f;
    dual = false;

    if (bird_.empty()) return false;

    const int mid = BIRD_W / 2;

    // Histogram cột trên nửa dưới của bird view, lấy thẳng từ col integral.
    const int y_hist0 = static_cast<int>(static_cast<float>(BIRD_H) * BIRD_HIST_TOP_FRAC);
    {
        const uint32_t* upper = col_integral_.data() + static_cast<size_t>(y_hist0) * BIRD_W;
        const uint32_t* lower = col_integral_.data() + static_cast<size_t>(BIRD_H) * BIRD_W;
        for (int x = 0; x < BIRD_W; ++x) {
            bird_col_sum_[static_cast<size_t>(x)] =
                static_cast<int>(lower[x] - upper[x]);
        }
    }

    int left_seed = 0;
    int right_seed = 0;

    // Tìm cặp đỉnh histogram có thung lũng thật sự giữa hai bên.
    // Không chia cứng tại BIRD_W/2 vì khi xe lệch tâm lane, đỉnh của làn
    // phải rơi vào nửa "trái" và che mất làn kia.
    const int hist_rows = BIRD_H - y_hist0;
    const int min_column = std::max(
        WINDOW_MIN_POINTS,
        static_cast<int>(static_cast<float>(hist_rows) * BIRD_MIN_COLUMN_RATIO));

    int peak_a = -1;
    int peak_a_val = 0;
    for (int x = 0; x < BIRD_W; ++x) {
        const int v = bird_col_sum_[static_cast<size_t>(x)];
        if (v > peak_a_val) { peak_a_val = v; peak_a = x; }
    }

    int peak_b = -1;
    int peak_b_val = 0;
    if (peak_a_val >= min_column) {
        for (int x = 0; x < BIRD_W; ++x) {
            if (std::abs(x - peak_a) <= WINDOW_MARGIN) continue;
            const int v = bird_col_sum_[static_cast<size_t>(x)];
            if (v > peak_b_val) { peak_b_val = v; peak_b = x; }
        }
    }

    int valley = BIRD_H;
    if (peak_b >= 0) {
        for (int x = std::min(peak_a, peak_b) + 1; x < std::max(peak_a, peak_b); ++x) {
            valley = std::min(valley, bird_col_sum_[static_cast<size_t>(x)]);
        }
    }

    const bool two_peaks = peak_a_val >= min_column &&
                           peak_b_val >= min_column &&
                           valley <= static_cast<int>(
                               static_cast<float>(std::min(peak_a_val, peak_b_val)) *
                               BIRD_VALLEY_RATIO);

    if (two_peaks) {
        left_seed = std::min(peak_a, peak_b);
        right_seed = std::max(peak_a, peak_b);

        // Bootstrap bề rộng làn ngay từ khoảng cách hai đỉnh.
        // Bản cũ để bird_lane_width_ = 0 tới khi width_ok chạy, mà
        // width_ok lại cần 2 seed tách biệt -> vòng luẩn: 2 làn dính ->
        // abort -> không bao giờ có width_ok -> vẫn dính mãi.
        if (bird_lane_width_ <= 0.0f) {
            bird_lane_width_ = static_cast<float>(right_seed - left_seed);
        }
    } else if (peak_a_val >= min_column) {
        // Chỉ thấy một đỉnh: quyết định bên bằng trọng tâm khối lane.
        long sum_x = 0;
        long sum_w = 0;
        for (int x = 0; x < BIRD_W; ++x) {
            sum_x += static_cast<long>(x) * bird_col_sum_[static_cast<size_t>(x)];
            sum_w += bird_col_sum_[static_cast<size_t>(x)];
        }

        const float centroid = sum_w > 0
            ? static_cast<float>(sum_x) / static_cast<float>(sum_w)
            : static_cast<float>(mid);

        if (std::fabs(centroid - static_cast<float>(mid)) <
            BIRD_SIDE_TOLERANCE * static_cast<float>(BIRD_W)) {
            // Bản cũ: bird_valid_ = false; return false;
            //
            // Đây là nhánh gây mất 2 làn kinh niên. Khi phòng tối làm 2 vạch
            // dính vào nhau, centroid rơi gần giữa -> hàm abort TOÀN BỘ IPM ->
            // rơi xuống nhánh Canny/scanline yếu hơn nhiều. Và vì bird_lane_width_
            // chưa bao giờ được set nên lần sau vẫn vậy -> kẹt vĩnh viễn.
            //
// Nay: coi đây là trường hợp MỘT LÀN, dùng cả bề rộng bootstrap
            // để suy ra vị trí vạch bên kia, và vẫn trả kết quả dùng được.
            // Nhánh single-lane ở detect_lanes sẽ tự ước lượng tâm đường.
            if (bird_lane_width_ <= 0.0f) {
                bird_lane_width_ = BIRD_BOOTSTRAP_WIDTH_FRAC * static_cast<float>(BIRD_W);
            }

            int half_w = static_cast<int>(std::lround(bird_lane_width_ * 0.5f));
            if (half_w < BIRD_MIN_HALF_WIDTH) half_w = BIRD_MIN_HALF_WIDTH;
            if (half_w > mid - 2) half_w = mid - 2;

            // Dồn cả hai vạch theo trọng tâm khối lane để giữ đúng vị trí xe
            // đang đứng, thay vì luôn giả định xe ở chính giữa bird view.
            const int shift = static_cast<int>(std::lround(centroid)) - mid;

            left_seed = std::max(2, mid - half_w + shift);
            right_seed = std::min(BIRD_W - 3, mid + half_w + shift);

            bird_merged_hint_ = true;
        } else if (centroid < static_cast<float>(mid)) {
            left_seed = static_cast<int>(std::lround(centroid));
            if (bird_lane_width_ <= 0.0f) {
                bird_lane_width_ = BIRD_BOOTSTRAP_WIDTH_FRAC * static_cast<float>(BIRD_W);
            }
        } else {
            right_seed = static_cast<int>(std::lround(centroid));
            if (bird_lane_width_ <= 0.0f) {
                bird_lane_width_ = BIRD_BOOTSTRAP_WIDTH_FRAC * static_cast<float>(BIRD_W);
            }
        }
    }

    // Seed = 0 là giá trị dành riêng cho "chưa tìm thấy" (biên trái không thể là
    // vị trí vạch vì bị sát mép ảnh), nên has_* suy ra thẳng từ seed. Cách cũ
    // suy ra từ việc "seed kia bằng 0" thì hỏng ở nhánh hai vạch dính giữa:
    // cả hai seed đều khác 0 nên has_left lẫn has_right đều false và hàm abort
    // trước khi kịp dùng kết quả.
    if (left_seed > 0) {
        left_seed = std::max(2, std::min(left_seed, BIRD_W - 3));
    }
    if (right_seed > 0) {
        right_seed = std::max(2, std::min(right_seed, BIRD_W - 3));
    }

    const bool has_left = left_seed > 0;
    const bool has_right = right_seed > 0;

    if (!has_left && !has_right) {
        bird_valid_ = false;
        return false;
    }

    int half_w = static_cast<int>(std::lround(bird_lane_width_ * 0.5f));
    if (half_w < BIRD_MIN_HALF_WIDTH) half_w = BIRD_MIN_HALF_WIDTH;
    if (half_w > mid - 2) half_w = mid - 2;

    if (bird_valid_) {
        const int prev_mid = static_cast<int>(std::lround(bird_center_prev_));
        const int guess_left = std::max(2, std::min(prev_mid - half_w, BIRD_W - 5));
        const int guess_right = std::min(BIRD_W - 3, guess_left + 2 * half_w);

        if (has_left && has_right) {
            const int seed_mid = (left_seed + right_seed) / 2;
            if (std::abs(seed_mid - prev_mid) > BIRD_XMID_JUMP_MAX) {
                left_seed = guess_left;
                right_seed = std::max(guess_left + 4, guess_right);
            }
        } else if (has_left) {
            if (std::abs(left_seed - guess_left) > BIRD_XMID_JUMP_MAX) left_seed = guess_left;
        } else {
            if (std::abs(right_seed - guess_right) > BIRD_XMID_JUMP_MAX) {
                right_seed = guess_right;
            }
        }
    }

    left_seed = std::max(0, std::min(left_seed, BIRD_W - 1));
    right_seed = std::max(0, std::min(right_seed, BIRD_W - 1));
    if (has_left && has_right && left_seed >= right_seed) right_seed = std::min(BIRD_W - 1, left_seed + 1);

    const int window_h = std::max(1, BIRD_H / N_WINDOWS);
    int lx = left_seed;
    int rx = right_seed;

    for (int w = 0; w < N_WINDOWS; ++w) {
        const int y_low = BIRD_H - (w + 1) * window_h;
        const int y_high = BIRD_H - w * window_h;
        const int y_mid = (y_low + y_high) / 2;

        if (has_left) search_window(y_low, y_high, y_mid, 0, lx);
        if (has_right) search_window(y_low, y_high, y_mid, 1, rx);
    }

    if (has_left && has_right && !bird_merged_hint_) {
        dual = true;
        center_x = static_cast<float>((lx + rx) / 2);
    } else if (has_left) {
        center_x = static_cast<float>(lx);
    } else if (has_right) {
        center_x = static_cast<float>(rx);
    } else {
        return false;
    }

    return !left_fit_px_.empty() || !right_fit_px_.empty();
}

// ============================================================================
// MỘT CỬA SỔ TRƯỢT
// Dùng đỉnh histogram + trung tâm run cục bộ. Gom pixel lane của chính cửa
// sổ đó vào bộ điểm fit.
// ============================================================================

bool CameraLane::search_window(
    int y_low,
    int y_high,
    int y_mid,
    int side,
    int& seed
) {
    const size_t stride = static_cast<size_t>(BIRD_W);
    const uint32_t* upper = col_integral_.data() + static_cast<size_t>(y_low) * stride;
    const uint32_t* lower = col_integral_.data() + static_cast<size_t>(y_high) * stride;

    const int x_low = std::max(0, seed - WINDOW_MARGIN);
    const int x_high = std::min(BIRD_W - 1, seed + WINDOW_MARGIN);

    int peak_x = -1;
    int peak_val = 0;
    for (int x = x_low; x <= x_high; ++x) {
        const int v = static_cast<int>(lower[x] - upper[x]);
        bird_win_sum_[static_cast<size_t>(x)] = v;

        if (v > peak_val ||
            (v == peak_val && peak_x >= 0 && std::abs(x - seed) < std::abs(peak_x - seed))) {
            peak_val = v;
            peak_x = x;
        }
    }

    if (peak_x < 0 || peak_val < WINDOW_MIN_POINTS) return false;

    const int run_threshold = static_cast<int>(
        static_cast<float>(peak_val) * WINDOW_RUN_RATIO);

    int run_low = peak_x;
    while (run_low > x_low &&
           bird_win_sum_[static_cast<size_t>(run_low - 1)] > run_threshold) --run_low;

    int run_high = peak_x;
    while (run_high < x_high &&
           bird_win_sum_[static_cast<size_t>(run_high + 1)] > run_threshold) ++run_high;

    const int centre = (run_low + run_high) / 2;
    if (std::abs(centre - seed) > WINDOW_MAX_SHIFT) return false;

    seed = centre;

    auto& fit_px = (side == 0) ? left_fit_px_ : right_fit_px_;
    auto& means = (side == 0) ? left_win_means_ : right_win_means_;

    // Quét trực tiếp band pixel thay vì duyệt vector điểm toàn cục.
    for (int y = y_low; y < y_high; ++y) {
        const uchar* row = bird_.ptr<uchar>(y);
        for (int x = run_low; x <= run_high; ++x) {
            if (row[x]) fit_px.emplace_back(x, y);
        }
    }

    means.emplace_back(centre, y_mid);
    return true;
}

// ============================================================================
// ĐƯỜNG CON VÀO ẢNH GỐC
// Dùng phép chiếu theo điểm (perspectiveTransform với m_ipm_inv_), giữ độ
// chính xác dưới pixel, tương đương chiếu ngược cả ảnh nhưng không tạo
// artefact raster hoá.
// ============================================================================

void CameraLane::build_curve_points(
    bool fit_ok,
    const float coef[3],
    int y0,
    int y1,
    std::vector<cv::Point>& out
) {
    out.clear();
    if (!fit_ok || m_ipm_inv_.empty() || ipm_w_ <= 0 || ipm_h_ <= 0) return;

    const int y_top = std::max(0, static_cast<int>(
        static_cast<float>(BIRD_H) * BIRD_SAMPLE_TOP_FRAC));

    bird_pts_buf_.clear();
    for (int by = BIRD_H - 1; by >= y_top; by -= BIRD_SAMPLE_STEP) {
        const float bx = poly_eval(coef, static_cast<float>(by));
        if (!std::isfinite(bx)) continue;
        bird_pts_buf_.emplace_back(bx, static_cast<float>(by));
    }
    if (bird_pts_buf_.empty()) return;

    // bird (BIRD_W x BIRD_H) -> ảnh làm việc (ipm_w_ x ipm_h_)
    roi_pts_buf_.clear();
    roi_pts_buf_.reserve(bird_pts_buf_.size());
    cv::perspectiveTransform(bird_pts_buf_, roi_pts_buf_, m_ipm_inv_);

    // Ảnh làm việc -> ảnh gốc camera. BƯỚC NÀY BẮT BUỘC: mask được tính ở
    // kích thước nhỏ (320x200) trong khi leftPts_/rightPts_ được dùng như
    // toạ độ ảnh gốc. Thiếu nó, toàn bộ điểm nằm trong nửa trên-trái và bị
    // lọc hết bởi điều kiện biên.
    const float sx = static_cast<float>(frame_w_) / static_cast<float>(ipm_w_);
    const float sy = static_cast<float>(frame_h_) / static_cast<float>(ipm_h_);

    out.reserve(roi_pts_buf_.size());
    for (const auto& p : roi_pts_buf_) {
        const float fy = p.y * sy;
        if (fy < static_cast<float>(y0) || fy > static_cast<float>(y1)) continue;

        const float fx = p.x * sx;
        if (fx < 0.0f || fx >= static_cast<float>(frame_w_)) continue;

        out.emplace_back(
            static_cast<int>(std::lround(fx)),
            static_cast<int>(std::lround(fy)));
    }

    std::reverse(out.begin(), out.end());
}

// ============================================================================
// LẤY MẪU LẠI HAI LÀN TRÊN CÙNG LƯỚI Y TRONG ẢNH GỐC
// ============================================================================

void CameraLane::resample_curve(
    const std::vector<cv::Point>& src,
    int y0_used,
    int y1,
    std::vector<cv::Point>& out
) {
    out.clear();
    if (src.size() < 2) return;

    out.reserve(static_cast<size_t>((y1 - y0_used) / IMAGE_SAMPLE_STEP + 1));

    size_t i = 0;
    for (int y = y0_used; y <= y1; y += IMAGE_SAMPLE_STEP) {
        while (i + 2 < src.size() && src[i + 1].y < y) ++i;

        const cv::Point& a = src[i];
        const cv::Point& b = src[std::min(i + 1, src.size() - 1)];

        if (y < a.y) continue;
        if (y > b.y) break;

        if (b.y == a.y) {
            out.emplace_back(a.x, y);
            continue;
        }

        const float t = static_cast<float>(y - a.y) / static_cast<float>(b.y - a.y);
        out.emplace_back(
            static_cast<int>(std::lround(a.x + t * static_cast<float>(b.x - a.x))), y);
    }
}

// ============================================================================
// NHÁNH CHÍNH: IPM
// ============================================================================

bool CameraLane::detect_lanes_ipm(
    const cv::Mat& mask_work,
    int y0_dual,
    int y0_single,
    int y1,
    float width_scale,
    bool& is_dual_lane,
    LaneOutput& out
) {
    is_dual_lane = false;

    if (!update_ipm(mask_work.cols, mask_work.rows)) return false;

    warpPerspective(mask_work, bird_, m_ipm_, bird_size_, INTER_NEAREST);
    if (bird_.empty()) return false;

    // Không che nắp xe ở đây.
    //
    // Bản cũ gọi mask_bonnet(bird_) với các hệ số lấy cho ảnh 640x400, áp lên
    // ảnh 320x240 -> xoá đúng vùng x∈[100,200], y≥155, tức đúng chỗ hai làn
    // hội tụ ở đáy bird view. Kết quả là 3 cửa sổ dưới cùng mất dữ liệu.
    // Nắp xe đã được che ở lane_mask::build_lane_mask trên ảnh gốc.

    bird_merged_hint_ = false;

    build_col_integral(bird_);

    float center_x = static_cast<float>(BIRD_W) * 0.5f;
    bool seed_dual = false;
    if (!window_search(center_x, seed_dual)) {
        bird_valid_ = false;
        return false;
    }

    // Fit với lọc ngoại lệ. Đích của hệ số là buffer "raw" để EMA tách biệt
    // được giá trị mới với giá trị đã lọc.
    float left_residual = BIRD_MAX_FIT_RESIDUAL_PX;
    float right_residual = BIRD_MAX_FIT_RESIDUAL_PX;

    const bool left_fit_ok =
        left_fit_px_.size() >= static_cast<size_t>(BIRD_MIN_FIT_PIXELS) &&
        fit_poly_filtered(left_fit_px_, left_win_means_,
                          left_fit_raw_, left_residual);
    const bool right_fit_ok =
        right_fit_px_.size() >= static_cast<size_t>(BIRD_MIN_FIT_PIXELS) &&
        fit_poly_filtered(right_fit_px_, right_win_means_,
                          right_fit_raw_, right_residual);

    if (!left_fit_ok && !right_fit_ok) {
        bird_valid_ = false;
        return false;
    }

    // EMA trên hệ số để giảm rung. Ảnh yếu thì EMA mạnh hơn.
    // edge_quality = sobel_p99 / 255 nên tương đối, không phụ thuộc ánh sáng
    // phòng, khác với việc so p99 với một hằng số tuyệt đối.
    const lane_mask::Options mopts = opts();
    const bool weak_frame =
        mask_result_.edge_quality < mopts.weak_edge_ratio;
    const float ema_alpha = static_cast<float>(
        weak_frame ? mopts.ema_alpha_weak : mopts.ema_alpha_strong);

    if (left_fit_ok) {
        if (left_fit_init_) {
            for (int i = 0; i < 3; ++i) {
                left_fit_[i] = ema_alpha * left_fit_[i] +
                    (1.0f - ema_alpha) * left_fit_raw_[i];
            }
        } else {
            std::copy(left_fit_raw_, left_fit_raw_ + 3, left_fit_);
            left_fit_init_ = true;
        }
    }
    if (right_fit_ok) {
        if (right_fit_init_) {
            for (int i = 0; i < 3; ++i) {
                right_fit_[i] = ema_alpha * right_fit_[i] +
                    (1.0f - ema_alpha) * right_fit_raw_[i];
            }
        } else {
            std::copy(right_fit_raw_, right_fit_raw_ + 3, right_fit_);
            right_fit_init_ = true;
        }
    }

    build_curve_points(left_fit_ok, left_fit_, y0_dual, y1, left_raw_);
    build_curve_points(right_fit_ok, right_fit_, y0_dual, y1, right_raw_);

    bool width_ok = false;
    float bird_width = 0.0f;
    float image_width = 0.0f;
    float image_center_x = 0.0f;

    if (left_fit_ok && right_fit_ok) {
        const float bird_y = static_cast<float>(BIRD_H - 1);
        const float bx_left = poly_eval(left_fit_, bird_y);
        const float bx_right = poly_eval(right_fit_, bird_y);
        bird_width = bx_right - bx_left;

        if (std::abs(bird_width) > 1.0f) {
            bird_bottom_buf_.clear();
            bird_bottom_buf_.reserve(2);
            bird_bottom_buf_.emplace_back(bx_left, bird_y);
            bird_bottom_buf_.emplace_back(bx_right, bird_y);

            roi_bottom_buf_.clear();
            roi_bottom_buf_.reserve(2);
            perspectiveTransform(bird_bottom_buf_, roi_bottom_buf_, m_ipm_inv_);

            if (roi_bottom_buf_.size() == 2) {
                // roi_bottom_buf_ nằm trong ảnh làm việc -> nhân tỉ lệ về
                // ảnh gốc, nếu không bề rộng đo được luôn nhỏ hơn thực tế
                // đúng bằng hệ số resize và mọi ngưỡng IPM_MIN/MAX_WIDTH_RATIO
                // sai lệch.
                const float sx = static_cast<float>(frame_w_) /
                                 static_cast<float>(ipm_w_);

                image_width = std::fabs(
                    static_cast<float>(roi_bottom_buf_[1].x - roi_bottom_buf_[0].x)) * sx;
                image_center_x = sx * static_cast<float>(
                    0.5 * (roi_bottom_buf_[0].x + roi_bottom_buf_[1].x));

                const float estimate = lane_width_est_px_ > 1.0f
                    ? lane_width_est_px_
                    : DEFAULT_LANE_WIDTH_PX * width_scale;

                width_ok = bird_width >= BIRD_MIN_LANE_WIDTH &&
                           bird_width <= BIRD_MAX_LANE_WIDTH &&
                           image_width >= estimate * IPM_MIN_WIDTH_RATIO &&
                           image_width <= estimate * IPM_MAX_WIDTH_RATIO;
            }
        }
    }

    if (width_ok) {
        bird_lane_width_ = bird_lane_width_ > 0.0f
            ? ALPHA_LANE_WIDTH * bird_width + (1.0f - ALPHA_LANE_WIDTH) * bird_lane_width_
            : bird_width;
        lane_width_est_px_ = ALPHA_LANE_WIDTH * image_width +
            (1.0f - ALPHA_LANE_WIDTH) * lane_width_est_px_;
        bird_center_prev_ = center_x;
        bird_valid_ = true;
        xmid_scan_ = std::max(0, std::min(
            static_cast<int>(std::lround(image_center_x)), frame_w_ - 1));
        is_dual_lane = true;
    } else {
        // Vẫn giữ bird_valid_ = true: có ít nhất một fit thành công, và giữ
        // bird_center_prev_ để lượt sau dùng được ràng buộc liên tục.
        bird_center_prev_ = bird_valid_ ? bird_center_prev_ : center_x;
        bird_valid_ = true;
    }

    const int y0_used = is_dual_lane ? y0_dual : y0_single;

    resample_curve(left_raw_, y0_used, y1, leftPts_);
    resample_curve(right_raw_, y0_used, y1, rightPts_);

    // Đánh giá độ tin cậy
    const float worst_residual = std::max(left_residual, right_residual);
    const size_t total_fit = left_fit_px_.size() + right_fit_px_.size();
    const bool ema_stable = left_fit_init_ && right_fit_init_;

    out.confidence = grade_confidence(
        leftPts_.size() >= static_cast<size_t>(MIN_FINAL_POINTS),
        rightPts_.size() >= static_cast<size_t>(MIN_FINAL_POINTS),
        is_dual_lane,
        total_fit,
        worst_residual,
        ema_stable);

    out.bird = bird_;

    return !leftPts_.empty() || !rightPts_.empty();
}

// ============================================================================
// VẼ BĂNG LÀN
// Vẽ thẳng vùng polygon với addWeighted trên ROI, không clone cả ảnh.
// ============================================================================

void CameraLane::draw_lane_band(cv::Mat& vis) const {
    if (leftPts_.size() < 2 || rightPts_.size() < 2) return;

    const int band = std::max(1, static_cast<int>(std::round(
        BIRD_LANE_BAND_MARGIN * static_cast<float>(vis.cols) /
        static_cast<float>(FRAME_W))));

    // Giới hạn vùng vẽ trong bounding box thực tế của đa giác
    int min_x = vis.cols;
    int max_x = 0;
    for (const auto& p : leftPts_) {
        min_x = std::min(min_x, p.x - band);
        max_x = std::max(max_x, p.x + band);
    }
    for (const auto& p : rightPts_) {
        min_x = std::min(min_x, p.x - band);
        max_x = std::max(max_x, p.x + band);
    }

    min_x = std::max(0, min_x);
    max_x = std::min(vis.cols - 1, max_x);
    if (max_x <= min_x) return;

    const Rect roi(min_x, 0, max_x - min_x + 1, vis.rows);

    // fillConvexPoly ghi vào buffer overlay có gốc toạ độ tại roi.x, còn
    // leftPts_/rightPts_ đã là toạ độ ảnh gốc. Phải trừ roi.x, nếu không đa
    // giác lệch sang trái đúng bằng min_x và băng làn vẽ sai vị trí.
    std::vector<Point> polygon;
    polygon.reserve(leftPts_.size() + rightPts_.size());

    const int rw = roi.width - 1;
    const int rh = roi.height - 1;

    for (const auto& p : leftPts_) {
        polygon.emplace_back(
            clamp_int(p.x - band - roi.x, 0, rw), clamp_int(p.y, 0, rh));
    }
    for (auto it = rightPts_.rbegin(); it != rightPts_.rend(); ++it) {
        polygon.emplace_back(
            clamp_int(it->x + band - roi.x, 0, rw), clamp_int(it->y, 0, rh));
    }

    cv::Mat overlay(roi.size(), CV_8UC3);
    fillConvexPoly(overlay, polygon, Scalar(0, 200, 0));
    addWeighted(overlay, BIRD_LANE_BAND_ALPHA, vis(roi),
                1.0 - BIRD_LANE_BAND_ALPHA, 0.0, vis(roi));
}

// ============================================================================
// NHÁNH DỰ PHÒNG: CANNY + QUÉT DÒNG
// ============================================================================

bool CameraLane::detect_lanes_scanline(
    const cv::Mat& roi,
    float width_scale,
    int y0_dual,
    int y0_single,
    int y1,
    bool& is_dual_lane
) {
    const int W = roi.cols;
    const int H = roi.rows;

    if (gray_.channels() != 1 || gray_.size() != roi.size()) {
        return false;
    }

    GaussianBlur(gray_, blur_, Size(BLUR_KERNEL, BLUR_KERNEL), 0);
    Canny(blur_, edges_, CANNY_LOW, CANNY_HIGH);
    morphologyEx(edges_, edges_, MORPH_CLOSE, morph_kernel_, Point(-1, -1), 1);

    leftPts_prelim_.clear();
    rightPts_prelim_.clear();
    prelim_centers_.clear();

    const float min_width = MIN_LANE_WIDTH_PX * width_scale;
    const float max_width = MAX_LANE_WIDTH_PX * width_scale;

    int xmid_candidate = std::max(0, std::min(xmid_scan_, W - 1));
    double width_sum = 0.0;
    int overlap_count = 0;

    for (int y = y0_dual; y <= y1; y += SCAN_STEP) {
        const int local_y = y - y0_dual;
        if (local_y < 0 || local_y >= H) continue;
        const uchar* row = edges_.ptr<uchar>(local_y);
        int left = -1, right = -1;

        // Bản cũ lấy pixel biên ĐẦU TIÊN từ xmid ra hai hướng: một pixel
        // nhiễu bất kỳ sinh ra một "làn" hoàn toàn sai. Nay chỉ nhận điểm nếu
        // đó nằm trong một cụm biên liên tục.
        const int MIN_RUN = 2;
        int run = 0;
        for (int x = xmid_candidate; x >= 0; --x) {
            if (row[x] != 0) {
                ++run;
                if (run >= MIN_RUN) { left = x + MIN_RUN - 1; break; }
            } else {
                run = 0;
            }
        }

        run = 0;
        for (int x = xmid_candidate; x < W; ++x) {
            if (row[x] != 0) {
                ++run;
                if (run >= MIN_RUN) { right = x - MIN_RUN + 1; break; }
            } else {
                run = 0;
            }
        }

        if (left >= 0) leftPts_prelim_.emplace_back(left, y);
        if (right >= 0) rightPts_prelim_.emplace_back(right, y);

        if (left >= 0 && right >= 0) {
            const int width = right - left;
            if (width >= min_width && width <= max_width) {
                width_sum += width;
                ++overlap_count;
                prelim_centers_.push_back((left + right) / 2);
            }
        }
    }

    const bool has_left_prelim =
        leftPts_prelim_.size() >= static_cast<size_t>(MIN_PRELIM_POINTS);
    const bool has_right_prelim =
        rightPts_prelim_.size() >= static_cast<size_t>(MIN_PRELIM_POINTS);
    float avg_lane_width = 0.0f;

    if (has_left_prelim && has_right_prelim && overlap_count >= MIN_COMMON_POINTS) {
        avg_lane_width = static_cast<float>(width_sum / static_cast<double>(overlap_count));
        if (avg_lane_width >= min_width && avg_lane_width <= max_width) {
            is_dual_lane = true;
        }
    }

    if (is_dual_lane) {
        lane_width_est_px_ = ALPHA_LANE_WIDTH * avg_lane_width +
            (1.0f - ALPHA_LANE_WIDTH) * lane_width_est_px_;
    }

    if (is_dual_lane && !prelim_centers_.empty()) {
        const int measured_mid = median_value(prelim_centers_);
        const int diff = measured_mid - xmid_candidate;
        if (std::abs(diff) > 20) {
            xmid_candidate += diff > 0 ? 20 : -20;
        } else {
            xmid_candidate = measured_mid;
        }
    } else {
        const int diff = W / 2 - xmid_candidate;
        if (std::abs(diff) > 5) {
            xmid_candidate += diff > 0 ? 5 : -5;
        } else {
            xmid_candidate = W / 2;
        }
    }

    xmid_scan_ = std::max(0, std::min(xmid_candidate, W - 1));

    const int y0_used = is_dual_lane ? y0_dual : y0_single;
    leftPts_.clear();
    rightPts_.clear();

    for (const auto& p : leftPts_prelim_) {
        if (p.y >= y0_used) leftPts_.push_back(p);
    }
    for (const auto& p : rightPts_prelim_) {
        if (p.y >= y0_used) rightPts_.push_back(p);
    }

    median_smooth(leftPts_);
    median_smooth(rightPts_);

    return is_dual_lane;
}

// ============================================================================
// DETECT LANES
// ============================================================================

void CameraLane::detect_lanes(
    const cv::Mat& bgr,
    float current_speed_kmh,
    LaneOutput& out
) {
    out.left.clear();
    out.right.clear();
    out.center.clear();
    out.mask.release();
    out.bird.release();
    out.confidence = LaneConfidence::NONE;
    out.camera_cmd = "STOP";
    out.dev_final_px = 0;
    out.target_speed_x10 = 0;
    out.valid = false;
    out.is_dual_lane = false;
    out.pixels_used = 0;
    out.curve_angle_deg = 0.0f;
    out.curvature = 0.0f;
    out.speed_factor = 0;
    out.detector_mode = 0;

    const auto reset_speed = [this]() {
        target_speed_initialized_ = false;
        speed_state_ = SpeedState::STRAIGHT;
    };

    if (bgr.empty() || bgr.cols < 100 || bgr.rows < 100) {
        reset_speed();
        return;
    }

    const int W = bgr.cols;
    const int H = bgr.rows;
    const int xmid_default = W / 2;

    // Mọi phép chiếu ngược IPM đều cần kích thước ảnh gốc thực tế, không phải
    // hằng số FRAME_W/FRAME_H: camera có thể trả về kích thước khác.
    frame_w_ = W;
    frame_h_ = H;

    xmid_scan_ = std::max(0, std::min(xmid_scan_, W - 1));

    if (!std::isfinite(current_speed_kmh)) current_speed_kmh = 0.0f;
    current_speed_kmh = clamp_float(current_speed_kmh, 0.0f, 20.0f);

    float roi_factor = roi_factor_dual_;
    if (current_speed_kmh > ROI_SPEED_START) {
        const float diff = current_speed_kmh - ROI_SPEED_START;
        roi_factor = clamp_float(
            roi_factor_dual_ - diff * ROI_SPEED_GAIN,
            ROI_FACTOR_FAST_MIN, roi_factor_dual_);
    }

    const int y0_dual = std::max(0, std::min(
        static_cast<int>(std::round(H * roi_factor)), H - 1));
    const int y0_single = std::max(0, std::min(
        static_cast<int>(std::round(H * roi_factor_single_)), H - 1));
    const int y1 = H - 1;

    if (y0_dual >= y1) {
        reset_speed();
        return;
    }

    const float width_scale = static_cast<float>(W) / static_cast<float>(FRAME_W);

    // -----------------------------------------------------------------------
    // NHÁNH CHÍNH: mask tự thích nghi sáng + IPM
    // -----------------------------------------------------------------------
    bool is_dual_lane = false;
    bool ipm_used = false;

    // Đọc tham số mask MỘT LẦN cho cả frame để không phải khoá ở nhiều chỗ
    // và để các bước sau dùng đúng cùng một bộ tham số với bước dựng mask.
    const lane_mask::Options mopts = opts();

    if (lane_mask::build_lane_mask(bgr, mopts, mask_, &mask_result_)) {
        out.mask = mask_;
        ipm_used = detect_lanes_ipm(
            mask_, y0_dual, y0_single, y1, width_scale, is_dual_lane, out);
    }

    if (!ipm_used) {
        // Nhánh dự phòng: Canny + quét dòng trên dải ROI gần xe
        leftPts_.clear();
        rightPts_.clear();
        is_dual_lane = false;
        bird_valid_ = false;

        const Rect roi_rect(0, y0_dual, W, H - y0_dual);
        const Mat roi = bgr(roi_rect);

        // Ảnh xám chỉ cần cho nhánh dự phòng. Bản cũ tính cả gray_full_ và
        // gray_ MỖI frame kể cả khi IPM thành công, tốn ~0.5-1.0 ms/frame.
        if (roi.channels() == 1) {
            gray_ = roi;
        } else if (roi.channels() == 3) {
            cvtColor(roi, gray_, COLOR_BGR2GRAY);
        } else if (roi.channels() == 4) {
            cvtColor(roi, gray_, COLOR_BGRA2GRAY);
        }

        if (gray_.channels() == 1) {
            detect_lanes_scanline(roi, width_scale, y0_dual, y0_single, y1, is_dual_lane);
        }

        out.confidence = LaneConfidence::WEAK;
    } else {
        median_smooth(leftPts_);
        median_smooth(rightPts_);
        out.detector_mode = 1;
    }

    out.is_dual_lane = is_dual_lane;
    out.left = leftPts_;
    out.right = rightPts_;
    out.pixels_used = static_cast<int>(leftPts_.size() + rightPts_.size());

    // Màu vạch: lấy trên mask 320x200 thay vì full-frame 640x400 như bản cũ.
    // Không phụ thuộc out.raw (ảnh gốc chỉ clone ở cuối hàm), nên nhánh này
    // chạy được thay vì bị bỏ qua do out.raw còn rỗng.
    if constexpr (ENABLE_LANE_COLOR_SAMPLE) {
        if (!mask_.empty() && mask_.size() == Size(mopts.work_w, mopts.work_h)) {
            Mat small;
            resize(bgr, small, mask_.size(), 0, 0, INTER_AREA);
            const Scalar mean_bgr = mean(small, mask_);
            if (mean_bgr[0] > 1.0 || mean_bgr[1] > 1.0 || mean_bgr[2] > 1.0) {
                out.lane_color_bgr = mean_bgr;
                const double luma = 0.114 * mean_bgr[0] +
                    0.587 * mean_bgr[1] + 0.299 * mean_bgr[2];
                out.lane_color_name = luma < 128.0 ? "black" : "white";
            }
        }
    }

    const bool has_left = leftPts_.size() >= static_cast<size_t>(MIN_FINAL_POINTS);
    const bool has_right = rightPts_.size() >= static_cast<size_t>(MIN_FINAL_POINTS);

    const float slope_left = has_left ? calculate_slope(leftPts_) : 0.0f;
    const float slope_right = has_right ? calculate_slope(rightPts_) : 0.0f;
    const float dominant_slope =
        (std::abs(slope_left) > std::abs(slope_right)) ? slope_left : slope_right;

    out.curve_angle_deg =
        std::atan(dominant_slope) * 180.0f / static_cast<float>(CV_PI);
    out.curvature = std::fabs(dominant_slope);

    if (is_dual_lane && has_left && has_right) {
        out.center.clear();
        size_t i = 0, j = 0;
        while (i < leftPts_.size() && j < rightPts_.size()) {
            if (leftPts_[i].y == rightPts_[j].y) {
                const int center_x = (leftPts_[i].x + rightPts_[j].x) / 2;
                out.center.emplace_back(center_x, leftPts_[i].y);
                ++i; ++j;
            } else if (leftPts_[i].y < rightPts_[j].y) {
                ++i;
            } else {
                ++j;
            }
        }

        if (!out.center.empty()) {
            const int center_x = median_tail_x(out.center, MEDIAN_TAIL_POINTS);
            const int raw_dev = center_x - xmid_default;
            int slope_offset = calculate_slope_offset(dominant_slope);

            const int recovery_threshold =
                static_cast<int>(std::round(95.0f * width_scale));
            if (std::abs(raw_dev) > recovery_threshold) {
                slope_offset = 0;
            }

            out.dev_final_px = clamp_int16(raw_dev + slope_offset);
            out.valid = true;
        }
    } else if (has_left || has_right) {
        const bool use_left = has_left && !has_right
            ? true
            : (!has_left && has_right ? false : leftPts_.size() >= rightPts_.size());
        const auto& lane = use_left ? leftPts_ : rightPts_;

        if (!lane.empty()) {
            const int lane_x = median_tail_x(lane, MEDIAN_TAIL_POINTS);
            const float lane_width = clamp_float(
                lane_width_est_px_,
                MIN_LANE_WIDTH_PX * width_scale,
                MAX_LANE_WIDTH_PX * width_scale);

            float estimated_center = static_cast<float>(lane_x);
            if (use_left) estimated_center += lane_width * 0.5f;
            else estimated_center -= lane_width * 0.5f;

            estimated_center = clamp_float(
                estimated_center, 0.0f, static_cast<float>(W - 1));

            const int raw_dev =
                static_cast<int>(std::round(estimated_center)) - xmid_default;
            out.dev_final_px = clamp_int16(raw_dev);
            out.valid = true;
        }
    }

    if (out.valid) {
        const int abs_dev = std::abs(static_cast<int>(out.dev_final_px));

        if (abs_dev < FWD_THRESHOLD_PX) out.camera_cmd = "FWD";
        else if (out.dev_final_px < 0) out.camera_cmd = "LEFT";
        else out.camera_cmd = "RIGHT";

        const uint8_t requested_speed = calculate_target_speed(dominant_slope, abs_dev);

        if (!target_speed_initialized_) {
            target_speed_x10_filtered_ = static_cast<float>(requested_speed);
            target_speed_initialized_ = true;
        } else {
            target_speed_x10_filtered_ = ALPHA_TARGET_SPEED *
                static_cast<float>(requested_speed) +
                (1.0f - ALPHA_TARGET_SPEED) * target_speed_x10_filtered_;
        }

        target_speed_x10_filtered_ =
            clamp_float(target_speed_x10_filtered_, 0.0f, 255.0f);
        out.target_speed_x10 =
            static_cast<uint8_t>(std::round(target_speed_x10_filtered_));

        out.speed_factor = static_cast<uint8_t>(std::round(
            100.0f * static_cast<float>(out.target_speed_x10) /
            static_cast<float>(SPEED_STRAIGHT_X10)));
        out.speed_factor =
            static_cast<uint8_t>(std::min(100, static_cast<int>(out.speed_factor)));

        // Tín hiệu lái yếu -> hạ độ tin cậy, để node điều khiển có thể giảm
        // tốc thay vì tin hoàn toàn.
        if (out.confidence == LaneConfidence::STRONG && abs_dev > SHARP_ENTER_DEV_PX) {
            out.confidence = LaneConfidence::GOOD;
        }
    } else {
        out.camera_cmd = "STOP";
        out.dev_final_px = 0;
        out.target_speed_x10 = 0;
        out.speed_factor = 0;
        target_speed_x10_filtered_ = static_cast<float>(SPEED_STRAIGHT_X10);
        target_speed_initialized_ = false;
        speed_state_ = SpeedState::STRAIGHT;
    }

    // -----------------------------------------------------------------------
    // ĐẦU RA ẢNH
    // Bản cũ clone bgr 3 lần mỗi frame (out.raw, out.vis, overlay trong
    // draw_lane_band) = 2.3 MB memcpy. Nay chỉ clone một lần cho raw và một
    // lần cho vis, overlay chỉ vẽ trong ROI của băng làn.
    // -----------------------------------------------------------------------
    out.raw = bgr.clone();

    if constexpr (ENABLE_VISUALIZATION) {
        const int y0_used = is_dual_lane ? y0_dual : y0_single;
        out.roi_y0 = y0_used;

        out.vis = bgr.clone();
        line(out.vis, Point(xmid_default, y0_used), Point(xmid_default, y1),
             Scalar(255, 255, 255), 1, LINE_AA);
        line(out.vis, Point(xmid_scan_, y0_used), Point(xmid_scan_, y1),
             Scalar(0, 255, 255), 1, LINE_AA);

        if (out.detector_mode == 1) {
            const Point tl(xmid_default + static_cast<int>((IPM_TL_X - 0.5f) * W), y0_dual);
            const Point tr(xmid_default + static_cast<int>((IPM_TR_X - 0.5f) * W), y0_dual);
            const Point br(xmid_default + static_cast<int>((IPM_BR_X - 0.5f) * W), y1);
            const Point bl(xmid_default + static_cast<int>((IPM_BL_X - 0.5f) * W), y1);
            const std::vector<Point> ipm_quad{tl, tr, br, bl};
            polylines(out.vis, ipm_quad, true, Scalar(0, 200, 255), 1, LINE_AA);
        }

        if (!out.left.empty()) polylines(out.vis, out.left, false, Scalar(0, 0, 255), 2, LINE_AA);
        if (!out.right.empty()) polylines(out.vis, out.right, false, Scalar(0, 0, 255), 2, LINE_AA);
        if (!out.center.empty()) polylines(out.vis, out.center, false, Scalar(0, 255, 0), 2, LINE_AA);

        draw_lane_band(out.vis);
    } else {
        out.roi_y0 = is_dual_lane ? y0_dual : y0_single;
        out.vis.release();
    }

    if constexpr (ENABLE_MASK_OVERLAY) {
        if (!out.mask.empty()) {
            Mat mask_full;
            resize(out.mask, mask_full, out.vis.size(), 0, 0, INTER_NEAREST);

            Mat mask_bgr;
            cvtColor(mask_full, mask_bgr, COLOR_GRAY2BGR);

            Mat tint(out.vis.size(), CV_8UC3, Scalar(0, 255, 0));
            bitwise_and(tint, mask_bgr, tint);
            addWeighted(tint, 0.35, out.vis, 1.0, 0.0, out.vis);
        }
    }

    if constexpr (ENABLE_DEBUG_LOG) {
        if ((debug_frame_count_++ % static_cast<uint64_t>(DEBUG_LOG_EVERY)) == 0) {
            std::cout << "[CameraLane] mode=" << static_cast<int>(out.detector_mode)
                      << " conf=" << lane_confidence_name(out.confidence)
                      << " dual=" << (is_dual_lane ? 1 : 0)
                      << " valid=" << (out.valid ? 1 : 0)
                      << " pts=" << out.pixels_used
                      << " roi=" << y0_dual << "/" << y0_single
                      << " lane_w=" << lane_width_est_px_
                      << " bird_w=" << bird_lane_width_
                      << " p_lo/hi=" << mask_result_.dark_threshold << "/"
                      << mask_result_.light_threshold
                      << " sobel_p99=" << mask_result_.sobel_p99
                      << " blobs=" << mask_result_.kept_blobs
                      << " dev=" << out.dev_final_px
                      << " cmd=" << out.camera_cmd
                      << " speed=" << static_cast<int>(out.target_speed_x10)
                      << " ms=" << out.processing_ms
                      << "\n";
        }
    }
}
