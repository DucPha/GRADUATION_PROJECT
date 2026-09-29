#include "camera_lane.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>

using namespace cv;

// ============================================================================
// CONSTRUCTOR
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

    work_output_.left.reserve(128);
    work_output_.right.reserve(128);
    work_output_.center.reserve(128);

    latest_output_.left.reserve(128);
    latest_output_.right.reserve(128);
    latest_output_.center.reserve(128);

    morph_kernel_ = getStructuringElement(MORPH_RECT, Size(3, 3));
}

// ============================================================================
// DESTRUCTOR
// ============================================================================

CameraLane::~CameraLane() {
    stop();
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
#ifdef _WIN32
    cap_.set(CAP_PROP_AUTO_EXPOSURE, 0.25);
    cap_.set(CAP_PROP_EXPOSURE, -6);
#endif

    const int actual_w = static_cast<int>(std::round(cap_.get(CAP_PROP_FRAME_WIDTH)));
    const int actual_h = static_cast<int>(std::round(cap_.get(CAP_PROP_FRAME_HEIGHT)));
    const double actual_fps = cap_.get(CAP_PROP_FPS);

    const int pool_w = actual_w > 0 ? actual_w : FRAME_W;
    const int pool_h = actual_h > 0 ? actual_h : FRAME_H;

    std::cout << "[CameraLane] " << pool_w << "x" << pool_h << " @ " << actual_fps << " FPS (BLACK LANE MODE)\n";

    {
        std::lock_guard<std::mutex> lock(frame_mtx_);
        latest_slot_ = -1;
        processing_slot_ = -1;

        for (size_t i = 0; i < FRAME_BUFFER_COUNT; ++i) {
            frame_pool_[i].image.create(pool_h, pool_w, CV_8UC3);
            frame_pool_[i].frame_id = 0;
        }
    }

    {
        std::lock_guard<std::mutex> lock(output_mtx_);
        latest_output_.left.clear();
        latest_output_.right.clear();
        latest_output_.center.clear();
        latest_output_.camera_cmd = "STOP";
        latest_output_.dev_final_px = 0;
        latest_output_.target_speed_x10 = 0;
        latest_output_.valid = false;
        latest_output_.pixels_used = 0;
        latest_output_.curve_angle_deg = 0.0f;
        latest_output_.curvature = 0.0f;
        latest_output_.speed_factor = 0;
        latest_output_.frame_id = 0;
        latest_output_.processing_ms = 0.0f;
        latest_output_.vis.release();
        latest_output_.raw.release();
        output_ready_ = false;
    }

    leftPts_prelim_.clear();
    rightPts_prelim_.clear();
    leftPts_.clear();
    rightPts_.clear();
    prelim_centers_.clear();

    gray_.release();
    blur_.release();
    edges_.release();

    xmid_scan_ = pool_w / 2;
    lane_width_est_px_ = DEFAULT_LANE_WIDTH_PX * (static_cast<float>(pool_w) / static_cast<float>(FRAME_W));
    target_speed_x10_filtered_ = static_cast<float>(SPEED_STRAIGHT_X10);
    target_speed_initialized_ = false;
    speed_state_ = SpeedState::STRAIGHT;
    current_speed_kmh_.store(0.0f, std::memory_order_relaxed);
    next_frame_id_ = 0;

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
        work_output_.processing_ms = std::chrono::duration<float, std::milli>(end_time - start_time).count();

        {
            std::lock_guard<std::mutex> lock(output_mtx_);
            latest_output_ = work_output_;
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
    if (pts.empty()) return 0;
    const int count = std::min(tail_count, static_cast<int>(pts.size()));
    std::array<int, 8> values{};
    const int n = std::min(count, static_cast<int>(values.size()));

    for (int i = 0; i < n; ++i) {
        values[i] = pts[pts.size() - n + i].x;
    }
    std::sort(values.begin(), values.begin() + n);
    return values[n / 2];
}

int CameraLane::median_value(std::vector<int>& values) {
    if (values.empty()) return 0;
    const size_t mid = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + mid, values.end());
    return values[mid];
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

    const float normalized = clamp_float((magnitude - SLOPE_OFFSET_START) / (SLOPE_OFFSET_END - SLOPE_OFFSET_START), 0.0f, 1.0f);
    const float offset = SLOPE_OFFSET_MIN_PX + normalized * (SLOPE_OFFSET_MAX_PX - SLOPE_OFFSET_MIN_PX);
    const int offset_i = static_cast<int>(std::round(offset));

    return dominant_slope > 0.0f ? -offset_i : offset_i;
}

uint8_t CameraLane::calculate_target_speed(float dominant_slope, int abs_deviation) {
    const float slope_mag = std::fabs(dominant_slope);

    const bool sharp_enter = slope_mag >= SHARP_ENTER_SLOPE || abs_deviation >= SHARP_ENTER_DEV_PX;
    const bool sharp_exit = slope_mag < SHARP_EXIT_SLOPE && abs_deviation < SHARP_EXIT_DEV_PX;
    const bool curve_enter = slope_mag >= CURVE_ENTER_SLOPE || abs_deviation >= CURVE_ENTER_DEV_PX;
    const bool curve_exit = slope_mag < CURVE_EXIT_SLOPE && abs_deviation < CURVE_EXIT_DEV_PX;

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

// ============================================================================
// DETECT LANES (BLACK LANE + HIGH SPEED)
// ============================================================================

void CameraLane::detect_lanes(const cv::Mat& bgr, float current_speed_kmh, LaneOutput& out) {
    out.left.clear();
    out.right.clear();
    out.center.clear();
    out.camera_cmd = "STOP";
    out.dev_final_px = 0;
    out.target_speed_x10 = 0;
    out.valid = false;
    out.pixels_used = 0;
    out.curve_angle_deg = 0.0f;
    out.curvature = 0.0f;
    out.speed_factor = 0;
    out.vis.release();
    out.raw.release();

    if (bgr.empty()) {
        target_speed_initialized_ = false;
        speed_state_ = SpeedState::STRAIGHT;
        return;
    }

    const int W = bgr.cols;
    const int H = bgr.rows;
    if (W < 100 || H < 100) {
        target_speed_initialized_ = false;
        speed_state_ = SpeedState::STRAIGHT;
        return;
    }

    const int xmid_default = W / 2;
    xmid_scan_ = std::max(0, std::min(xmid_scan_, W - 1));

    if (!std::isfinite(current_speed_kmh)) current_speed_kmh = 0.0f;
    current_speed_kmh = clamp_float(current_speed_kmh, 0.0f, 20.0f);

    float roi_factor = ROI_FACTOR_SLOW;
    if (current_speed_kmh > ROI_SPEED_START) {
        const float diff = current_speed_kmh - ROI_SPEED_START;
        roi_factor = clamp_float(ROI_FACTOR_SLOW - diff * ROI_SPEED_GAIN, ROI_FACTOR_FAST_MIN, ROI_FACTOR_SLOW);
    }

    const int y0_dual = std::max(0, std::min(static_cast<int>(std::round(H * roi_factor)), H - 1));
    const int y0_single = std::max(0, std::min(static_cast<int>(std::round(H * SINGLE_ROI_FACTOR)), H - 1));
    const int y1 = H - 1;

    if (y0_dual >= y1) {
        target_speed_initialized_ = false;
        speed_state_ = SpeedState::STRAIGHT;
        return;
    }

    const Rect roi_rect(0, y0_dual, W, H - y0_dual);
    const Mat roi = bgr(roi_rect);

    if (roi.channels() == 1) gray_ = roi;
    else if (roi.channels() == 3) cvtColor(roi, gray_, COLOR_BGR2GRAY);
    else if (roi.channels() == 4) cvtColor(roi, gray_, COLOR_BGRA2GRAY);
    else {
        target_speed_initialized_ = false;
        speed_state_ = SpeedState::STRAIGHT;
        return;
    }

    GaussianBlur(gray_, blur_, Size(BLUR_KERNEL, BLUR_KERNEL), 0);
    Canny(blur_, edges_, CANNY_LOW, CANNY_HIGH);
    morphologyEx(edges_, edges_, MORPH_CLOSE, morph_kernel_, Point(-1, -1), 1);

    leftPts_prelim_.clear();
    rightPts_prelim_.clear();
    prelim_centers_.clear();

    const float width_scale = static_cast<float>(W) / static_cast<float>(FRAME_W);
    const float min_width = MIN_LANE_WIDTH_PX * width_scale;
    const float max_width = MAX_LANE_WIDTH_PX * width_scale;

    int xmid_candidate = xmid_scan_;
    double width_sum = 0.0;
    int overlap_count = 0;

    for (int y = y0_dual; y <= y1; y += SCAN_STEP) {
        const int local_y = y - y0_dual;
        const uchar* row = edges_.ptr<uchar>(local_y);
        int left = -1, right = -1;

        for (int x = xmid_candidate; x >= 0; --x) {
            if (row[x] != 0) { left = x; break; }
        }
        for (int x = xmid_candidate; x < W; ++x) {
            if (row[x] != 0) { right = x; break; }
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

    const bool has_left_prelim = leftPts_prelim_.size() >= MIN_PRELIM_POINTS;
    const bool has_right_prelim = rightPts_prelim_.size() >= MIN_PRELIM_POINTS;
    bool is_dual_lane = false;
    float avg_lane_width = 0.0f;

    if (has_left_prelim && has_right_prelim && overlap_count >= MIN_COMMON_POINTS) {
        avg_lane_width = static_cast<float>(width_sum / static_cast<double>(overlap_count));
        if (avg_lane_width >= min_width && avg_lane_width <= max_width) {
            is_dual_lane = true;
        }
    }

    if (is_dual_lane) {
        lane_width_est_px_ = ALPHA_LANE_WIDTH * avg_lane_width + (1.0f - ALPHA_LANE_WIDTH) * lane_width_est_px_;
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
        const int diff = xmid_default - xmid_candidate;
        if (std::abs(diff) > 5) {
            xmid_candidate += diff > 0 ? 5 : -5;
        } else {
            xmid_candidate = xmid_default;
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

    out.left = leftPts_;
    out.right = rightPts_;
    out.pixels_used = static_cast<int>(leftPts_.size() + rightPts_.size());

    const bool has_left = leftPts_.size() >= MIN_FINAL_POINTS;
    const bool has_right = rightPts_.size() >= MIN_FINAL_POINTS;

    const float slope_left = has_left ? calculate_slope(leftPts_) : 0.0f;
    const float slope_right = has_right ? calculate_slope(rightPts_) : 0.0f;
    const float dominant_slope = (std::abs(slope_left) > std::abs(slope_right)) ? slope_left : slope_right;

    out.curve_angle_deg = std::atan(dominant_slope) * 180.0f / static_cast<float>(CV_PI);
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

            const int recovery_threshold = static_cast<int>(std::round(95.0f * width_scale));
            if (std::abs(raw_dev) > recovery_threshold) {
                slope_offset = 0;
            }

            out.dev_final_px = clamp_int16(raw_dev + slope_offset);
            out.valid = true;
        }
    } else if (has_left || has_right) {
        const bool use_left = has_left && !has_right ? true : (!has_left && has_right ? false : leftPts_.size() >= rightPts_.size());
        const auto& lane = use_left ? leftPts_ : rightPts_;

        if (!lane.empty()) {
            const int lane_x = median_tail_x(lane, MEDIAN_TAIL_POINTS);
            const float lane_width = clamp_float(lane_width_est_px_, 100.0f * width_scale, 500.0f * width_scale);
            float estimated_center = static_cast<float>(lane_x);

            if (use_left) estimated_center += lane_width * 0.5f;
            else estimated_center -= lane_width * 0.5f;

            estimated_center = clamp_float(estimated_center, 0.0f, static_cast<float>(W - 1));
            const int raw_dev = static_cast<int>(std::round(estimated_center)) - xmid_default;
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
            target_speed_x10_filtered_ = ALPHA_TARGET_SPEED * static_cast<float>(requested_speed) + (1.0f - ALPHA_TARGET_SPEED) * target_speed_x10_filtered_;
        }

        target_speed_x10_filtered_ = clamp_float(target_speed_x10_filtered_, 0.0f, 255.0f);
        out.target_speed_x10 = static_cast<uint8_t>(std::round(target_speed_x10_filtered_));

        out.speed_factor = static_cast<uint8_t>(std::round(100.0f * static_cast<float>(out.target_speed_x10) / static_cast<float>(SPEED_STRAIGHT_X10)));
        out.speed_factor = static_cast<uint8_t>(std::min(100, static_cast<int>(out.speed_factor)));
    } else {
        out.camera_cmd = "STOP";
        out.dev_final_px = 0;
        out.target_speed_x10 = 0;
        out.speed_factor = 0;
        target_speed_x10_filtered_ = static_cast<float>(SPEED_STRAIGHT_X10);
        target_speed_initialized_ = false;
        speed_state_ = SpeedState::STRAIGHT;
    }

    out.raw = bgr.clone();

    if constexpr (ENABLE_VISUALIZATION) {
        out.vis = bgr.clone();
        line(out.vis, Point(xmid_default, y0_used), Point(xmid_default, y1), Scalar(255, 255, 255), 1, LINE_AA);
        line(out.vis, Point(xmid_scan_, y0_used), Point(xmid_scan_, y1), Scalar(0, 255, 255), 1, LINE_AA);

        if (!out.left.empty()) polylines(out.vis, out.left, false, Scalar(0, 0, 255), 2, LINE_AA);
        if (!out.right.empty()) polylines(out.vis, out.right, false, Scalar(0, 0, 255), 2, LINE_AA);
        if (!out.center.empty()) polylines(out.vis, out.center, false, Scalar(0, 255, 0), 2, LINE_AA);
    } else {
        out.vis.release();
    }
}