#include "camera_lane.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>

using namespace cv;

// ============================================================================
// CONSTRUCTOR & DESTRUCTOR
// ============================================================================

CameraLane::CameraLane(
    int device_index,
    int fps,
    bool use_v4l2,
    bool auto_exposure,
    int exposure
)
    : device_index_(device_index),
      fps_(fps),
      use_v4l2_(use_v4l2),
      auto_exposure_(auto_exposure),
      exposure_(exposure),
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
    bird_nonzero_.reserve(4096);
    bird_col_sum_.assign(BIRD_W, 0);
    bird_win_sum_.assign(BIRD_W, 0);
    blob_keep_lut_.reserve(256);

    work_output_.left.reserve(128);
    work_output_.right.reserve(128);
    work_output_.center.reserve(128);

    latest_output_.left.reserve(128);
    latest_output_.right.reserve(128);
    latest_output_.center.reserve(128);

    build_scaled_kernels();

    clahe_ = createCLAHE(CLAHE_CLIP_LIMIT, Size(CLAHE_GRID_X, CLAHE_GRID_Y));
}

// Kích thước kernel phải co lại theo px_scale_ thì mới giữ được độ dày mảnh
// và độ dài vệt của làn khi chạy ở 352x288 thay vì 640x480. Ví dụ kernel 15px
// ở khung 640 rộng chỉ còn ~8px ở khung 352 — nếu giữ nguyên, lane band bị
// nở gấp đôi và 2 làn dính vào nhau.
void CameraLane::build_scaled_kernels() {
    const int k_close_x = std::max(1, scale_px(MASK_CLOSE_X, px_scale_));
    const int k_close_y = std::max(1, scale_px(MASK_CLOSE_KERNEL, px_scale_));
    const int k_open = std::max(1, scale_px(MASK_OPEN_KERNEL, px_scale_));

    // Kernel phải có số cạnh LẺ: OpenCV yêu cầu vị trí tâm rõ ràng, kernel chẵn
    // sẽ bị làm tròn vị trí tâm và méo lệch so với ý đồ.
    const auto odd = [](int v) { return (v % 2 == 0) ? v + 1 : v; };

    morph_kernel_ = getStructuringElement(MORPH_RECT, Size(3, 3));
    mask_close_kernel_ =
        getStructuringElement(MORPH_RECT, Size(odd(k_close_x), odd(k_close_y)));
    mask_open_kernel_ =
        getStructuringElement(MORPH_RECT, Size(odd(k_open), odd(k_open)));
}

int CameraLane::scale_px(float design_px, float scale) {
    return std::max(1, static_cast<int>(std::lround(design_px * scale)));
}

float CameraLane::scale_pxf(float design_px, float scale) {
    return design_px * scale;
}

CameraLane::~CameraLane() {
    stop();
}

// MỞ CAMERA / DÒ THIẾT BỊ
// ============================================================================

// Camera này KHÔNG có manual exposure (kiểm tra bằng `v4l2-ctl --list-ctrls`:
// chỉ có brightness/contrast/saturation/hue/gamma/gain/power_line_frequency/
// sharpness, KHÔNG có exposure_auto hay exposure_absolute). Đo thực nghiệm
// cho thấy `brightness` không đổi được luma đầu ra (đều ra 90) vì
// auto-exposure ghi đè hoàn toàn, nên không thể ép shutter ngắn bằng phần mềm.
//
// Kết luận: tần số thực tế bị giới hạn bởi BĂNG THÔNG USB isochronous của
// camera, không phải bởi ánh sáng. Số đo trên chính máy này:
//     640x480 YUYV -> 18.4 MB/s ở 30fps -> chỉ ~6 FPS, hay rớt khỏi USB
//     352x288 YUYV -> 13.2 MB/s ở 63fps -> 63 FPS, 0 lỗi trong 30 giây
// Nên mặc định dùng CAP_W x CAP_H = 352x288.
static void apply_v4l2_controls(int index) {
    const std::string dev = "/dev/video" + std::to_string(index);
    // power_line_frequency=0: tắt chống nhấp nháy đèn 50/60Hz.
    // Các control khác (đã đo) không đổi được luma vì AE ghi đè, nên không
    // đặt để khỏi rối. Gộp tất cả vào MỘT lệnh để khỏi phải spawn 8 tiến
    // trình mỗi khi khởi động.
    const std::string cmd =
        "v4l2-ctl -d " + dev + " -c power_line_frequency=0 2>/dev/null";
    const int rc = std::system(cmd.c_str());
    (void)rc;
}

int CameraLane::detect_device_index(int preferred_index) {
    // probe: node capture thật phải trả về 1 frame đúng kích thước mới coi
    // là dùng được. /dev/video1 của camera này mở được nhưng KHÔNG có format.
    auto probe = [](int idx) -> bool {
        cv::VideoCapture c(idx, CAP_V4L2);
        if (!c.isOpened()) return false;
        c.set(CAP_PROP_FOURCC, VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
        c.set(CAP_PROP_FRAME_WIDTH, CAP_W);
        c.set(CAP_PROP_FRAME_HEIGHT, CAP_H);
        bool ok = false;
        for (int i = 0; i < 3 && !ok; ++i) {
            cv::Mat f;
            ok = c.read(f) && !f.empty();
        }
        c.release();
        return ok;
    };

    // Camera này liên tục rớt khỏi bus USB rồi cắm lại (Device 013 -> 016,
    // video0 <-> video1). Trong lúc tái liệt kê, MỌI node đều không đọc được,
    // nên phải thử lại nhiều vòng thay vì kết luận "không có camera" ngay.
    const int max_rounds = 5;
    const int sleep_ms = 400;

    for (int round = 0; round < max_rounds; ++round) {
        if (preferred_index >= 0 && probe(preferred_index)) return preferred_index;

        // Dò tất cả /dev/videoN. Ưu tiên index nhỏ trước cho ổn định.
        for (int idx = 0; idx < 16; ++idx) {
            if (idx == preferred_index) continue;
            if (probe(idx)) return idx;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
    }

    return -1;
}

bool CameraLane::open_device(int index, int want_w, int want_h) {
    const int backend = use_v4l2_ ? CAP_V4L2 : CAP_ANY;

    cap_.release();
    if (!cap_.open(index, backend)) {
        last_error_ = "cannot open /dev/video" + std::to_string(index);
        return false;
    }

    // Thứ tự quan trọng: FOURCC -> W/H -> FPS -> BUFFERSIZE.
    // Camera không hỗ trợ MJPG (--list-formats-ext đã xác nhận), nên YUYV là
    // lựa chọn duy nhất. BUFFERSIZE=1 giữ đúng 1 frame trong hàng đợi để
    // luôn đọc frame MỚI NHẤT thay vì frame cũ (giảm latency).
    cap_.set(CAP_PROP_FOURCC, VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
    cap_.set(CAP_PROP_FRAME_WIDTH, want_w);
    cap_.set(CAP_PROP_FRAME_HEIGHT, want_h);
    cap_.set(CAP_PROP_FPS, fps_);
    cap_.set(CAP_PROP_BUFFERSIZE, 1);

    if (auto_exposure_) {
        cap_.set(CAP_PROP_AUTO_EXPOSURE, 0.75);   // 0.75 = auto
    } else {
        cap_.set(CAP_PROP_AUTO_EXPOSURE, 0.25);   // 0.25 = manual
        cap_.set(CAP_PROP_EXPOSURE, exposure_);
    }

    // Xác nhận camera thực sự trả về đúng kích thước yêu cầu; nếu không thì
    // thử bước kế tiếp (fallback) thay vì âm thầm chạy sai độ phân giải.
    const int got_w = static_cast<int>(std::round(cap_.get(CAP_PROP_FRAME_WIDTH)));
    const int got_h = static_cast<int>(std::round(cap_.get(CAP_PROP_FRAME_HEIGHT)));
    if (got_w < 32 || got_h < 32) {
        last_error_ = "camera returned empty format";
        cap_.release();
        return false;
    }

    actual_w_ = got_w;
    actual_h_ = got_h;
    px_scale_ = static_cast<float>(got_w) / static_cast<float>(FRAME_W);
    build_scaled_kernels();
    return true;
}

// ============================================================================
// START & STOP
// ============================================================================

bool CameraLane::start() {
    if (running_.load(std::memory_order_relaxed)) return true;

    last_error_.clear();
    actual_measured_fps_ = 0.0;

    // -----------------------------------------------------------------
    // 1. DÒ THIẾT BỊ
    // -----------------------------------------------------------------
    // Camera hay rớt khỏi bus USB và cắm lại với số /dev/video KHÁC
    // (video0 -> video1 -> video0 ...). Vì vậy không được tin chỉ số cố
    // định; mặc định device_index = -1 => tự dò.
    const int found = detect_device_index(device_index_);
    if (found < 0) {
        last_error_ = "no usable UVC capture node found (/dev/video0..15)";
        std::cerr << "[CameraLane] ERROR: " << last_error_ << "\n";
        return false;
    }
    if (found != device_index_) {
        std::cout << "[CameraLane] Camera resolved to /dev/video" << found << "\n";
    }
    device_index_ = found;
    apply_v4l2_controls(device_index_);

    // -----------------------------------------------------------------
    // 2. THỬ TỪNG ĐỘ PHÂN GIẢI, LỚN NHẤT TRƯỚC ĐƯỢC
    // -----------------------------------------------------------------
    bool opened = false;
    for (int i = 0; i < RES_FALLBACK_COUNT; ++i) {
        const int w = RES_FALLBACKS[i].w;
        const int h = RES_FALLBACKS[i].h;
        if (open_device(device_index_, w, h)) {
            opened = true;
            break;
        }
        std::cerr << "[CameraLane] " << w << "x" << h << " failed: "
                  << last_error_ << "\n";
        // Mỗi lần thử tốn ~1s và camera hay biến mất khỏi bus giữa chừng.
        // Dò lại thiết bị thay vì cố mở tiếp index đã chết.
        const int again = detect_device_index(device_index_);
        if (again >= 0) {
            if (again != device_index_) {
                std::cout << "[CameraLane] Re-enumerated: /dev/video"
                          << again << "\n";
                device_index_ = again;
                apply_v4l2_controls(again);
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
        }
    }

    if (!opened) {
        std::cerr << "[CameraLane] ERROR: cannot open camera at any resolution\n";
        return false;
    }

    const int fourcc = static_cast<int>(cap_.get(CAP_PROP_FOURCC));
    const char fourcc_str[5] = {
        static_cast<char>(fourcc & 0xFF),
        static_cast<char>((fourcc >> 8) & 0xFF),
        static_cast<char>((fourcc >> 16) & 0xFF),
        static_cast<char>((fourcc >> 24) & 0xFF),
        '\0'
    };

    std::cout << "[CameraLane] /dev/video" << device_index_
              << " -> " << actual_w_ << "x" << actual_h_
              << " @ " << cap_.get(CAP_PROP_FPS) << " FPS"
              << " (FOURCC=" << fourcc_str
              << ", px_scale=" << px_scale_ << ")\n";

    // -----------------------------------------------------------------
    // 3. ĐO FPS THẬT + TỰ HẠ ĐỘ PHÂN GIẢI NẾU CHẬM
    // -----------------------------------------------------------------
    // Đo trên luồng thật thay vì tin CAP_PROP_FPS (driver báo số cấu hình,
    // không phải số frame thực sự tới).
    auto measure_fps = [this](double seconds) {
        cv::Mat tmp;
        int frames = 0;
        const auto t0 = std::chrono::steady_clock::now();
        while (std::chrono::duration<double>(
                   std::chrono::steady_clock::now() - t0).count() < seconds) {
            if (cap_.read(tmp) && !tmp.empty()) ++frames;
        }
        const double el = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
        return el > 0.0 ? frames / el : 0.0;
    };

    actual_measured_fps_ = measure_fps(1.5);
    std::cout << "[CameraLane] Measured: " << actual_measured_fps_ << " FPS\n";

    // Nếu đo ra thấp hơn yêu cầu quá nhiều, thử hạ độ phân giải tiếp cho tới
    // khi đạt yêu cầu. Đây là cơ chế tự chữa khi cắm vào cổng USB bận.
    for (int i = 1; i < RES_FALLBACK_COUNT; ++i) {
        if (actual_measured_fps_ >= fps_ * 0.8) break;
        if (RES_FALLBACKS[i].w >= actual_w_) continue;

        std::cout << "[CameraLane] " << actual_measured_fps_
                  << " FPS < target, trying " << RES_FALLBACKS[i].w
                  << "x" << RES_FALLBACKS[i].h << "\n";
        if (!open_device(device_index_, RES_FALLBACKS[i].w, RES_FALLBACKS[i].h)) {
            continue;
        }
        actual_measured_fps_ = measure_fps(1.5);
        std::cout << "[CameraLane]   -> " << actual_measured_fps_ << " FPS at "
                  << actual_w_ << "x" << actual_h_ << "\n";
    }

    if (actual_measured_fps_ < fps_ * 0.5) {
        std::cerr << "[CameraLane] WARNING: only " << actual_measured_fps_
                  << " FPS at " << actual_w_ << "x" << actual_h_
                  << " (target " << fps_ << ").\n"
                  << "   Likely USB 2.0 bandwidth contention (ESP32 serial + "
                  << "LiDAR on same hub).\n"
                  << "   Try: plug camera into a port NOT shared with other "
                  << "USB devices.\n";
    }

    {
        std::lock_guard<std::mutex> lock(frame_mtx_);
        shared_frame_.release();
        shared_frame_id_ = 0;
        new_frame_available_ = false;
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
        latest_output_.is_dual_lane = false;
        latest_output_.pixels_used = 0;
        latest_output_.curve_angle_deg = 0.0f;
        latest_output_.curvature = 0.0f;
        latest_output_.speed_factor = 0;
        latest_output_.frame_id = 0;
        latest_output_.processing_ms = 0.0f;
        latest_output_.vis.release();
        latest_output_.raw.release();
        latest_output_.binary_mask.release();
        latest_output_.bird_mask.release();
        output_ready_ = false;
    }

    leftPts_prelim_.clear();
    rightPts_prelim_.clear();
    leftPts_.clear();
    rightPts_.clear();
    prelim_centers_.clear();
    left_win_means_.clear();
    right_win_means_.clear();

    gray_.release();
    gray_full_.release();
    blur_.release();
    edges_.release();
    eq_.release();
    sobel_x_.release();
    mask_.release();
    mask_tmp_.release();
    bird_.release();
    m_ipm_.release();
    m_ipm_inv_.release();
    ipm_w_ = 0;
    ipm_h_ = 0;

    xmid_scan_ = actual_w_ / 2;
    lane_width_est_px_ =
        DEFAULT_LANE_WIDTH_PX * (static_cast<float>(actual_w_) / static_cast<float>(FRAME_W));
    bird_lane_width_ = 0.0f;
    bird_center_prev_ = static_cast<float>(BIRD_W) * 0.5f;
    bird_valid_ = false;
    track_run_w_[0] = 0.0f;
    track_run_w_[1] = 0.0f;
    target_speed_x10_filtered_ = static_cast<float>(SPEED_STRAIGHT_X10);
    target_speed_initialized_ = false;
    speed_state_ = SpeedState::STRAIGHT;
    lost_frame_count_ = 0;
    last_valid_dev_ = 0;
    last_valid_slope_ = 0.0f;
    bird_hood_y_ = BIRD_H;
    bird_valid_rows_ = BIRD_H;
    current_speed_kmh_.store(0.0f, std::memory_order_relaxed);
    next_frame_id_ = 0;
    debug_frame_count_ = 0;

    running_.store(true, std::memory_order_release);
    capture_thread_ = std::thread(&CameraLane::capture_thread, this);
    process_thread_ = std::thread(&CameraLane::process_thread, this);

    return true;
}

void CameraLane::stop() {
    if (!running_.exchange(false, std::memory_order_acq_rel)) {
        if (capture_thread_.joinable()) capture_thread_.join();
        if (process_thread_.joinable()) process_thread_.join();
        if (cap_.isOpened()) cap_.release();
        return;
    }

    frame_ready_cv_.notify_all();

    if (capture_thread_.joinable()) capture_thread_.join();
    if (process_thread_.joinable()) process_thread_.join();
    if (cap_.isOpened()) cap_.release();

    std::cout << "[CameraLane] Stopped.\n";
}

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
    return new_frame_available_ ? 1u : 0u;
}

// ============================================================================
// TRIPLE-BUFFER SWAP PIPELINE
// ============================================================================

void CameraLane::capture_thread() {
    cv::Mat local_cap;
    int fail_count = 0;

    while (running_.load(std::memory_order_relaxed)) {
        // grab() + retrieve() tách biệt: grab() non-blocking, retrieve() copy data
        // Giảm latency so với read() = grab()+retrieve() nội bộ
        if (!cap_.grab()) {
            ++fail_count;
            capture_fail_count_.store(fail_count, std::memory_order_relaxed);

            if (fail_count == 10) {
                std::cerr << "[CameraLane] Camera grab failed x10, re-opening...\n";
                cap_.release();
                std::this_thread::sleep_for(std::chrono::milliseconds(200));

                // Camera này hay RỚT KHỎI BUS USB rồi cắm lại với số
                // /dev/video khác. Mở lại đúng index cũ sẽ hỏng, nên phải DÒ
                // LẠI từ đầu rồi mới mở ở độ phân giải đang chạy.
                const int w = actual_w_ > 0 ? actual_w_ : CAP_W;
                const int h = actual_h_ > 0 ? actual_h_ : CAP_H;
                const int idx = detect_device_index(device_index_);
                if (idx >= 0) {
                    device_index_ = idx;
                    apply_v4l2_controls(idx);
                    if (open_device(idx, w, h)) {
                        std::cout << "[CameraLane] Re-opened /dev/video" << idx
                                  << " at " << actual_w_ << "x" << actual_h_ << "\n";
                    } else {
                        std::cerr << "[CameraLane] Re-open failed: " << last_error_ << "\n";
                    }
                } else {
                    std::cerr << "[CameraLane] Camera gone from USB bus, "
                              << "retrying scan...\n";
                }
                fail_count = 0;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        if (!cap_.retrieve(local_cap) || local_cap.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        fail_count = 0;
        capture_fail_count_.store(0, std::memory_order_relaxed);

        const uint64_t fid = ++next_frame_id_;

        {
            std::lock_guard<std::mutex> lock(frame_mtx_);
            std::swap(shared_frame_, local_cap);
            shared_frame_id_ = fid;
            new_frame_available_ = true;
        }

        frame_ready_cv_.notify_one();
    }
}

void CameraLane::process_thread() {
    cv::Mat proc_frame;
    uint64_t proc_fid = 0;

    while (running_.load(std::memory_order_relaxed)) {
        {
            std::unique_lock<std::mutex> lock(frame_mtx_);
            // Chờ frame mới - không timeout để tránh lag 20ms
            frame_ready_cv_.wait(lock, [this]() {
                return !running_.load(std::memory_order_relaxed) || new_frame_available_;
            });

            if (!running_.load(std::memory_order_relaxed)) break;
            if (!new_frame_available_) continue;

            std::swap(proc_frame, shared_frame_);
            proc_fid = shared_frame_id_;
            new_frame_available_ = false;
        }

        if (proc_frame.empty()) continue;

        const float current_speed = current_speed_kmh_.load(std::memory_order_relaxed);
        const auto start_time = std::chrono::steady_clock::now();
        detect_lanes(proc_frame, current_speed, work_output_);
        const auto end_time = std::chrono::steady_clock::now();

        work_output_.frame_id = proc_fid;
        work_output_.processing_ms = std::chrono::duration<float, std::milli>(end_time - start_time).count();

        {
            std::lock_guard<std::mutex> lock(output_mtx_);
            latest_output_ = work_output_;
            output_ready_ = true;
        }
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
    const int n = std::max(0, std::min(count, std::min(
        static_cast<int>(pts.size()), MEDIAN_TAIL_POINTS_MAX)));
    if (n == 0) return 0;

    for (int i = 0; i < n; ++i) {
        values[static_cast<size_t>(i)] = pts[pts.size() - static_cast<size_t>(n) + static_cast<size_t>(i)].x;
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
    if (lane.size() < 6) return 0.0f;
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
    // offset cộng vào dev tính bằng pixel THỰC TẾ, nên phải quy đổi về px_scale_.
    const int offset_i = scale_px(offset, px_scale_);

    return dominant_slope > 0.0f ? -offset_i : offset_i;
}

// abs_deviation ở đây LUÔN được tính trong design space (640x480) — xem chỗ
// gọi trong detect_lanes(). Nhờ vậy toàn bộ ngưỡng bên dưới giữ nguyên ý nghĩa
// thiết kế, không phải sửa lại khi đổi CAP_W/CAP_H.
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
// POLYNOMIAL FIT (Matx33d + Cholesky)
// ============================================================================

float CameraLane::poly_eval(const float coef[3], float y) {
    const float yn = y / static_cast<float>(BIRD_H);
    return coef[0] + coef[1] * yn + coef[2] * yn * yn;
}

bool CameraLane::fit_poly(const std::vector<cv::Point>& pts, float coef[3]) {
    if (pts.size() < static_cast<size_t>(BIRD_MIN_FIT_POINTS)) return false;

    Matx33d A = Matx33d::zeros();
    Vec3d b = Vec3d::all(0);

    for (const auto& p : pts) {
        const double yn = static_cast<double>(p.y) / static_cast<double>(BIRD_H);
        const double yn2 = yn * yn;
        const double v[3] = {1.0, yn, yn2};

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                A(i, j) += v[i] * v[j];
            }
            b(i) += v[i] * static_cast<double>(p.x);
        }
    }

    Vec3d res;
    if (!solve(A, b, res, DECOMP_CHOLESKY)) {
        if (!solve(A, b, res, DECOMP_LU)) return false;
    }

    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(res(i))) return false;
        coef[i] = static_cast<float>(res(i));
    }

    return true;
}

bool CameraLane::fit_residual_ok(const std::vector<cv::Point>& pts, const float coef[3]) {
    if (pts.empty()) return false;

    double sum = 0.0;
    for (const auto& p : pts) {
        sum += std::abs(static_cast<double>(poly_eval(coef, static_cast<float>(p.y)) - static_cast<double>(p.x)));
    }

    return (sum / static_cast<double>(pts.size())) <= static_cast<double>(BIRD_MAX_FIT_RESIDUAL_PX);
}

// ============================================================================
// LỌC ẢNH
// ============================================================================

bool CameraLane::sobel_color_thresholding(const cv::Mat& bgr) {
    if (bgr.channels() == 1) return false;

    Mat source = bgr;
    if (bgr.channels() == 4) cvtColor(bgr, source, COLOR_BGRA2BGR);
    if (source.channels() != 3) return false;

    split(source, bgr_channels_);

    clahe_->apply(bgr_channels_[1], eq_);
    Sobel(eq_, sobel_x_, CV_16S, 1, 0, 3, 1.0, 0.0, BORDER_DEFAULT);
    convertScaleAbs(sobel_x_, sobel_x_);

    inRange(sobel_x_, SOBEL_MIN_THRESHOLD, SOBEL_MAX_THRESHOLD, mask_tmp_);
    inRange(bgr_channels_[2], LANE_MIN_VALUE, LANE_MAX_VALUE, mask_);
    bitwise_and(mask_tmp_, mask_, mask_);

    mask_bonnet(mask_);
    return true;
}

void CameraLane::mask_bonnet(cv::Mat& mask) {
    if (!BONNET_ENABLE || mask.empty()) return;

    const int w = mask.cols;
    const int h = mask.rows;

    const int x0 = std::max(0, static_cast<int>(BONNET_X0 * static_cast<float>(w)));
    const int x1 = std::min(w, static_cast<int>(BONNET_X1 * static_cast<float>(w)));
    const int y0 = std::max(0, static_cast<int>(BONNET_Y0 * static_cast<float>(h)));

    if (x1 > x0 && y0 < h) {
        rectangle(mask, Rect(x0, y0, x1 - x0, h - y0), Scalar(0), FILLED);
    }
}

bool CameraLane::filter_lane_blobs(const cv::Mat& gray_src) {
    const float width_scale = static_cast<float>(mask_.cols) / static_cast<float>(FRAME_W);
    const int min_area = static_cast<int>(std::round(MIN_BLOB_AREA * width_scale * width_scale));
    const int max_blob_width = static_cast<int>(std::round(MAX_BLOB_WIDTH_PX * width_scale));
    // Ngưỡng chiều cao và biên độ Sobel cũng phải co theo độ phân giải:
    // chiều cao blob là số pixel, còn biên độ Sobel giảm khi ảnh bị thu nhỏ.
    const int min_blob_height = scale_px(MIN_BLOB_HEIGHT, width_scale);
    const double sobel_peak_min =
        static_cast<double>(scale_px(SOBEL_X_PEAK_MIN, width_scale));
    const int adaptive_block = scale_px(ADAPTIVE_BLOCK, width_scale);

    morphologyEx(mask_, mask_, MORPH_CLOSE, mask_close_kernel_);
    morphologyEx(mask_, mask_, MORPH_OPEN, mask_open_kernel_);

    const int components = connectedComponentsWithStats(mask_, labels_, stats_, centroids_, 8, CV_32S);
    if (components <= 1) return false;

    blob_keep_lut_.assign(components, 0);
    int kept = 0;

    const bool check_sobel = (!sobel_x_.empty() && sobel_x_.size() == mask_.size());

    for (int i = 1; i < components; ++i) {
        const int area = stats_.at<int>(i, CC_STAT_AREA);
        const int height = stats_.at<int>(i, CC_STAT_HEIGHT);
        const int width = stats_.at<int>(i, CC_STAT_WIDTH);

        if (area < min_area || height < min_blob_height || width > max_blob_width) continue;

        if (check_sobel) {
            const int left = stats_.at<int>(i, CC_STAT_LEFT);
            const int top = stats_.at<int>(i, CC_STAT_TOP);
            const Rect bbox(left, top, width, height);

            double peak = 0.0;
            const Mat sobel_roi = sobel_x_(bbox);
            const Mat labels_roi = labels_(bbox);

            for (int r = 0; r < bbox.height; ++r) {
                const auto* lbl_ptr = labels_roi.ptr<int32_t>(r);
                const auto* sob_ptr = sobel_roi.ptr<uint8_t>(r);
                for (int c = 0; c < bbox.width; ++c) {
                    if (lbl_ptr[c] == i && sob_ptr[c] > peak) {
                        peak = sob_ptr[c];
                    }
                }
            }

            if (peak < sobel_peak_min) continue;
        }

        blob_keep_lut_[i] = 255;
        ++kept;
    }

    if (kept > 0) {
        for (int r = 0; r < mask_.rows; ++r) {
            const auto* lbl_ptr = labels_.ptr<int32_t>(r);
            auto* msk_ptr = mask_.ptr<uint8_t>(r);
            for (int c = 0; c < mask_.cols; ++c) {
                msk_ptr[c] = blob_keep_lut_[lbl_ptr[c]];
            }
        }
        return true;
    }

    if (gray_src.empty() || gray_src.channels() != 1) return false;

    adaptiveThreshold(
        gray_src, mask_, 255,
        ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV,
        adaptive_block, ADAPTIVE_C);

    morphologyEx(mask_, mask_, MORPH_CLOSE, mask_close_kernel_);
    morphologyEx(mask_, mask_, MORPH_OPEN, mask_open_kernel_);

    const int min_area_adaptive = std::max(15, min_area / 4);
    const int components2 = connectedComponentsWithStats(mask_, labels_, stats_, centroids_, 8, CV_32S);
    if (components2 <= 1) return false;

    blob_keep_lut_.assign(components2, 0);
    int kept2 = 0;

    for (int i = 1; i < components2; ++i) {
        const int area = stats_.at<int>(i, CC_STAT_AREA);
        const int height = stats_.at<int>(i, CC_STAT_HEIGHT);
        const int width = stats_.at<int>(i, CC_STAT_WIDTH);

        if (area < min_area_adaptive || height < min_blob_height || width > max_blob_width) continue;

        blob_keep_lut_[i] = 255;
        ++kept2;
    }

    if (kept2 == 0) return false;

    for (int r = 0; r < mask_.rows; ++r) {
        const auto* lbl_ptr = labels_.ptr<int32_t>(r);
        auto* msk_ptr = mask_.ptr<uint8_t>(r);
        for (int c = 0; c < mask_.cols; ++c) {
            msk_ptr[c] = blob_keep_lut_[lbl_ptr[c]];
        }
    }

    return true;
}

// ============================================================================
// IPM
// ============================================================================

bool CameraLane::update_ipm(int w, int h) {
    if (w < 32 || h < 32) return false;

    if (m_ipm_.empty() || ipm_w_ != w || ipm_h_ != h) {
        const float fx = static_cast<float>(w);
        const float fy = static_cast<float>(h);

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
        ipm_w_ = w;
        ipm_h_ = h;

        if (BONNET_ENABLE) {
            const int hood_y = std::max(0, std::min(
                static_cast<int>(BONNET_Y0 * fy), static_cast<int>(fy) - 1));
            const std::vector<Point2f> hood_pt = {Point2f(0.5f * fx, static_cast<float>(hood_y))};
            std::vector<Point2f> hood_dst;
            perspectiveTransform(hood_pt, hood_dst, m_ipm_);

            if (!hood_dst.empty() && std::isfinite(hood_dst[0].y)) {
                bird_hood_y_ = static_cast<int>(std::lround(hood_dst[0].y)) + 1;
            } else {
                bird_hood_y_ = static_cast<int>(BIRD_H * BONNET_Y0);
            }
        } else {
            bird_hood_y_ = BIRD_H;
        }

        bird_valid_rows_ = std::max(BIRD_MIN_VALID_ROWS, std::min(bird_hood_y_, BIRD_H));
    }

    return !m_ipm_.empty() && !m_ipm_inv_.empty();
}

bool CameraLane::transforming_view(const cv::Mat& src, int flag, cv::Mat& dst) {
    if (src.empty()) return false;

    const bool to_bird = (flag == VIEW_BIRD_EYE);
    const Mat& m = to_bird ? m_ipm_ : m_ipm_inv_;
    if (m.empty()) return false;

    if (to_bird) {
        warpPerspective(src, dst, m, bird_size_, INTER_LINEAR);
    } else {
        warpPerspective(src, dst, m, Size(ipm_w_, ipm_h_), INTER_LINEAR);
    }

    return !dst.empty();
}

// ============================================================================
// SLIDING WINDOW TRÊN BIRD'S-EYE
// ============================================================================

bool CameraLane::window_search(float& center_x, bool& dual) {
    left_win_means_.clear();
    right_win_means_.clear();
    left_fit_px_.clear();
    right_fit_px_.clear();
    center_x = static_cast<float>(BIRD_W) * 0.5f;
    dual = false;

    if (bird_.empty()) return false;

    const int y_bottom = std::max(BIRD_MIN_VALID_ROWS, std::min(bird_valid_rows_, BIRD_H));
    const int y_top = std::max(0, std::min(
        static_cast<int>(static_cast<float>(BIRD_H) * BIRD_HIST_TOP_FRAC), y_bottom - 1));
    const int y_hist0 = y_top;

    std::fill(bird_col_sum_.begin(), bird_col_sum_.end(), 0);

    for (int y = y_hist0; y < y_bottom; ++y) {
        const uchar* row = bird_.ptr<uchar>(y);
        for (int x = 0; x < BIRD_W; ++x) {
            if (row[x]) ++bird_col_sum_[static_cast<size_t>(x)];
        }
    }

    const int mid = BIRD_W / 2;
    int left_seed = 0;
    int right_seed = 0;

    const int min_column = std::max(
        WINDOW_MIN_POINTS,
        static_cast<int>(static_cast<float>(y_bottom - y_hist0) * BIRD_MIN_COLUMN_RATIO));

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
    } else if (peak_a_val >= min_column) {
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
            bird_valid_ = false;
            return false;
        }

        if (centroid < static_cast<float>(mid)) left_seed = static_cast<int>(std::lround(centroid));
        else right_seed = static_cast<int>(std::lround(centroid));
    }

    const bool has_left = two_peaks || (peak_a_val >= min_column && right_seed == 0);
    const bool has_right = two_peaks || (peak_a_val >= min_column && left_seed == 0);

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

    findNonZero(bird_, bird_nonzero_);

    const int window_h = std::max(1, y_bottom / N_WINDOWS);
    int lx = left_seed;
    int rx = right_seed;

    for (int w = 0; w < N_WINDOWS; ++w) {
        const int y_low = y_bottom - (w + 1) * window_h;
        const int y_high = y_bottom - w * window_h;
        if (y_low < y_hist0) break;
        const int y_mid = (y_low + y_high) / 2;

        if (has_left) search_window(y_low, y_high, y_mid, 0, lx);
        if (has_right) search_window(y_low, y_high, y_mid, 1, rx);
    }

    if (has_left && has_right) {
        dual = true;
        center_x = static_cast<float>((lx + rx) / 2);
    } else if (has_left) {
        center_x = static_cast<float>(lx);
    } else {
        center_x = static_cast<float>(rx);
    }

    return !left_fit_px_.empty() || !right_fit_px_.empty();
}

bool CameraLane::search_window(
    int y_low,
    int y_high,
    int y_mid,
    int side,
    int& seed
) {
    std::fill(bird_win_sum_.begin(), bird_win_sum_.end(), 0);

    for (const auto& p : bird_nonzero_) {
        if (p.y >= y_low && p.y < y_high) {
            ++bird_win_sum_[static_cast<size_t>(p.x)];
        }
    }

    const int x_low = std::max(0, seed - WINDOW_MARGIN);
    const int x_high = std::min(BIRD_W - 1, seed + WINDOW_MARGIN);

    int peak_x = -1;
    int peak_val = 0;
    for (int x = x_low; x <= x_high; ++x) {
        const int v = bird_win_sum_[static_cast<size_t>(x)];
        if (v > peak_val ||
            (v == peak_val && peak_x >= 0 && std::abs(x - seed) < std::abs(peak_x - seed))) {
            peak_val = v;
            peak_x = x;
        }
    }

    if (peak_x < 0 || peak_val < WINDOW_MIN_POINTS) return false;

    const int run_threshold = static_cast<int>(static_cast<float>(peak_val) * WINDOW_RUN_RATIO);
    int run_low = peak_x;
    while (run_low > x_low && bird_win_sum_[static_cast<size_t>(run_low - 1)] > run_threshold) --run_low;

    int run_high = peak_x;
    while (run_high + 1 <= x_high && run_high + 1 < BIRD_W &&
           bird_win_sum_[static_cast<size_t>(run_high + 1)] > run_threshold) {
        ++run_high;
    }

    const int centre = (run_low + run_high) / 2;
    if (std::abs(centre - seed) > WINDOW_MAX_SHIFT) return false;

    track_run_w_[side] = static_cast<float>(run_high - run_low + 1);
    seed = centre;

    auto& fit_px = (side == 0) ? left_fit_px_ : right_fit_px_;
    auto& means = (side == 0) ? left_win_means_ : right_win_means_;

    for (const auto& p : bird_nonzero_) {
        if (p.y >= y_low && p.y < y_high && p.x >= run_low && p.x <= run_high) {
            fit_px.push_back(p);
        }
    }

    means.emplace_back(centre, y_mid);
    return true;
}

// ============================================================================
// ĐƯA ĐƯỜNG CONG VỀ KHUNG HÌNH GỐC
// ============================================================================

void CameraLane::build_curve_points(
    bool fit_ok,
    const float coef[3],
    int frame_w,
    int y0,
    int y1,
    std::vector<cv::Point>& out
) {
    out.clear();
    if (!fit_ok || m_ipm_inv_.empty()) return;

    const int y_top = std::max(0, static_cast<int>(static_cast<float>(BIRD_H) * BIRD_SAMPLE_TOP_FRAC));

    std::vector<Point2f> bird_pts;
    bird_pts.reserve(static_cast<size_t>((BIRD_H - y_top) / BIRD_SAMPLE_STEP + 1));
    for (int by = BIRD_H - 1; by >= y_top; by -= BIRD_SAMPLE_STEP) {
        const float bx = poly_eval(coef, static_cast<float>(by));
        if (!std::isfinite(bx)) continue;
        bird_pts.emplace_back(bx, static_cast<float>(by));
    }
    if (bird_pts.empty()) return;

    std::vector<Point2f> full_pts;
    perspectiveTransform(bird_pts, full_pts, m_ipm_inv_);

    out.reserve(full_pts.size());
    for (const auto& p : full_pts) {
        const int x = static_cast<int>(std::lround(p.x));
        const int y = static_cast<int>(std::lround(p.y));
        if (x < 0 || x >= frame_w) continue;
        if (y < y0 || y > y1) continue;
        out.emplace_back(x, y);
    }

    std::reverse(out.begin(), out.end());
}

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
        out.emplace_back(static_cast<int>(std::lround(a.x + t * static_cast<float>(b.x - a.x))), y);
    }
}

// ============================================================================
// NHÁNH CHÍNH: IPM
// ============================================================================

bool CameraLane::detect_lanes_ipm(
    const cv::Mat& full_frame,
    int y0_dual,
    int y0_single,
    int y1,
    float width_scale,
    bool& is_dual_lane,
    cv::Mat& out_bird
) {
    is_dual_lane = false;

    if (!update_ipm(full_frame.cols, full_frame.rows)) return false;
    if (!transforming_view(mask_, VIEW_BIRD_EYE, bird_)) return false;

    if (bird_valid_rows_ < BIRD_H) {
        bird_.rowRange(bird_valid_rows_, BIRD_H).setTo(0);
    }

    float center_x = static_cast<float>(BIRD_W) * 0.5f;
    bool seed_dual = false;
    if (!window_search(center_x, seed_dual)) {
        bird_valid_ = false;
        return false;
    }

    const bool left_fit_ok = left_fit_px_.size() >= static_cast<size_t>(BIRD_MIN_FIT_PIXELS) &&
                             fit_poly(left_fit_px_, left_fit_) &&
                             fit_residual_ok(left_win_means_, left_fit_);
    const bool right_fit_ok = right_fit_px_.size() >= static_cast<size_t>(BIRD_MIN_FIT_PIXELS) &&
                              fit_poly(right_fit_px_, right_fit_) &&
                              fit_residual_ok(right_win_means_, right_fit_);

    if (!left_fit_ok && !right_fit_ok) {
        bird_valid_ = false;
        return false;
    }

    build_curve_points(left_fit_ok, left_fit_, full_frame.cols, y0_dual, y1, left_raw_);
    build_curve_points(right_fit_ok, right_fit_, full_frame.cols, y0_dual, y1, right_raw_);

    bool width_ok = false;
    float bird_width = 0.0f;
    float image_width = 0.0f;
    float image_center_x = 0.0f;

    if (left_fit_ok && right_fit_ok) {
        const float bird_y = static_cast<float>(BIRD_H - 1);
        const float bx_left = poly_eval(left_fit_, bird_y);
        const float bx_right = poly_eval(right_fit_, bird_y);
        bird_width = bx_right - bx_left;

        std::vector<Point2f> bird_bottom = {
            Point2f(bx_left, bird_y),
            Point2f(bx_right, bird_y)
        };
        std::vector<Point2f> full_bottom;
        perspectiveTransform(bird_bottom, full_bottom, m_ipm_inv_);

        image_width = std::abs(static_cast<float>(full_bottom[1].x - full_bottom[0].x));
        image_center_x = static_cast<float>(0.5 * (full_bottom[0].x + full_bottom[1].x));

        const float estimate = lane_width_est_px_ > 1.0f
            ? lane_width_est_px_
            : DEFAULT_LANE_WIDTH_PX * width_scale;

        width_ok = bird_width >= BIRD_MIN_LANE_WIDTH &&
                   bird_width <= BIRD_MAX_LANE_WIDTH &&
                   image_width >= estimate * IPM_MIN_WIDTH_RATIO &&
                   image_width <= estimate * IPM_MAX_WIDTH_RATIO;
    }

    if (width_ok) {
        bird_lane_width_ = bird_lane_width_ > 0.0f
            ? ALPHA_LANE_WIDTH * bird_width + (1.0f - ALPHA_LANE_WIDTH) * bird_lane_width_
            : bird_width;
        lane_width_est_px_ = ALPHA_LANE_WIDTH * image_width + (1.0f - ALPHA_LANE_WIDTH) * lane_width_est_px_;
        bird_center_prev_ = center_x;
        bird_valid_ = true;
        xmid_scan_ = std::max(0, std::min(
            static_cast<int>(std::lround(image_center_x)),
            full_frame.cols - 1));
        is_dual_lane = true;
    } else {
        bird_center_prev_ = bird_valid_ ? bird_center_prev_ : center_x;
        bird_valid_ = true;
    }

    const int y0_used = is_dual_lane ? y0_dual : y0_single;

    resample_curve(left_raw_, y0_used, y1, leftPts_);
    resample_curve(right_raw_, y0_used, y1, rightPts_);

    if (!bird_.empty()) out_bird = bird_.clone();

    return !leftPts_.empty() || !rightPts_.empty();
}

void CameraLane::draw_lane_band(cv::Mat& vis) const {
    if (leftPts_.size() < 2 || rightPts_.size() < 2) return;

    const int band = std::max(1, static_cast<int>(std::round(
        BIRD_LANE_BAND_MARGIN * static_cast<float>(vis.cols) / static_cast<float>(FRAME_W))));

    std::vector<Point> polygon;
    polygon.reserve(leftPts_.size() + rightPts_.size());

    for (const auto& p : leftPts_) polygon.emplace_back(p.x - band, p.y);
    for (auto it = rightPts_.rbegin(); it != rightPts_.rend(); ++it) {
        polygon.emplace_back(it->x + band, it->y);
    }

    Mat overlay = vis.clone();
    fillConvexPoly(overlay, polygon, Scalar(0, 200, 0));
    addWeighted(overlay, BIRD_LANE_BAND_ALPHA, vis, 1.0 - BIRD_LANE_BAND_ALPHA, 0.0, vis);
}

// ============================================================================
// NHÁNH DỰ PHÒNG: CANNY + SCANLINE
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

    GaussianBlur(gray_, blur_, Size(scale_px(BLUR_KERNEL, width_scale),
                                    scale_px(BLUR_KERNEL, width_scale)), 0);
    Canny(blur_, edges_, CANNY_LOW, CANNY_HIGH);
    morphologyEx(edges_, edges_, MORPH_CLOSE, morph_kernel_, Point(-1, -1), 1);

    leftPts_prelim_.clear();
    rightPts_prelim_.clear();
    prelim_centers_.clear();

    const float min_width = MIN_LANE_WIDTH_PX * width_scale;
    const float max_width = MAX_LANE_WIDTH_PX * width_scale;

    int xmid_candidate = xmid_scan_;
    double width_sum = 0.0;
    int overlap_count = 0;

    for (int y = y0_dual; y <= y1; y += SCAN_STEP) {
        const int local_y = y - y0_dual;
        if (local_y < 0 || local_y >= H) continue;
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

    const bool has_left_prelim = leftPts_prelim_.size() >= static_cast<size_t>(MIN_PRELIM_POINTS);
    const bool has_right_prelim = rightPts_prelim_.size() >= static_cast<size_t>(MIN_PRELIM_POINTS);
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
// MAIN PIPELINE
// ============================================================================

void CameraLane::detect_lanes(const cv::Mat& bgr, float current_speed_kmh, LaneOutput& out) {
    out.left.clear();
    out.right.clear();
    out.center.clear();
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
    out.held_frames = 0;

    if (bgr.empty()) {
        target_speed_initialized_ = false;
        speed_state_ = SpeedState::STRAIGHT;
        return;
    }

    const int W = bgr.cols;
    const int H = bgr.rows;
    if (W < 100 || H < 100) return;

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

    if (y0_dual >= y1) return;

    const Rect roi_rect(0, y0_dual, W, H - y0_dual);
    const Mat roi = bgr(roi_rect);

    const float width_scale = static_cast<float>(W) / static_cast<float>(FRAME_W);
    // Nguồn sự thật cho mọi quy đổi design-space <-> pixel thực tế. Lấy từ
    // khung vừa nhận (không phải px_scale_ của lúc start) để vẫn đúng nếu
    // camera tự đổi độ phân giải sau khi reconnect.
    px_scale_ = width_scale;

    if (bgr.channels() == 1) gray_full_ = bgr;
    else if (bgr.channels() == 3) cvtColor(bgr, gray_full_, COLOR_BGR2GRAY);
    else if (bgr.channels() == 4) cvtColor(bgr, gray_full_, COLOR_BGRA2GRAY);

    if (roi.channels() == 1) gray_ = roi;
    else if (roi.channels() == 3) cvtColor(roi, gray_, COLOR_BGR2GRAY);
    else if (roi.channels() == 4) cvtColor(roi, gray_, COLOR_BGRA2GRAY);

    bool is_dual_lane = false;
    bool ipm_used = false;

    if (sobel_color_thresholding(bgr) && filter_lane_blobs(gray_full_)) {
        ipm_used = detect_lanes_ipm(bgr, y0_dual, y0_single, y1, width_scale,
                                    is_dual_lane, out.bird_mask);
        if (!mask_.empty() && mask_.size() == bgr.size()) {
            out.binary_mask = mask_.clone();
        }
    }

    if (!ipm_used) {
        leftPts_.clear();
        rightPts_.clear();
        is_dual_lane = false;
        if (gray_.channels() == 1) {
            detect_lanes_scanline(roi, width_scale, y0_dual, y0_single, y1, is_dual_lane);
        }
        bird_valid_ = false;
        out.bird_mask.release();
    } else {
        median_smooth(leftPts_);
        median_smooth(rightPts_);
        out.detector_mode = 1;
    }

    out.is_dual_lane = is_dual_lane;
    out.left = leftPts_;
    out.right = rightPts_;
    out.pixels_used = static_cast<int>(leftPts_.size() + rightPts_.size());
    out.lane_width_est_px = lane_width_est_px_;

    if constexpr (ENABLE_LANE_COLOR_SAMPLE) {
        if (!mask_.empty() && mask_.size() == bgr.size()) {
            const Scalar mean_bgr = mean(bgr, mask_);
            if (mean_bgr[0] > 1.0 || mean_bgr[1] > 1.0 || mean_bgr[2] > 1.0) {
                out.lane_color_bgr = mean_bgr;
                const double luma = 0.114 * mean_bgr[0] + 0.587 * mean_bgr[1] + 0.299 * mean_bgr[2];
                out.lane_color_name = luma < 128.0 ? "black" : "white";
            }
        }
    }

    const bool has_left = leftPts_.size() >= static_cast<size_t>(MIN_FINAL_POINTS);
    const bool has_right = rightPts_.size() >= static_cast<size_t>(MIN_FINAL_POINTS);

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
            const float lane_width = clamp_float(lane_width_est_px_, MIN_LANE_WIDTH_PX * width_scale, MAX_LANE_WIDTH_PX * width_scale);
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
        lost_frame_count_ = 0;
        last_valid_dev_ = out.dev_final_px;
        last_valid_slope_ = dominant_slope;

        const int abs_dev = std::abs(static_cast<int>(out.dev_final_px));

        // Ngưỡng FWD_THRESHOLD_PX / SHARP_*_DEV_PX / CURVE_*_DEV_PX đều viết cho
        // khung 640 rộng. dev_final_px là pixel thực tế (352 khi chạy CAP_W),
        // nên quy về design space trước khi so sánh.
        const float inv_px = (px_scale_ > 0.0f) ? (1.0f / px_scale_) : 1.0f;
        const int abs_dev_design = static_cast<int>(std::round(abs_dev * inv_px));

        if (abs_dev_design < FWD_THRESHOLD_PX) out.camera_cmd = "FWD";
        else if (out.dev_final_px < 0) out.camera_cmd = "LEFT";
        else out.camera_cmd = "RIGHT";

        const uint8_t requested_speed = calculate_target_speed(dominant_slope, abs_dev_design);

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
    } else if (lost_frame_count_ < MAX_LOST_FRAMES && last_valid_dev_ != 0) {
        ++lost_frame_count_;
        out.held_frames = lost_frame_count_;

        out.dev_final_px = last_valid_dev_;
        out.valid = true;
        out.curve_angle_deg = std::atan(last_valid_slope_) * 180.0f / static_cast<float>(CV_PI);
        out.curvature = std::fabs(last_valid_slope_);

        const int abs_dev = std::abs(static_cast<int>(out.dev_final_px));
        const float inv_px = (px_scale_ > 0.0f) ? (1.0f / px_scale_) : 1.0f;
        const int abs_dev_design = static_cast<int>(std::round(abs_dev * inv_px));
        if (abs_dev_design < FWD_THRESHOLD_PX) out.camera_cmd = "FWD";
        else if (out.dev_final_px < 0) out.camera_cmd = "LEFT";
        else out.camera_cmd = "RIGHT";

        const float decay = 1.0f - 0.30f * static_cast<float>(lost_frame_count_);
        target_speed_x10_filtered_ = ALPHA_TARGET_SPEED * (static_cast<float>(SPEED_SHARP_X10) * decay) +
                                     (1.0f - ALPHA_TARGET_SPEED) * target_speed_x10_filtered_;
        target_speed_x10_filtered_ = clamp_float(target_speed_x10_filtered_, 0.0f, 255.0f);
        out.target_speed_x10 = static_cast<uint8_t>(std::round(target_speed_x10_filtered_));
        out.speed_factor = static_cast<uint8_t>(std::min(
            100, static_cast<int>(std::round(
                100.0f * static_cast<float>(out.target_speed_x10) / static_cast<float>(SPEED_STRAIGHT_X10)))));
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
        const int y0_used = is_dual_lane ? y0_dual : y0_single;

        bgr.copyTo(out.vis);
        line(out.vis, Point(xmid_default, y0_used), Point(xmid_default, y1), Scalar(255, 255, 255), 1, LINE_AA);
        line(out.vis, Point(xmid_scan_, y0_used), Point(xmid_scan_, y1), Scalar(0, 255, 255), 1, LINE_AA);

        if (out.detector_mode == 1) {
            const Point tl(static_cast<int>(IPM_TL_X * W), 0);
            const Point tr(static_cast<int>(IPM_TR_X * W), 0);
            const Point br(static_cast<int>(IPM_BR_X * W), y1);
            const Point bl(static_cast<int>(IPM_BL_X * W), y1);
            const std::vector<Point> trapezoid = {tl, tr, br, bl};
            polylines(out.vis, trapezoid, true, Scalar(0, 200, 255), 1, LINE_AA);
        }

        if (BONNET_ENABLE) {
            const int hood_y = std::max(0, std::min(
                static_cast<int>(BONNET_Y0 * static_cast<float>(H)), H - 1));
            line(out.vis, Point(0, hood_y), Point(W, hood_y), Scalar(0, 0, 255), 1, LINE_AA);
        }

        if (!out.left.empty()) polylines(out.vis, out.left, false, Scalar(0, 0, 255), 2, LINE_AA);
        if (!out.right.empty()) polylines(out.vis, out.right, false, Scalar(0, 0, 255), 2, LINE_AA);
        if (!out.center.empty()) polylines(out.vis, out.center, false, Scalar(0, 255, 0), 2, LINE_AA);

        draw_lane_band(out.vis);

        if constexpr (ENABLE_BIRD_DEBUG) {
            Mat bird_back;
            if (transforming_view(bird_, VIEW_NORMAL, bird_back) && !bird_back.empty()) {
                Mat canvas;
                const int bottom_pad = out.vis.rows - bird_back.rows;
                if (bottom_pad >= 0) {
                    copyMakeBorder(bird_back, canvas, 0, bottom_pad, 0, 0, BORDER_CONSTANT, Scalar(0));
                    if (canvas.size() == out.vis.size()) {
                        Mat tint;
                        cvtColor(canvas, tint, COLOR_GRAY2BGR);
                        addWeighted(out.vis, 1.0 - BIRD_LANE_BAND_ALPHA, tint, BIRD_LANE_BAND_ALPHA, 0.0, out.vis);
                    }
                }
            }

            if (!bird_.empty() && out.vis.cols >= BIRD_W / 2 && out.vis.rows >= BIRD_H / 2) {
                const Rect inset(0, 0, BIRD_W / 2, BIRD_H / 2);
                Mat small;
                resize(bird_, small, inset.size(), 0, 0, INTER_AREA);
                cvtColor(small, small, COLOR_GRAY2BGR);
                rectangle(small, Rect(0, 0, small.cols - 1, small.rows - 1), Scalar(0, 0, 0), 1);
                small.copyTo(out.vis(inset));
            }
        }
    } else {
        out.vis.release();
    }

    if constexpr (ENABLE_DEBUG_LOG) {
        if ((debug_frame_count_++ % static_cast<uint64_t>(DEBUG_LOG_EVERY)) == 0) {
            std::cout << "[CameraLane] mode=" << static_cast<int>(out.detector_mode)
                      << " dual=" << (is_dual_lane ? 1 : 0)
                      << " valid=" << (out.valid ? 1 : 0)
                      << " pts=" << out.pixels_used
                      << " lane_w=" << lane_width_est_px_
                      << " bird_w=" << bird_lane_width_
                      << " bird_mid=" << bird_center_prev_
                      << " dev=" << out.dev_final_px
                      << " cmd=" << out.camera_cmd
                      << " speed=" << static_cast<int>(out.target_speed_x10)
                      << " color=" << out.lane_color_name
                      << "\n";
        }
    }
}