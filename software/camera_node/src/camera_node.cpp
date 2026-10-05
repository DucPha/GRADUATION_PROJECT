#include "camera_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib> // Cho std::abs
#include <iostream>
#include <utility>

// ============================================================================
// CONSTRUCTOR / DESTRUCTOR
// ============================================================================

CameraLane::CameraLane(int camera_index, int target_fps)
    : camera_index_(camera_index),
      target_fps_(target_fps > 0 ? target_fps : 30),
      clahe_(cv::createCLAHE(CLAHE_CLIP, cv::Size(CLAHE_TILE, CLAHE_TILE))) {}

CameraLane::~CameraLane() {
    stop();
}

// ============================================================================
// BAT / TAT
// ============================================================================

bool CameraLane::start() {
    if (running_.load()) return true;
    if (!open_camera()) return false;

    running_.store(true);
    worker_ = std::thread(&CameraLane::capture_loop, this);
    return true;
}

void CameraLane::stop() {
    running_.store(false);
    if (worker_.joinable()) worker_.join();
    cap_.release();
}

bool CameraLane::is_running() const {
    return running_.load();
}

void CameraLane::set_roi_top_frac(float frac) {
    roi_top_frac_.store(std::clamp(frac, 0.05f, 0.90f));
}

// ============================================================================
// MO CAMERA
// ============================================================================

bool CameraLane::open_camera() {
    const int kProbeCount = 4;
    const int kFirstIndex = camera_index_ >= 0 ? camera_index_ : 0;
    const int kLastIndex = camera_index_ >= 0 ? camera_index_ : kProbeCount - 1;

    for (int idx = kFirstIndex; idx <= kLastIndex; ++idx) {
        if (!cap_.open(idx, cv::CAP_V4L2)) {
            cap_.release();
            continue;
        }

        cap_.set(cv::CAP_PROP_FRAME_WIDTH, WORK_W * 2);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, WORK_H * 2);
        cap_.set(cv::CAP_PROP_FPS, target_fps_); // Đã xoá dấu * bị dư

        cv::Mat probe;
        if (!cap_.read(probe) || probe.empty()) {
            cap_.release();
            continue;
        }

        camera_index_ = idx;
        std::cout << "[CameraLane] Camera opened at index " << idx 
                  << ", real size " << probe.cols << "x" << probe.rows << "\n";

        if (probe.cols != WORK_W * 2 || probe.rows != WORK_H * 2) {
            std::cout << "[CameraLane] Expected " << (WORK_W * 2) << "x" << (WORK_H * 2)
                      << ". Detector will scale the coordinates.\n";
        }

        return true;
    }

    std::cerr << "[CameraLane] No camera found\n";
    return false;
}

// ============================================================================
// LUONG DOC FRAME
// ============================================================================

void CameraLane::capture_loop() {
    cv::Mat frame;
    int fail_count = 0;

    while (running_.load()) {
        if (!cap_.read(frame) || frame.empty()) {
            if (++fail_count >= MAX_READ_FAIL) {
                std::cerr << "[CameraLane] Camera stopped delivering frames\n";
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        fail_count = 0;
        LaneOutput out;
        out.frame_id = ++frame_id_;
        detect(frame, out);

        {
            std::lock_guard<std::mutex> lock(mtx_);
            latest_ = std::move(out);
            last_frame_time_ = std::chrono::steady_clock::now();
        }
    }
}

void CameraLane::get_latest(LaneOutput& out, bool copy_vis) const {
    std::lock_guard<std::mutex> lock(mtx_);

    out.vis = copy_vis ? latest_.vis : cv::Mat();
    out.left_pts = latest_.left_pts;
    out.right_pts = latest_.right_pts;
    out.two_lanes = latest_.two_lanes;
    out.state = latest_.state; // Lấy state
    out.curve_px = latest_.curve_px; // Lấy curve
    out.dev_px = latest_.dev_px;
    out.dev_cm = latest_.dev_cm;
    out.lane_width_cm = latest_.lane_width_cm;
    out.fit_ok = latest_.fit_ok;
    out.proc_ms = latest_.proc_ms;
    out.frame_id = latest_.frame_id;

    if (last_frame_time_ == std::chrono::steady_clock::time_point{}) {
        out.age_ms = static_cast<unsigned long>(-1);
        out.stale = true;
        return;
    }

    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - last_frame_time_
    ).count();

    out.age_ms = (ms < 0) ? 0ul : static_cast<unsigned long>(ms);
    out.stale = out.age_ms > static_cast<unsigned long>(STALE_AGE_MS);
}

// ============================================================================
// DETECTOR
// ============================================================================

bool CameraLane::detect(const cv::Mat& frame, LaneOutput& out) {
    const auto t_begin = std::chrono::steady_clock::now();

    cv::Mat work;
    cv::resize(frame, work, cv::Size(WORK_W, WORK_H), 0, 0, cv::INTER_AREA);

    // ---- 1. Vung lam viec --------------------------------------------------
    const int top = std::clamp(
        static_cast<int>(std::lround(roi_top_frac_.load() * WORK_H)),
        1, WORK_H - 2
    );
    const int bottom = std::clamp(
        static_cast<int>(std::lround(ROI_BOTTOM_FRAC * WORK_H)),
        top + N_WINDOWS, WORK_H
    );
    const int win_h = std::max(1, (bottom - top) / N_WINDOWS);

    auto band_of = [&](int i, int& y0, int& y1) {
        y1 = bottom - i * win_h;
        y0 = std::max(top, y1 - win_h);
    };

    // ---- 2. Xam -> GaussianBlur -> CLAHE --------------------------
    cv::Mat gray;
    cv::Mat bin;
    cv::cvtColor(work, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(BLUR_KSIZE, BLUR_KSIZE), 0);
    clahe_->apply(gray, gray);

    // ---- 3. ROI hinh thang & Otsu cuc bo ---------------------------------
    cv::Mat mask = cv::Mat::zeros(WORK_H, WORK_W, CV_8U);
    const int half_top = WORK_W * 3 / 10; // Đã nới đỉnh hình thang (192 px)
    const int half_bot = WORK_W / 2 - 2;
    const std::vector<cv::Point> roi_poly = {
        cv::Point(WORK_W / 2 - half_top, top),
        cv::Point(WORK_W / 2 + half_top, top),
        cv::Point(WORK_W / 2 + half_bot, bottom),
        cv::Point(WORK_W / 2 - half_bot, bottom),
    };
    
    cv::fillPoly(mask, roi_poly, cv::Scalar(255));
    
    // Otsu chỉ tính trên vùng ROI, rồi mới che mask lên ảnh nhị phân
    const double thr = cv::threshold(
        gray(cv::Rect(0, top, WORK_W, bottom - top)), bin, 0, 255,
        cv::THRESH_BINARY_INV | cv::THRESH_OTSU
    );
    cv::threshold(gray, bin, thr, 255, cv::THRESH_BINARY_INV);
    cv::bitwise_and(bin, mask, bin);

    // ---- 4. Morphology ------------------------------------
    cv::morphologyEx(
        bin, bin, cv::MORPH_CLOSE,
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 15))
    );
    cv::morphologyEx(
        bin, bin, cv::MORPH_OPEN,
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3))
    );

    // ---- 5. Cua so truot co du doan phoi canh --------------------------
    std::vector<bool> band_ok(N_WINDOWS, false);
    std::vector<int> peak_left(N_WINDOWS, -1);
    std::vector<int> peak_right(N_WINDOWS, -1);

    int seed_left = -1, seed_right = -1, seed_y = 0;
    int center_x = WORK_W / 2;
    
    int yl = 0, yr = 0;                 // hàng lần cuối thấy từng vạch
    int matched = 0, nL = 0, nR = 0;

    auto predict = [&](int x, int y_from, int y_to) {
        const int d0 = y_from - HORIZON_Y;
        const int d1 = y_to - HORIZON_Y;
        if (d0 <= 1 || d1 <= 1) return x;
        const int cx = WORK_W / 2;
        return cx + static_cast<int>(std::lround((x - cx) * static_cast<double>(d1) / d0));
    };

    for (int i = N_WINDOWS - 1; i >= 0; --i) {
        if (seed_left < 0 && i < N_WINDOWS - MAX_SEED_SEARCH) break;

        int y0 = 0, y1 = 0;
        band_of(i, y0, y1);
        if (y1 - y0 < 2) continue;
        const int y_mid = (y0 + y1) / 2;

        cv::Mat col_sum;
        cv::reduce(bin(cv::Rect(0, y0, WORK_W, y1 - y0)),
                   col_sum, 0, cv::REDUCE_SUM, CV_32S);
        col_sum /= 255; // Đếm pixel đúng

        if (seed_left < 0) {
            int l = 0, r = 0;
            if (!seed_pair_from(col_sum, l, r)) continue;
            seed_left = l; seed_right = r;
            seed_y = y_mid; yl = yr = y_mid;
        }

        int left = 0, right = 0;
        const bool okL = peak_in(col_sum, predict(seed_left,  yl, y_mid), left);
        const bool okR = peak_in(col_sum, predict(seed_right, yr, y_mid), right);
        
        if (!okL && !okR) continue;
        if (okL && okR && (left >= right || right - left < MIN_LANE_GAP)) continue;

        if (okL) { seed_left  = left;  yl = y_mid; peak_left[i]  = left;  ++nL; }
        if (okR) { seed_right = right; yr = y_mid; peak_right[i] = right; ++nR; }

        if (okL && okR) {
            band_ok[i] = true;
            ++matched;
            out.left_pts.push_back({left, y_mid});
            out.right_pts.push_back({right, y_mid});
            center_x = (left + right) / 2;
        }
    }

    // ---- 6. Quyet dinh 2 lan ---------------------------
    bool two_lanes = matched >= MIN_MATCHED_WINDOWS;

    if (two_lanes) {
        int width = 0;
        int y_near = bottom;
        for (int i = 0; i < N_WINDOWS; ++i) {
            if (band_ok[i]) {
                width = peak_right[i] - peak_left[i];
                int y0 = 0, y1 = 0;
                band_of(i, y0, y1);
                y_near = (y0 + y1) / 2;
                break;
            }
        }
        two_lanes = width >= LANE_WIDTH_MIN && width <= LANE_WIDTH_MAX;

        if (two_lanes) {
            out.lane_width_cm = px_to_cm(static_cast<float>(width), y_near);
        }
    }

    const int y_look = top + static_cast<int>((bottom - top) * LOOKAHEAD_FRAC);
    
    // ---- 7. FALLBACK 1 LAN ---------------------------
    if (two_lanes && out.lane_width_cm > 20.0f) {
        lane_w_m_ = out.lane_width_cm / 100.0f;
    }
    
    LaneState state = two_lanes ? LaneState::TWO_LINES : LaneState::LOST;
    
    if (!two_lanes) {
        const bool use_left = nL >= nR;
        const std::vector<int>& pk = use_left ? peak_left : peak_right;
        
        if ((use_left ? nL : nR) >= MIN_ONE_SIDE_WINDOWS) {
            int best_i = -1, best_d = 1 << 30;
            for (int i = 0; i < N_WINDOWS; ++i) {
                if (pk[i] < 0) continue;
                int y0 = 0, y1 = 0;
                band_of(i, y0, y1);
                const int d = std::abs((y0 + y1) / 2 - y_look);
                if (d < best_d) { best_d = d; best_i = i; }
            }
            
            int y0 = 0, y1 = 0;
            band_of(best_i, y0, y1);
            const int dy = (y0 + y1) / 2 - HORIZON_Y;
            
            if (dy > 1) {
                const int half = static_cast<int>(std::lround(
                    0.5 * lane_w_m_ * dy / CAMERA_HEIGHT_M
                ));      // nửa làn, px
                
                const int centre_px = use_left ? pk[best_i] + half : pk[best_i] - half;
                
                // quy về hàng y_look để dev_px cùng quy ước với trường hợp 2 vạch
                const double off = (centre_px - WORK_W / 2) * static_cast<double>(y_look - HORIZON_Y) / dy;
                const double scale = frame.cols / static_cast<double>(WORK_W);
                
                dev_ema_ = static_cast<int>(std::lround(
                    EMA_ALPHA * off * scale + (1.0 - EMA_ALPHA) * dev_ema_
                ));
                out.dev_cm = px_to_cm(static_cast<float>(off), y_look);
                center_x = centre_px;
                state = LaneState::ONE_LINE;
            }
        }
    }

    // ---- 8. Fit duong bac 2 ---------------------------------------------
    double lf[3] = {0, 0, 0};
    double rf[3] = {0, 0, 0};
    int ly0 = 0, ly1 = 0, ry0 = 0, ry1 = 0;

    auto eval_fit = [](const double c[3], int y, int y0, int y1) {
        const double span = static_cast<double>(y1 - y0);
        if (span < 1.0) return c[2];
        const double t = static_cast<double>(y - y0) / span;
        return c[0] * t * t + c[1] * t + c[2];
    };

    if (two_lanes) {
        std::vector<int> centers;
        centers.reserve(out.left_pts.size());
        for (size_t k = 0; k < out.left_pts.size(); ++k) {
            centers.push_back((out.left_pts[k].x + out.right_pts[k].x) / 2);
        }
        std::sort(centers.begin(), centers.end());

        const size_t mid = centers.size() / 2;
        int centre_px = (centers.size() % 2 == 1)
            ? centers[mid]
            : (centers[mid - 1] + centers[mid]) / 2;

        out.fit_ok = fit_poly2(out.left_pts, lf, ly0, ly1)
                  && fit_poly2(out.right_pts, rf, ry0, ry1);

        if (out.fit_ok) {
            out.curve_px = static_cast<float>(0.5 * (lf[0] + rf[0])); // Cập nhật curve_px
            const double cx = 0.5 * (
                eval_fit(lf, y_look, ly0, ly1) + eval_fit(rf, y_look, ry0, ry1)
            );
            if (std::isfinite(cx) && cx > 0.0 && cx < WORK_W) {
                centre_px = static_cast<int>(std::lround(cx));
            } else {
                out.fit_ok = false;
            }
        }

        const double scale = frame.cols / static_cast<double>(WORK_W);
        const double dev = (centre_px - WORK_W / 2) * scale;

        dev_ema_ = static_cast<int>(std::lround(
            EMA_ALPHA * dev + (1.0 - EMA_ALPHA) * dev_ema_
        ));

        out.dev_cm = px_to_cm(static_cast<float>(centre_px - WORK_W / 2), y_look);
        center_x = centre_px;
    }

    // ---- 9. Giu gia tri khi mat lan -------------------------------------
    out.two_lanes = two_lanes;
    out.state = state;
    lost_frames_ = (state == LaneState::LOST) ? lost_frames_ + 1 : 0;
    out.dev_px = (state != LaneState::LOST || lost_frames_ <= HOLD_FRAMES) ? dev_ema_ : 0;

    // ---- 10. Ve anh quan sat -----------------------------------------------
    out.vis = frame.clone();
    const double sx = out.vis.cols / static_cast<double>(WORK_W);
    const double sy = out.vis.rows / static_cast<double>(WORK_H);

    auto to_vis = [&](int x, int y) {
        return cv::Point(cvRound(x * sx), cvRound(y * sy));
    };

    std::vector<cv::Point> roi_vis;
    roi_vis.reserve(roi_poly.size());
    for (const auto& p : roi_poly) roi_vis.push_back(to_vis(p.x, p.y));
    cv::polylines(out.vis, roi_vis, true, cv::Scalar(255, 200, 0), 2, cv::LINE_AA);

    cv::line(out.vis, to_vis(0, HORIZON_Y), to_vis(WORK_W, HORIZON_Y), cv::Scalar(255, 0, 255), 1);
    cv::putText(out.vis, "horizon", to_vis(4, HORIZON_Y - 4), 
                cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(255, 0, 255), 1, cv::LINE_AA);

    cv::line(out.vis, to_vis(0, y_look), to_vis(WORK_W, y_look), cv::Scalar(0, 255, 255), 1);

    for (int i = 0; i < N_WINDOWS; ++i) {
        int y0 = 0, y1 = 0;
        band_of(i, y0, y1);
        if (y1 - y0 < 2) continue;
        cv::rectangle(out.vis, to_vis(0, y0), to_vis(WORK_W, y1),
                      band_ok[i] ? cv::Scalar(70, 70, 70) : cv::Scalar(0, 0, 255), 1);
    }

    if (!out.left_pts.empty()) {
        std::vector<cv::Point> vis_left, vis_right;
        for (const auto& p : out.left_pts) vis_left.push_back(to_vis(p.x, p.y));
        for (const auto& p : out.right_pts) vis_right.push_back(to_vis(p.x, p.y));

        cv::polylines(out.vis, vis_left, false, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
        cv::polylines(out.vis, vis_right, false, cv::Scalar(255, 0, 0), 2, cv::LINE_AA);

        cv::circle(out.vis, to_vis(seed_left, seed_y), 7, cv::Scalar(0, 255, 255), 2);
        cv::circle(out.vis, to_vis(seed_right, seed_y), 7, cv::Scalar(0, 255, 255), 2);
    }

    // Đổi if (two_lanes) thành if (state != LaneState::LOST) để vẽ tâm cả khi có 1 làn hoặc đang Hold
    if (state != LaneState::LOST) {
        cv::line(out.vis, to_vis(center_x, top), to_vis(center_x, bottom), cv::Scalar(0, 255, 255), 2);
    }

    if (out.fit_ok) {
        auto draw_fit = [&](const double c[3], int y0, int y1, const cv::Scalar& col) {
            std::vector<cv::Point> poly;
            for (int k = 0; k <= 20; ++k) {
                const int y = y0 + (y1 - y0) * k / 20;
                const double x = eval_fit(c, y, y0, y1);
                if (!std::isfinite(x)) continue;
                poly.push_back(to_vis(cvRound(x), y));
            }
            if (poly.size() > 1) cv::polylines(out.vis, poly, false, col, 2, cv::LINE_AA);
        };

        draw_fit(lf, ly0, ly1, cv::Scalar(0, 200, 0));
        draw_fit(rf, ry0, ry1, cv::Scalar(255, 120, 0));
        cv::circle(out.vis, to_vis(center_x, y_look), 6, cv::Scalar(0, 255, 255), 2);
    }

    const cv::Scalar flag_color = two_lanes ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
    cv::putText(out.vis, two_lanes ? "2 LANES OK" : (state == LaneState::ONE_LINE ? "1 LANE TRACK" : "LOST"),
                cv::Point(12, 34), cv::FONT_HERSHEY_SIMPLEX, 1.0, flag_color, 2, cv::LINE_AA);

    const std::string detail = "dev=" + std::to_string(out.dev_px)
        + "px " + std::to_string(static_cast<int>(std::lround(out.dev_cm))) + "cm"
        + " w=" + std::to_string(static_cast<int>(std::lround(out.lane_width_cm))) + "cm"
        + " c=" + std::to_string(out.curve_px);

    auto dist_text = [](int y) {
        const int dy = y - HORIZON_Y;
        return dy <= 1 ? std::string("inf") : std::to_string(dy);
    };
    const std::string range = "vung " + std::to_string(y_look - HORIZON_Y)
            + "-" + std::to_string(bottom - HORIZON_Y) + "px duoi horizon"
            + " [" + dist_text(top) + ".." + dist_text(bottom) + "]";

    cv::putText(out.vis, range, cv::Point(12, 84), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 200, 0), 1, cv::LINE_AA);
    cv::putText(out.vis, detail, cv::Point(12, 64), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

    out.proc_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_begin).count();
    return two_lanes;
}

// ============================================================================
// TIM CAP 2 VANH (Giữ nguyên)
// ============================================================================
bool CameraLane::seed_pair_from(const cv::Mat& col_sum, int& seed_left, int& seed_right) {
    std::vector<std::pair<int, int>> runs;
    std::pair<int, int> run = {0, 0};
    bool inside = false;

    for (int x = 0; x < col_sum.cols; ++x) {
        const int v = col_sum.at<int>(0, x);
        const bool on = v >= WINDOW_MIN_POINTS;
        if (on && !inside) { run = {x, x}; inside = true; }
        else if (on) { run.second = x; }
        else if (inside) { runs.push_back(run); inside = false; }
    }
    if (inside) runs.push_back(run);

    std::vector<std::pair<int, int>> keep;
    for (const auto& r : runs) {
        if ((r.second - r.first + 1) <= MAX_RUN_WIDTH) keep.push_back(r);
    }

    if (keep.size() < 2) return false;

    const auto& left_run = keep.front();
    const auto& right_run = keep.back();
    seed_left = (left_run.first + left_run.second) / 2;
    seed_right = (right_run.first + right_run.second) / 2;

    const int gap = seed_right - seed_left;
    return gap >= MIN_LANE_GAP && gap <= LANE_WIDTH_MAX;
}

// ============================================================================
// TIM DINH COT CUA 1 VANH (Giữ nguyên)
// ============================================================================
bool CameraLane::peak_in(const cv::Mat& col_sum, int seed, int& peak) {
    const int lo = std::max(0, seed - WINDOW_MARGIN);
    const int hi = std::min(col_sum.cols - 1, seed + WINDOW_MARGIN);
    int best_x = -1, best_v = -1;

    for (int x = lo; x <= hi; ++x) {
        const int v = col_sum.at<int>(0, x);
        if (v > best_v) { best_v = v; best_x = x; }
    }

    if (best_x < 0 || best_v < WINDOW_MIN_POINTS) return false;

    int width = 1;
    for (int x = best_x - 1; x >= lo && col_sum.at<int>(0, x) >= WINDOW_MIN_POINTS; --x) ++width;
    for (int x = best_x + 1; x <= hi && col_sum.at<int>(0, x) >= WINDOW_MIN_POINTS; ++x) ++width;

    if (width > MAX_RUN_WIDTH) return false;
    peak = best_x;
    return true;
}

// ============================================================================
// FIT DUONG BAC 2 (Giữ nguyên)
// ============================================================================
bool CameraLane::fit_poly2(const std::vector<cv::Point>& pts, double coef[3], int& y0, int& y1) {
    coef[0] = coef[1] = coef[2] = 0.0;
    y0 = y1 = 0;

    if (pts.size() < 3) return false;

    int lo = pts.front().y, hi = pts.front().y;
    for (const auto& p : pts) {
        lo = std::min(lo, p.y);
        hi = std::max(hi, p.y);
    }

    if (hi - lo < 4) return false;

    y0 = lo; y1 = hi;
    const double inv = 1.0 / static_cast<double>(hi - lo);

    cv::Mat A = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat b = cv::Mat::zeros(3, 1, CV_64F);
    double* Ad = A.ptr<double>();
    double* bd = b.ptr<double>();

    for (const auto& p : pts) {
        const double t = static_cast<double>(p.y - lo) * inv;
        const double t2 = t * t;
        const double x = p.x;

        Ad[0] += t2 * t2; Ad[1] += t2 * t; Ad[2] += t2;
        Ad[4] += t * t;   Ad[5] += t;      Ad[8] += 1.0;

        bd[0] += t2 * x; bd[1] += t * x; bd[2] += x;
    }

    Ad[3] = Ad[1]; Ad[6] = Ad[2]; Ad[7] = Ad[5];

    cv::Mat sol;
    if (!cv::solve(A, b, sol, cv::DECOMP_SVD)) return false;

    for (int k = 0; k < 3; ++k) {
        coef[k] = sol.at<double>(k);
        if (!std::isfinite(coef[k])) return false;
    }

    if (std::fabs(coef[0]) > 4.0 * WORK_W) return false;
    return true;
}

// ============================================================================
// PX SANG CENTIMET (Giữ nguyên)
// ============================================================================
float CameraLane::px_to_cm(float px, int y) {
    const int dy = y - HORIZON_Y;
    if (dy <= 1) return 0.0f; 
    return px * CAMERA_HEIGHT_M * 100.0f / static_cast<float>(dy);
}