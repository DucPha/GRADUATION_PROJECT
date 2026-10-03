#include "camera_node.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <utility>

// ============================================================================
// CONSTRUCTOR / DESTRUCTOR
// ============================================================================

CameraLane::CameraLane(
    int camera_index,
    int target_fps
)
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
    if (running_.load()) {
        return true;
    }

    if (!open_camera()) {
        return false;
    }

    running_.store(true);
    worker_ = std::thread(&CameraLane::capture_loop, this);
    return true;
}

void CameraLane::stop() {
    running_.store(false);

    if (worker_.joinable()) {
        worker_.join();
    }

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
        cap_.set(cv::CAP_PROP_FPS, target_fps_);

        // Doc thu mot frame de chan do loi "mo ra nhung khong co hinh"
        cv::Mat probe;
        if (!cap_.read(probe) || probe.empty()) {
            cap_.release();
            continue;
        }

        camera_index_ = idx;
        std::cout << "[CameraLane] Camera opened at index "
                  << idx << ", real size "
                  << probe.cols << "x" << probe.rows << "\n";

        if (probe.cols != WORK_W * 2 || probe.rows != WORK_H * 2) {
            std::cout << "[CameraLane] Expected "
                      << (WORK_W * 2) << "x" << (WORK_H * 2)
                      << ". Detector will scale the coordinates, but check the "
                         "camera resolution setting.\n";
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

void CameraLane::get_latest(
    LaneOutput& out,
    bool copy_vis
) const {
    std::lock_guard<std::mutex> lock(mtx_);

    // Vong dieu khien 100 Hz khong can anh, chi luong ve 10 Hz moi can
    out.vis = copy_vis ? latest_.vis : cv::Mat();

    out.left_pts = latest_.left_pts;
    out.right_pts = latest_.right_pts;
    out.two_lanes = latest_.two_lanes;
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

bool CameraLane::detect(
    const cv::Mat& frame,
    LaneOutput& out
) {
    const auto t_begin = std::chrono::steady_clock::now();

    cv::Mat work;
    cv::resize(frame, work, cv::Size(WORK_W, WORK_H), 0, 0, cv::INTER_AREA);

    // ---- Vung lam viec --------------------------------------------------
    // Tinh truoc, de mask duoc dung truoc khi nguong
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

    // ---- 3. Xam -> 4. GaussianBlur -> 5. CLAHE --------------------------
    cv::Mat gray;
    cv::Mat bin;
    cv::cvtColor(work, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(BLUR_KSIZE, BLUR_KSIZE), 0);
    clahe_->apply(gray, gray);

    // ---- 2. ROI hinh thang ---------------------------------------------
    // Tren hep, duoi rong: bo trii va 2 ben ngoai (cot tru, tuong, van can)
    cv::Mat mask = cv::Mat::zeros(WORK_H, WORK_W, CV_8U);
    const int half_top = WORK_W / 6;
    const int half_bot = WORK_W / 2 - 2;
    const std::vector<cv::Point> roi_poly = {
        cv::Point(WORK_W / 2 - half_top, top),
        cv::Point(WORK_W / 2 + half_top, top),
        cv::Point(WORK_W / 2 + half_bot, bottom),
        cv::Point(WORK_W / 2 - half_bot, bottom),
    };
    cv::fillPoly(mask, roi_poly, cv::Scalar(255));
    cv::bitwise_and(gray, gray, mask);

    // ---- 6. Nguong Otsu + morphology ------------------------------------
    cv::threshold(gray, bin, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
    cv::morphologyEx(
        bin, bin, cv::MORPH_CLOSE,
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 15))
    );
    cv::morphologyEx(
        bin, bin, cv::MORPH_OPEN,
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3))
    );

    // ---- Qua tu cua so XA nhat toi cua so GAN xe ----------------------
    // Chay i = N-1 .. 0: i = N-1 la cua so xa nhat (2 vanh hep nhat),
    // i = 0 la cua so gan xe (2 vanh rong nhat).
    std::vector<bool> band_ok(N_WINDOWS, false);
    std::vector<int> peak_left(N_WINDOWS, -1);
    std::vector<int> peak_right(N_WINDOWS, -1);

    int seed_left = -1;
    int seed_right = -1;
    int seed_y = 0;
    int matched = 0;
    int center_x = WORK_W / 2;

    for (int i = N_WINDOWS - 1; i >= 0; --i) {
        // Het so cua so duoc phep dung de tim 2 vanh
        if (seed_left < 0 && i < N_WINDOWS - MAX_SEED_SEARCH) {
            break;
        }

        int y0 = 0;
        int y1 = 0;
        band_of(i, y0, y1);
        if (y1 - y0 < 2) {
            continue;
        }

        cv::Mat col_sum;
        cv::reduce(
            bin(cv::Rect(0, y0, WORK_W, y1 - y0)),
            col_sum, 0, cv::REDUCE_SUM, CV_32S
        );

        // Chua co 2 vanh -> thu tim cap vanh o cua so nay
        if (seed_left < 0) {
            int left = 0;
            int right = 0;
            if (!seed_pair_from(col_sum, left, right)) {
                continue;
            }
            seed_left = left;
            seed_right = right;
            seed_y = (y0 + y1) / 2;
        }

        int left = 0;
        int right = 0;
        if (!peak_in(col_sum, seed_left, left) || !peak_in(col_sum, seed_right, right)) {
            continue;
        }

        // 2 vanh lao cheo nhau, hoac 2 dinh qua sat -> bo cua so nay
        if (left >= right || (right - left) < MIN_LANE_GAP) {
            continue;
        }

        seed_left = left;
        seed_right = right;

        band_ok[i] = true;
        peak_left[i] = left;
        peak_right[i] = right;
        ++matched;

        const int y_center = (y0 + y1) / 2;
        out.left_pts.push_back({left, y_center});
        out.right_pts.push_back({right, y_center});
        center_x = (left + right) / 2;
    }

// ---- Quyet dinh co nhan duoc 2 vanh khong ---------------------------
bool two_lanes = matched >= MIN_MATCHED_WINDOWS;

if (two_lanes) {
        // Be rong 2 vanh do o cua so GAN XE (i = 0), noi 2 vanh rong nhat.
        // Khong do o cua so xa (i = N-1): o do lan hep nhat, sat nguong
        // LANE_WIDTH_MIN, de bi lo oi khi lan thon.
        int width = 0;
        int y_near = bottom;
        for (int i = 0; i < N_WINDOWS; ++i) {
            if (band_ok[i]) {
                width = peak_right[i] - peak_left[i];
                int y0 = 0;
                int y1 = 0;
                band_of(i, y0, y1);
                y_near = (y0 + y1) / 2;
                break;
            }
        }
        two_lanes = width >= LANE_WIDTH_MIN && width <= LANE_WIDTH_MAX;

        if (two_lanes) {
            // Do ra centimet bang cong thuc pinhole
            out.lane_width_cm = px_to_cm(static_cast<float>(width), y_near);
        }
    }

    // Hang lay sai lech: nhin truoc, khong phai sat bumper
    const int y_look = top + static_cast<int>((bottom - top) * LOOKAHEAD_FRAC);

    // ---- 9. Fit duong bac 2 ---------------------------------------------
    double lf[3] = {0, 0, 0};
    double rf[3] = {0, 0, 0};
    int ly0 = 0, ly1 = 0, ry0 = 0, ry1 = 0;

    auto eval_fit = [](const double c[3], int y, int y0, int y1) {
        const double span = static_cast<double>(y1 - y0);
        if (span < 1.0) {
            return c[2];
        }
        const double t = static_cast<double>(y - y0) / span;
        return c[0] * t * t + c[1] * t + c[2];
    };

    if (two_lanes) {
        // Trung vi 5 cua so: luon co, dung khi fit that bai
        std::vector<int> centers;
        centers.reserve(out.left_pts.size());
        for (size_t k = 0; k < out.left_pts.size(); ++k) {
            centers.push_back(
                (out.left_pts[k].x + out.right_pts[k].x) / 2
            );
        }
        std::sort(centers.begin(), centers.end());

        const size_t mid = centers.size() / 2;
        int centre_px = (centers.size() % 2 == 1)
            ? centers[mid]
            : (centers[mid - 1] + centers[mid]) / 2;

        // Uu tien duong fit bac 2: it rung hon trung vi cua 5 diem roi rac
        out.fit_ok = fit_poly2(out.left_pts, lf, ly0, ly1)
                  && fit_poly2(out.right_pts, rf, ry0, ry1);

        if (out.fit_ok) {
            const double cx = 0.5 * (
                eval_fit(lf, y_look, ly0, ly1) + eval_fit(rf, y_look, ry0, ry1)
            );
            if (std::isfinite(cx) && cx > 0.0 && cx < WORK_W) {
                centre_px = static_cast<int>(std::lround(cx));
            } else {
                out.fit_ok = false;
            }
        }

        // Do lech tinh theo pixel anh goc de giu nguyen quy uoc da tinh
        // truoc day tren may
        const double scale = frame.cols / static_cast<double>(WORK_W);
        const double dev = (centre_px - WORK_W / 2) * scale;

        dev_ema_ = static_cast<int>(std::lround(
            EMA_ALPHA * dev + (1.0 - EMA_ALPHA) * dev_ema_
        ));

        out.dev_cm = px_to_cm(
            static_cast<float>(centre_px - WORK_W / 2), y_look
        );
        center_x = centre_px;
    }

    out.two_lanes = two_lanes;
    out.dev_px = two_lanes ? dev_ema_ : 0;

    // ---- Ve anh quan sat -----------------------------------------------
    out.vis = frame.clone();

    const double sx = out.vis.cols / static_cast<double>(WORK_W);
    const double sy = out.vis.rows / static_cast<double>(WORK_H);

    auto to_vis = [&](int x, int y) {
        return cv::Point(cvRound(x * sx), cvRound(y * sy));
    };

    // ---- Ve vung camera dang nhin --------------------------------------
    // Dung lai roi_poly cua mask nen hinh ve dung vung detector doc
    std::vector<cv::Point> roi_vis;
    roi_vis.reserve(roi_poly.size());
    for (const auto& p : roi_poly) {
        roi_vis.push_back(to_vis(p.x, p.y));
    }
    cv::polylines(out.vis, roi_vis, true, cv::Scalar(255, 200, 0), 2, cv::LINE_AA);

    // Duong chan troi: moc de hieu chinh HORIZON_Y
    cv::line(
        out.vis, to_vis(0, HORIZON_Y), to_vis(WORK_W, HORIZON_Y),
        cv::Scalar(255, 0, 255), 1
    );
    cv::putText(
        out.vis, "horizon",
        to_vis(4, HORIZON_Y - 4),
        cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(255, 0, 255), 1, cv::LINE_AA
    );

    // Hang lay dev
    cv::line(
        out.vis, to_vis(0, y_look), to_vis(WORK_W, y_look),
        cv::Scalar(0, 255, 255), 1
    );

    for (int i = 0; i < N_WINDOWS; ++i) {
        int y0 = 0;
        int y1 = 0;
        band_of(i, y0, y1);
        if (y1 - y0 < 2) {
            continue;
        }
        // Cua so bi lo mau do, cua so dung mau xam
        cv::rectangle(
            out.vis, to_vis(0, y0), to_vis(WORK_W, y1),
            band_ok[i] ? cv::Scalar(70, 70, 70) : cv::Scalar(0, 0, 255),
            1
        );
    }

    if (!out.left_pts.empty()) {
        std::vector<cv::Point> vis_left;
        std::vector<cv::Point> vis_right;
        vis_left.reserve(out.left_pts.size());
        vis_right.reserve(out.right_pts.size());

        for (const auto& p : out.left_pts) {
            vis_left.push_back(to_vis(p.x, p.y));
        }
        for (const auto& p : out.right_pts) {
            vis_right.push_back(to_vis(p.x, p.y));
        }

        cv::polylines(out.vis, vis_left, false, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
        cv::polylines(out.vis, vis_right, false, cv::Scalar(255, 0, 0), 2, cv::LINE_AA);

        cv::circle(out.vis, to_vis(seed_left, seed_y), 7, cv::Scalar(0, 255, 255), 2);
        cv::circle(out.vis, to_vis(seed_right, seed_y), 7, cv::Scalar(0, 255, 255), 2);
    }

    if (two_lanes) {
        cv::line(
            out.vis, to_vis(center_x, top), to_vis(center_x, bottom),
            cv::Scalar(0, 255, 255), 2
        );
    }

    // Duong fit bac 2: it rung hon duong noi 5 diem cua so
    if (out.fit_ok) {
        auto draw_fit = [&](const double c[3], int y0, int y1, const cv::Scalar& col) {
            std::vector<cv::Point> poly;
            poly.reserve(21);
            for (int k = 0; k <= 20; ++k) {
                const int y = y0 + (y1 - y0) * k / 20;
                const double x = eval_fit(c, y, y0, y1);
                if (!std::isfinite(x)) {
                    continue;
                }
                poly.push_back(to_vis(cvRound(x), y));
            }
            if (poly.size() > 1) {
                cv::polylines(out.vis, poly, false, col, 2, cv::LINE_AA);
            }
        };

        draw_fit(lf, ly0, ly1, cv::Scalar(0, 200, 0));
        draw_fit(rf, ry0, ry1, cv::Scalar(255, 120, 0));

        // Diem nam tinh sai lech
        cv::circle(
            out.vis, to_vis(center_x, y_look), 6, cv::Scalar(0, 255, 255), 2
        );
    }

    const cv::Scalar flag_color = two_lanes ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);

    cv::putText(
        out.vis,
        two_lanes ? "2 LANES OK" : "NO 2 LANES",
        cv::Point(12, 34),
        cv::FONT_HERSHEY_SIMPLEX, 1.0, flag_color, 2, cv::LINE_AA
    );

    const std::string detail = "dev=" + std::to_string(out.dev_px)
        + "px " + std::to_string(static_cast<int>(std::lround(out.dev_cm))) + "cm"
        + " w=" + std::to_string(static_cast<int>(std::lround(out.lane_width_cm))) + "cm";

    // Quang cach vung detector: tren cung voi chan troi thi khong do duoc
    auto dist_text = [](int y) {
        const int dy = y - HORIZON_Y;
        return dy <= 1 ? std::string("inf") : std::to_string(dy);
    };
    const std::string range = "vung " + std::to_string(y_look - HORIZON_Y)
            + "-" + std::to_string(bottom - HORIZON_Y) + "px duoi horizon"
            + " [" + dist_text(top) + ".." + dist_text(bottom) + "]";

    cv::putText(
        out.vis, range,
        cv::Point(12, 84),
        cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 200, 0), 1, cv::LINE_AA
    );

    cv::putText(
        out.vis, detail,
        cv::Point(12, 64),
        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1, cv::LINE_AA
    );

    out.proc_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t_begin
    ).count();

    return two_lanes;
}

// ============================================================================
// TIM CAP 2 VANH
// ============================================================================

bool CameraLane::seed_pair_from(
    const cv::Mat& col_sum,
    int& seed_left,
    int& seed_right
) {
    std::vector<std::pair<int, int>> runs;   // [start, end] inclusive
    std::pair<int, int> run = {0, 0};
    bool inside = false;

    for (int x = 0; x < col_sum.cols; ++x) {
        const int v = col_sum.at<int>(0, x);
        const bool on = v >= WINDOW_MIN_POINTS;

        if (on && !inside) {
            run = {x, x};
            inside = true;
        } else if (on) {
            run.second = x;
        } else if (inside) {
            runs.push_back(run);
            inside = false;
        }
    }
    if (inside) {
        runs.push_back(run);
    }

    // Loai run qua rong (bong do, vat can, goc phong)
    std::vector<std::pair<int, int>> keep;
    for (const auto& r : runs) {
        if ((r.second - r.first + 1) <= MAX_RUN_WIDTH) {
            keep.push_back(r);
        }
    }

    if (keep.size() < 2) {
        return false;
    }

    const auto& left_run = keep.front();
    const auto& right_run = keep.back();

    seed_left = (left_run.first + left_run.second) / 2;
    seed_right = (right_run.first + right_run.second) / 2;

    const int gap = seed_right - seed_left;
    return gap >= MIN_LANE_GAP && gap <= LANE_WIDTH_MAX;
}

// ============================================================================
// TIM DINH COT CUA 1 VANH
// ============================================================================

bool CameraLane::peak_in(
    const cv::Mat& col_sum,
    int seed,
    int& peak
) {
    const int lo = std::max(0, seed - WINDOW_MARGIN);
    const int hi = std::min(col_sum.cols - 1, seed + WINDOW_MARGIN);

    int best_x = -1;
    int best_v = -1;

    for (int x = lo; x <= hi; ++x) {
        const int v = col_sum.at<int>(0, x);
        if (v > best_v) {
            best_v = v;
            best_x = x;
        }
    }

    if (best_x < 0 || best_v < WINDOW_MIN_POINTS) {
        return false;
    }

    // Do do rong cua run quanh dinh, run qua rong thi bo qua
    int width = 1;
    for (int x = best_x - 1; x >= lo && col_sum.at<int>(0, x) >= WINDOW_MIN_POINTS; --x) {
        ++width;
    }
    for (int x = best_x + 1; x <= hi && col_sum.at<int>(0, x) >= WINDOW_MIN_POINTS; ++x) {
        ++width;
    }

    if (width > MAX_RUN_WIDTH) {
        return false;
    }

    peak = best_x;
    return true;
}

// ============================================================================
// FIT DUONG BAC 2
// ============================================================================

// Giai x = a*t^2 + b*t + c bang bo phuong trinh binh phuong nho nhat.
// Toa do y duoc chuan hoa ve [0,1] truoc khi giai, nen y^4 khong bao gio
// vuot qua n va he ma tri on dinh.
bool CameraLane::fit_poly2(
    const std::vector<cv::Point>& pts,
    double coef[3],
    int& y0,
    int& y1
) {
    coef[0] = coef[1] = coef[2] = 0.0;
    y0 = 0;
    y1 = 0;

    if (pts.size() < 3) {
        return false;
    }

    int lo = pts.front().y;
    int hi = pts.front().y;
    for (const auto& p : pts) {
        lo = std::min(lo, p.y);
        hi = std::max(hi, p.y);
    }

    // Cac diem sat nhau: duong cong khong xac dinh duoc, de phien ban truoc
    if (hi - lo < 4) {
        return false;
    }

    y0 = lo;
    y1 = hi;
    const double inv = 1.0 / static_cast<double>(hi - lo);

    cv::Mat A = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat b = cv::Mat::zeros(3, 1, CV_64F);
    double* Ad = A.ptr<double>();
    double* bd = b.ptr<double>();

    for (const auto& p : pts) {
        const double t = static_cast<double>(p.y - lo) * inv;
        const double t2 = t * t;
        const double x = p.x;

        Ad[0] += t2 * t2;   // sum t^4
        Ad[1] += t2 * t;    // sum t^3
        Ad[2] += t2;        // sum t^2
        Ad[4] += t * t;     // sum t^2
        Ad[5] += t;         // sum t
        Ad[8] += 1.0;       // sum 1

        bd[0] += t2 * x;
        bd[1] += t * x;
        bd[2] += x;
    }

    // A doi xung
    Ad[3] = Ad[1];
    Ad[6] = Ad[2];
    Ad[7] = Ad[5];

    cv::Mat sol;
    if (!cv::solve(A, b, sol, cv::DECOMP_SVD)) {
        return false;
    }

    for (int k = 0; k < 3; ++k) {
        coef[k] = sol.at<double>(k);
        if (!std::isfinite(coef[k])) {
            return false;
        }
    }

    // Duong cong khong the uon nguoc lai nhieu vong trong khung: so ao
    if (std::fabs(coef[0]) > 4.0 * WORK_W) {
        return false;
    }

    return true;
}

// ============================================================================
// PX SANG CENTIMET
// ============================================================================

// Camera cao h, chan troi o hang cy, dai phang dat. Mot doan rong w_px o hang y
// tuong ung voi W = w_px * h / (y - cy) met tren duong.
// Cong thuc nay cho cung ket qua nhu IPM ma khong can bien doi phoi canh.
float CameraLane::px_to_cm(float px, int y) {
    const int dy = y - HORIZON_Y;
    if (dy <= 1) {
        return 0.0f;   // gan chan troi: khong do duoc
    }
    return px * CAMERA_HEIGHT_M * 100.0f / static_cast<float>(dy);
}