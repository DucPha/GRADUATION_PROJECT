#include "camera_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>
#include <utility>

// ============================================================================
// CONSTRUCTOR / DESTRUCTOR
// ============================================================================

CameraLane::CameraLane(int camera_index, int target_fps,
                       const CameraProfile &profile)
    : camera_index_(camera_index),
      target_fps_(target_fps > 0 ? target_fps : 30),
      roi_top_frac_(std::clamp(profile.roi_top_frac, 0.0f, 0.95f)),
      clahe_(cv::createCLAHE(CLAHE_CLIP, cv::Size(CLAHE_TILE, CLAHE_TILE))) {
  h_ = std::max(0.05f, profile.height_m);
  roi_bottom_frac_ = std::clamp(profile.roi_bottom_frac, 0.1f, 1.0f);

  const double hfov =
      std::clamp(static_cast<double>(profile.hfov_deg), 20.0, 170.0);
  f_px_ = (WORK_W / 2.0) / std::tan(hfov * CV_PI / 360.0);

  pitch_ = profile.axis_ground_m > 0.0f
               ? std::atan2(static_cast<double>(h_),
                            static_cast<double>(profile.axis_ground_m))
               : 0.0;

  // Chan troi = vi tri anh cua huong nam ngang. Cam cui xuong pitch thi
  // chan troi nam tren tam anh f * tan(pitch) hang.
  horizon_y_ = profile.horizon_y != -9999
                   ? profile.horizon_y
                   : static_cast<int>(
                         std::lround(WORK_H / 2.0 - f_px_ * std::tan(pitch_)));

  // Doan rong w_px o hang y ung X = w_px * h / (cos(pitch) * (y - horizon))
  k_ = h_ / static_cast<float>(std::cos(pitch_));
}

CameraLane::~CameraLane() { stop(); }

// ============================================================================
// BAT / TAT
// ============================================================================

bool CameraLane::start() {
  if (running_.load()) {
    return true;
  }

  // Luong cu da tu thoat (mat camera) nhung chua join: gan thread moi len
  // thread con joinable se goi std::terminate
  if (worker_.joinable()) {
    worker_.join();
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

bool CameraLane::is_running() const { return running_.load(); }

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

    // MJPG: YUYV 640x480 an nhieu bang thong USB, nhieu cam bi tut fps
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, WORK_W * 2);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, WORK_H * 2);
    cap_.set(cv::CAP_PROP_FPS, target_fps_);

    // Hang doi V4L2 mac dinh ~4 frame: o 10 fps la tre toi 0.4 s
    cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);

    // Doc thu mot frame de chan loi "mo ra nhung khong co hinh"
    cv::Mat probe;
    if (!cap_.read(probe) || probe.empty()) {
      cap_.release();
      continue;
    }

    // Doc lai fourcc driver that su dang dung: set() co the bi bo qua
    const int fcc = static_cast<int>(cap_.get(cv::CAP_PROP_FOURCC));
    const char fcc_name[5] = {static_cast<char>(fcc & 0xFF),
                              static_cast<char>((fcc >> 8) & 0xFF),
                              static_cast<char>((fcc >> 16) & 0xFF),
                              static_cast<char>((fcc >> 24) & 0xFF), 0};

    camera_index_ = idx;
    std::cout << "[CameraLane] Camera opened at index " << idx << ", real size "
              << probe.cols << "x" << probe.rows << ", format " << fcc_name
              << ", driver fps " << cap_.get(cv::CAP_PROP_FPS) << "\n";

    if (std::string(fcc_name) != "MJPG") {
      std::cout << "[CameraLane] Camera did not accept MJPG (got " << fcc_name
                << "): fps may be limited by USB bandwidth.\n";
    }

    if (probe.cols != WORK_W * 2 || probe.rows != WORK_H * 2) {
      std::cout << "[CameraLane] Expected " << (WORK_W * 2) << "x"
                << (WORK_H * 2)
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

    // Moc thoi gian la luc nhan frame, de age_ms gom ca thoi gian xu ly
    const auto t_grab = std::chrono::steady_clock::now();
    fail_count = 0;

    LaneOutput out;
    out.frame_id = ++frame_id_;
    detect(frame, out);

    {
      std::lock_guard<std::mutex> lock(mtx_);
      latest_ = std::move(out);
      last_frame_time_ = t_grab;
    }
  }

  running_.store(false);
}

void CameraLane::get_latest(LaneOutput &out, bool copy_vis) const {
  std::lock_guard<std::mutex> lock(mtx_);

  // Vong dieu khien 100 Hz khong can anh, chi luong ve 10 Hz moi can
  out.vis = copy_vis ? latest_.vis : cv::Mat();
  out.raw = copy_vis ? latest_.raw : cv::Mat();
  out.roi = copy_vis ? latest_.roi : cv::Mat();
  out.bin = copy_vis ? latest_.bin : cv::Mat();

  out.left_pts = latest_.left_pts;
  out.right_pts = latest_.right_pts;
  out.two_lanes = latest_.two_lanes;
  out.state = latest_.state;
  out.dev_px = latest_.dev_px;
  out.dev_cm = latest_.dev_cm;
  out.lane_width_cm = latest_.lane_width_cm;
  out.path_dx_cm = latest_.path_dx_cm;
  out.curvature = latest_.curvature;
  out.speed_scale = latest_.speed_scale;
  out.fit_ok = latest_.fit_ok;
  out.proc_ms = latest_.proc_ms;
  out.frame_id = latest_.frame_id;

  if (last_frame_time_ == std::chrono::steady_clock::time_point{}) {
    out.age_ms = static_cast<unsigned long>(-1);
    out.stale = true;
    return;
  }

  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::steady_clock::now() - last_frame_time_)
                      .count();

  out.age_ms = (ms < 0) ? 0ul : static_cast<unsigned long>(ms);
  out.stale = out.age_ms > static_cast<unsigned long>(STALE_AGE_MS);
}

// ============================================================================
// DETECTOR
// ============================================================================

bool CameraLane::detect(const cv::Mat &frame, LaneOutput &out) {
  const auto t_begin = std::chrono::steady_clock::now();

  const int cx = WORK_W / 2;
  const double sx = frame.cols / static_cast<double>(WORK_W);
  const double sy = frame.rows / static_cast<double>(WORK_H);

  // ---- 1. Vung lam viec (toa do trong khung WORK_W x WORK_H) ----------
  // Day ROI: theo profile, nhung khong thap hon hang ma be ngang anh con
  // chua duoc 1 lan LANE_W_FIT_M cong le 2 ben (cam cui thi hang gan xe
  // chi thay duoc ~0.7 m, 2 vach khong con nam tron trong anh)
  const int fit_dy =
      static_cast<int>(WORK_W * k_ / (LANE_W_FIT_M + 2.0f * ROI_SIDE_MARGIN_M));
  const int bottom = std::clamp(
      std::min(static_cast<int>(std::lround(roi_bottom_frac_ * WORK_H)),
               horizon_y_ + fit_dy),
      2 * N_WINDOWS + 1, WORK_H);

  int top = static_cast<int>(std::lround(roi_top_frac_.load() * WORK_H));

  // Dinh ROI khong duoc cao hon chan troi + ROI_MIN_DY: o do px_to_cm
  // khong do duoc va vach chi con ~1 pixel
  if (top < horizon_y_ + ROI_MIN_DY) {
    top = horizon_y_ + ROI_MIN_DY;
    if (!warned_roi_) {
      warned_roi_ = true;
      std::cerr << "[CameraLane] ROI top is above horizon+" << ROI_MIN_DY
                << ", clamped to row " << top
                << ". Check horizon_y / axis_ground_m (purple line on vis).\n";
    }
  }
  top = std::clamp(top, 0, bottom - 2 * N_WINDOWS);

  const int roi_h = bottom - top;
  const int win_h = roi_h / N_WINDOWS;

  if (!logged_geometry_) {
    logged_geometry_ = true;
    std::cout << "[CameraLane] geometry: h=" << h_
              << " m, pitch=" << pitch_ * 180.0 / CV_PI << " deg, f=" << f_px_
              << " px, horizon_y=" << horizon_y_
              << "; ground distance at image centre row = "
              << ground_dist_m(WORK_H / 2.0) << " m; ROI rows " << top << ".."
              << bottom << " = " << ground_dist_m(bottom) << ".."
              << ground_dist_m(top) << " m, lane px at ROI bottom = "
              << lane_px(LANE_W_FIT_M, bottom - horizon_y_) << "/" << WORK_W
              << "\n";
  }

  auto band_of = [&](int i, int &y0, int &y1) {
    y1 = bottom - i * win_h;
    y0 = y1 - win_h;
  };

  // ---- 2. Cat ROI, thu ve 320 cot, xam -> blur -> CLAHE ----------------
  // Chi xu ly phan ROI: ~45% so pixel so voi xu ly ca khung
  const int fy0 =
      std::clamp(static_cast<int>(std::lround(top * sy)), 0, frame.rows - 1);
  const int fy1 = std::clamp(static_cast<int>(std::lround(bottom * sy)),
                             fy0 + 1, frame.rows);

  // Anh ROI goc do phan giai (khong resize) cho GUI
  out.roi = frame.rowRange(fy0, fy1).clone();

  cv::Mat gray;
  {
    cv::Mat roi_bgr;
    cv::resize(frame.rowRange(fy0, fy1), roi_bgr, cv::Size(WORK_W, roi_h), 0, 0,
               cv::INTER_AREA);
    cv::cvtColor(roi_bgr, gray, cv::COLOR_BGR2GRAY);
  }
  cv::GaussianBlur(gray, gray, cv::Size(BLUR_KSIZE, BLUR_KSIZE), 0);
  clahe_->apply(gray, gray);

  // ---- 3. ROI hinh thang (toa do cuc bo: hang 0 = top) ------------------
  const int half_top = static_cast<int>(WORK_W * ROI_TOP_HALF_FRAC);
  const int half_bot = cx - 2;
  const std::vector<cv::Point> roi_poly = {
      cv::Point(cx - half_top, top),
      cv::Point(cx + half_top, top),
      cv::Point(cx + half_bot, bottom),
      cv::Point(cx - half_bot, bottom),
  };

  std::vector<cv::Point> poly_local;
  poly_local.reserve(roi_poly.size());
  for (const auto &p : roi_poly) {
    poly_local.emplace_back(p.x, p.y - top);
  }

  cv::Mat mask = cv::Mat::zeros(roi_h, WORK_W, CV_8U);
  cv::fillPoly(mask, poly_local, cv::Scalar(1));

  // ---- 4. Otsu trong hinh thang + morphology ----------------------------
  // bin chi co 0/1 nen tong cot = so pixel vach, khong can chia 255
  double contrast = 0.0;
  const int thr = otsu_masked(gray, mask, contrast);

  cv::Mat bin = cv::Mat::zeros(roi_h, WORK_W, CV_8U);
  if (contrast >= MIN_CONTRAST) {
    cv::threshold(gray, bin, thr, 1, cv::THRESH_BINARY_INV);
    cv::bitwise_and(bin, mask, bin);

    static const cv::Mat k_close =
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 15));
    static const cv::Mat k_open =
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(bin, bin, cv::MORPH_CLOSE, k_close);
    cv::morphologyEx(bin, bin, cv::MORPH_OPEN, k_open);
  }

  // Mask nhi phan cho GUI (0/1, chua resize)
  out.bin = bin.clone();

  // ---- 5. Cua so truot tu XA ve GAN, du doan vi tri theo phoi canh -----
  // Vach cach truc camera x px o hang cach chan troi d0 thi o hang d1 se
  // cach x * d1 / d0. Nho vay WINDOW_MARGIN chi con phai bu cho do cong.
  auto predict = [&](int x, int y_from, int y_to) {
    const int d0 = y_from - horizon_y_;
    const int d1 = y_to - horizon_y_;
    if (d0 <= 1 || d1 <= 1) {
      return x;
    }
    return cx + static_cast<int>(
                    std::lround((x - cx) * static_cast<double>(d1) / d0));
  };

  struct Track {
    std::vector<bool> band_ok;
    std::vector<int> peak_left;
    std::vector<int> peak_right;
    std::vector<cv::Point> left_pts;
    std::vector<cv::Point> right_pts;
    int matched = 0; // so cua so thay du 2 vach
    int nL = 0;      // so cua so thay vach trai
    int nR = 0;      // so cua so thay vach phai
    int score() const { return 2 * matched + nL + nR; }
  };

  // far_to_near = true : tim cap vach dau tien o MAX_SEED_SEARCH cua so xa nhat
  //                      (xa thi lan hep, thay du 2 vach ke ca khi lan rong).
  // far_to_near = false: tim o cua so gan xe, dung khi vao cua gap: vach xa
  //                      da troi ra ngoai hinh thang nhung cua so gan van thay.
  auto run_pass = [&](bool far_to_near) {
    Track t;
    t.band_ok.assign(N_WINDOWS, false);
    t.peak_left.assign(N_WINDOWS, -1);
    t.peak_right.assign(N_WINDOWS, -1);

    int seed_left = -1;
    int seed_right = -1;
    int yl = 0;
    int yr = 0;

    for (int n = 0; n < N_WINDOWS; ++n) {
      if (seed_left < 0 && n >= MAX_SEED_SEARCH) {
        break;
      }

      const int i = far_to_near ? (N_WINDOWS - 1 - n) : n;

      int y0 = 0;
      int y1 = 0;
      band_of(i, y0, y1);
      const int y_mid = (y0 + y1) / 2;
      const int dy = y_mid - horizon_y_;

      cv::Mat col_sum;
      cv::reduce(bin.rowRange(y0 - top, y1 - top), col_sum, 0, cv::REDUCE_SUM,
                 CV_32S);

      if (seed_left < 0) {
        int l = 0;
        int r = 0;
        if (!seed_pair_from(col_sum, dy, win_h, l, r)) {
          continue;
        }
        seed_left = l;
        seed_right = r;
        yl = y_mid;
        yr = y_mid;
      }

      const int pl = predict(seed_left, yl, y_mid);
      const int pr = predict(seed_right, yr, y_mid);

      int left = 0;
      int right = 0;
      const bool okL = peak_in(col_sum, pl, run_limit_px(pl, dy, win_h), left);
      const bool okR = peak_in(col_sum, pr, run_limit_px(pr, dy, win_h), right);

      if (!okL && !okR) {
        continue;
      }

      // 2 vach cheo nhau hoac qua sat -> bo cua so, khong cap nhat seed
      if (okL && okR &&
          (left >= right || right - left < lane_px(MIN_LANE_GAP_M, dy))) {
        continue;
      }

      if (okL) {
        seed_left = left;
        yl = y_mid;
        t.peak_left[i] = left;
        ++t.nL;
      }
      if (okR) {
        seed_right = right;
        yr = y_mid;
        t.peak_right[i] = right;
        ++t.nR;
      }

      if (okL && okR) {
        t.band_ok[i] = true;
        ++t.matched;
        t.left_pts.push_back({left, y_mid});
        t.right_pts.push_back({right, y_mid});
      }
    }

    return t;
  };

  Track trk = run_pass(true);
  if (trk.matched < MIN_MATCHED_WINDOWS) {
    Track alt = run_pass(false);
    if (alt.score() > trk.score()) {
      trk = std::move(alt);
    }
  }

  const std::vector<bool> &band_ok = trk.band_ok;
  const std::vector<int> &peak_left = trk.peak_left;
  const std::vector<int> &peak_right = trk.peak_right;
  const int matched = trk.matched;
  const int nL = trk.nL;
  const int nR = trk.nR;
  out.left_pts = trk.left_pts;
  out.right_pts = trk.right_pts;

  // ---- 6. Du 2 vach khong? Kiem tra be rong bang met --------------------
  // Do o cua so GAN XE nhat co du 2 vach (vach rong nhat, do chinh xac nhat)
  bool two_lanes = matched >= MIN_MATCHED_WINDOWS;

  if (two_lanes) {
    two_lanes = false;
    for (int i = 0; i < N_WINDOWS; ++i) {
      if (!band_ok[i]) {
        continue;
      }
      int y0 = 0;
      int y1 = 0;
      band_of(i, y0, y1);
      const float width_cm = px_to_cm(
          static_cast<float>(peak_right[i] - peak_left[i]), (y0 + y1) / 2);
      const float width_m = width_cm / 100.0f;

      two_lanes = width_m >= LANE_W_MIN_M && width_m <= LANE_W_MAX_M;
      if (two_lanes) {
        out.lane_width_cm = width_cm;
        lane_w_m_ += LANE_W_ALPHA * (width_m - lane_w_m_);
      }
      break;
    }
  }

  // ---- 7. Diem tam lan cua tung cua so ----------------------------------
  // 2 vach: trung diem. 1 vach: vach +/- nua be rong lan (da hoc) doi ra
  // pixel theo hang do. Nho vay vao cua, khi vach trong ra khoi khung, cac
  // cua so gan xe van co diem tam thay vi bi bo.
  // Neu khong du 2 vach thi chi dung ben co nhieu cua so hon.
  const bool use_left = nL >= nR;
  const bool use_L = two_lanes || use_left;
  const bool use_R = two_lanes || !use_left;

  std::vector<cv::Point> cpts; // xa -> gan
  cpts.reserve(N_WINDOWS);

  for (int i = N_WINDOWS - 1; i >= 0; --i) {
    int y0 = 0;
    int y1 = 0;
    band_of(i, y0, y1);
    const int y_mid = (y0 + y1) / 2;
    const int dy = y_mid - horizon_y_;
    if (dy <= 1) {
      continue;
    }

    const bool has_l = use_L && peak_left[i] >= 0;
    const bool has_r = use_R && peak_right[i] >= 0;
    const int half = lane_px(0.5f * lane_w_m_, dy);

    if (has_l && has_r) {
      cpts.emplace_back((peak_left[i] + peak_right[i]) / 2, y_mid);
    } else if (has_l) {
      cpts.emplace_back(peak_left[i] + half, y_mid);
    } else if (has_r) {
      cpts.emplace_back(peak_right[i] - half, y_mid);
    }
  }

  LaneState state = LaneState::LOST;
  if (two_lanes) {
    state = LaneState::TWO_LINES;
  } else if (static_cast<int>(cpts.size()) >= MIN_ONE_SIDE_WINDOWS) {
    state = LaneState::ONE_LINE;
  }

  // ---- 8. Fit duong tam lan, do lech tai hang nhin truoc ---------------
  const int y_look =
      top + static_cast<int>(std::lround(roi_h * LOOKAHEAD_FRAC));

  double cf[3] = {0, 0, 0};
  int cy0 = 0;
  int cy1 = 0;

  auto eval_fit = [](const double c[3], int y, int y0, int y1) {
    const double span = static_cast<double>(y1 - y0);
    if (span < 1.0) {
      return c[2];
    }
    // Gioi han ngoai suy: bac 2 ra xa vung fit rat nhanh vo nghia
    const double t =
        std::clamp(static_cast<double>(y - y0) / span, -0.25, 1.25);
    return c[0] * t * t + c[1] * t + c[2];
  };

  int centre_look = cx;

  if (state != LaneState::LOST) {
    bool have = false;

    out.fit_ok = fit_poly2(cpts, cf, cy0, cy1);
    if (out.fit_ok) {
      const double fx = eval_fit(cf, y_look, cy0, cy1);
      if (std::isfinite(fx) && fx > 0.0 && fx < WORK_W) {
        centre_look = static_cast<int>(std::lround(fx));
        have = true;
      } else {
        out.fit_ok = false;
      }
    }

    if (!have) {
      // Fit that bai: lay diem tam gan hang nhin truoc nhat, quy ve
      // hang y_look bang ti le phoi canh de dev_px cung quy uoc
      const cv::Point *best = &cpts.front();
      for (const auto &p : cpts) {
        if (std::abs(p.y - y_look) < std::abs(best->y - y_look)) {
          best = &p;
        }
      }
      const double k = static_cast<double>(y_look - horizon_y_) /
                       static_cast<double>(best->y - horizon_y_);
      centre_look =
          std::clamp(cx + static_cast<int>(std::lround((best->x - cx) * k)), 0,
                     WORK_W - 1);
    }

    // Lech ngang (cm) cua duong tam o xa so voi gan, do bang met o tung
    // hang. = 0 khi duong tam cung huong xe (thang, xe di dung huong);
    // khac 0 khi vao cua hoac xe dang chech huong. > 0: duong re sang phai.
    {
      const cv::Point &pf = cpts.front();
      const cv::Point &pn = cpts.back();
      const double xf = out.fit_ok ? eval_fit(cf, cy0, cy0, cy1) : pf.x;
      const double xn = out.fit_ok ? eval_fit(cf, cy1, cy0, cy1) : pn.x;
      const double Xf = (xf - cx) * k_ / (pf.y - horizon_y_);
      const double Xn = (xn - cx) * k_ / (pn.y - horizon_y_);
      out.path_dx_cm = static_cast<float>(100.0 * (Xf - Xn));
    }

    // Do cong: fit X(Z) = c0 + c1*Z + c2*Z^2 bang met that o tung hang, khi
    // do 1/R = 2*c2. Z la khoang cach ngang tu chan cam toi diem tren dat.
    if (cpts.size() >= 4) {
      std::vector<double> zs;
      std::vector<double> xs;
      for (const auto &p : cpts) {
        const double z = ground_dist_m(p.y);
        const int d = p.y - horizon_y_;
        if (std::isfinite(z) && d > 1) {
          zs.push_back(z);
          xs.push_back((p.x - cx) * static_cast<double>(k_) / d);
        }
      }
      if (zs.size() >= 4 && std::fabs(zs.front() - zs.back()) >= 0.3) {
        const double zm = 0.5 * (zs.front() + zs.back());
        for (auto &z : zs) {
          z -= zm;
        }
        double q[3];
        if (fit_quad(zs, xs, q)) {
          out.curvature = static_cast<float>(2.0 * q[2]);
        }
      }
    }

    const double off = static_cast<double>(centre_look - cx);
    const double raw = off * sx; // pixel anh goc, quy uoc cu cua firmware

    if (!ema_primed_) {
      dev_ema_ = static_cast<int>(std::lround(raw));
      ema_primed_ = true;
    } else {
      dev_ema_ = static_cast<int>(
          std::lround(EMA_ALPHA * raw + (1.0 - EMA_ALPHA) * dev_ema_));
    }

    out.dev_cm = px_to_cm(static_cast<float>(off), y_look);
  }

  // ---- 9. Giu gia tri khi mat lan, he so toc do ------------------------
  if (state != LaneState::LOST) {
    last_valid_time_ = t_begin;
  }
  const auto lost_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                           t_begin - last_valid_time_)
                           .count();
  const bool holding = state == LaneState::LOST && lost_ms <= HOLD_MS;

  // Mat lan qua lau: lan sau bat lai lai thi bo qua EMA cu
  if (state == LaneState::LOST && !holding) {
    ema_primed_ = false;
  }

  out.two_lanes = two_lanes;
  out.state = state;
  out.dev_px = (state != LaneState::LOST || holding) ? dev_ema_ : 0;

  float speed = 1.0f;
  if (state == LaneState::LOST) {
    speed = SPEED_LOST;
  } else {
    const float k = std::clamp((std::fabs(out.path_dx_cm) - PATH_DX_START_CM) /
                                   (PATH_DX_FULL_CM - PATH_DX_START_CM),
                               0.0f, 1.0f);
    const float kc = std::clamp((std::fabs(out.curvature) - CURV_START) /
                                    (CURV_FULL - CURV_START),
                                0.0f, 1.0f);
    speed = 1.0f - std::max(k, kc) * (1.0f - SPEED_MIN_CURVE);

    // It cua so thay vach = camera dang kho do (vao cua, thieu sang):
    // khong biet do cong thi cung giam toc
    if (state == LaneState::ONE_LINE ||
        static_cast<int>(cpts.size()) <= MIN_MATCHED_WINDOWS) {
      speed = std::min(speed, SPEED_ONE_LINE);
    }
  }
  out.speed_scale = speed;

  // ---- 10. Ve anh quan sat ----------------------------------------------
  out.raw = frame.clone();
  out.vis = out.raw.clone();

  auto to_vis = [&](int x, int y) {
    return cv::Point(cvRound(x * sx), cvRound(y * sy));
  };

  std::vector<cv::Point> roi_vis;
  roi_vis.reserve(roi_poly.size());
  for (const auto &p : roi_poly) {
    roi_vis.push_back(to_vis(p.x, p.y));
  }
  cv::polylines(out.vis, roi_vis, true, cv::Scalar(255, 200, 0), 2,
                cv::LINE_AA);

  // Chan troi: do lai de horizon_y dung. Cam cui xuong thi nam ngoai anh.
  if (horizon_y_ >= 0 && horizon_y_ < WORK_H) {
    cv::line(out.vis, to_vis(0, horizon_y_), to_vis(WORK_W, horizon_y_),
             cv::Scalar(255, 0, 255), 1);
    cv::putText(out.vis, "horizon", to_vis(4, horizon_y_ - 4),
                cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(255, 0, 255), 1,
                cv::LINE_AA);
  }

  // Hang lay do lech
  cv::line(out.vis, to_vis(0, y_look), to_vis(WORK_W, y_look),
           cv::Scalar(0, 255, 255), 1);

  for (int i = 0; i < N_WINDOWS; ++i) {
    int y0 = 0;
    int y1 = 0;
    band_of(i, y0, y1);
    const bool seen = peak_left[i] >= 0 || peak_right[i] >= 0;
    cv::rectangle(
        out.vis, to_vis(0, y0), to_vis(WORK_W, y1),
        band_ok[i] ? cv::Scalar(70, 70, 70)
                   : (seen ? cv::Scalar(0, 160, 255) : cv::Scalar(0, 0, 255)),
        1);

    const int y_mid = (y0 + y1) / 2;
    if (peak_left[i] >= 0) {
      cv::circle(out.vis, to_vis(peak_left[i], y_mid), 4, cv::Scalar(0, 255, 0),
                 -1);
    }
    if (peak_right[i] >= 0) {
      cv::circle(out.vis, to_vis(peak_right[i], y_mid), 4,
                 cv::Scalar(255, 0, 0), -1);
    }
  }

  for (const auto &p : cpts) {
    cv::circle(out.vis, to_vis(p.x, p.y), 3, cv::Scalar(0, 255, 255), -1);
  }

  if (out.fit_ok) {
    std::vector<cv::Point> poly;
    poly.reserve(21);
    for (int k = 0; k <= 20; ++k) {
      const int y = cy0 + (cy1 - cy0) * k / 20;
      const double x = eval_fit(cf, y, cy0, cy1);
      if (std::isfinite(x)) {
        poly.push_back(to_vis(cvRound(x), y));
      }
    }
    if (poly.size() > 1) {
      cv::polylines(out.vis, poly, false, cv::Scalar(0, 255, 255), 2,
                    cv::LINE_AA);
    }
  }

  if (state != LaneState::LOST) {
    cv::circle(out.vis, to_vis(centre_look, y_look), 6, cv::Scalar(0, 255, 255),
               2);
  }

  cv::Scalar flag_color(0, 0, 255);
  const char *flag_text = holding ? "LOST (hold)" : "LOST";
  if (state == LaneState::TWO_LINES) {
    flag_color = cv::Scalar(0, 255, 0);
    flag_text = "2 LANES OK";
  } else if (state == LaneState::ONE_LINE) {
    flag_color = cv::Scalar(0, 200, 255);
    flag_text = use_left ? "1 LANE (L)" : "1 LANE (R)";
  }
  cv::putText(out.vis, flag_text, cv::Point(12, 34), cv::FONT_HERSHEY_SIMPLEX,
              1.0, flag_color, 2, cv::LINE_AA);

  const std::string detail =
      "dev=" + std::to_string(out.dev_px) + "px " +
      std::to_string(static_cast<int>(std::lround(out.dev_cm))) + "cm" + " w=" +
      std::to_string(static_cast<int>(std::lround(lane_w_m_ * 100.0f))) + "cm" +
      " v=" +
      std::to_string(static_cast<int>(std::lround(out.speed_scale * 100.0f))) +
      "%";

  auto dist_text = [&](int y) {
    const double z = ground_dist_m(y);
    return std::isfinite(z)
               ? std::to_string(static_cast<int>(std::lround(z * 100.0)))
               : std::string("inf");
  };
  const std::string range = "ROI y=" + std::to_string(top) + ".." +
                            std::to_string(bottom) + " = " + dist_text(bottom) +
                            ".." + dist_text(top) + " cm truoc cam" +
                            ", nhin truoc " + dist_text(y_look) + " cm";

  const std::string dbg =
      "ctr=" + std::to_string(static_cast<int>(std::lround(contrast))) +
      " thr=" + std::to_string(thr) +
      " dx=" + std::to_string(static_cast<int>(std::lround(out.path_dx_cm))) +
      "cm" + " k=" + std::to_string(out.curvature).substr(0, 5) +
      " pts=" + std::to_string(cpts.size());

  cv::putText(out.vis, detail, cv::Point(12, 64), cv::FONT_HERSHEY_SIMPLEX, 0.6,
              cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
  cv::putText(out.vis, range, cv::Point(12, 84), cv::FONT_HERSHEY_SIMPLEX, 0.45,
              cv::Scalar(255, 200, 0), 1, cv::LINE_AA);
  cv::putText(out.vis, dbg, cv::Point(12, 102), cv::FONT_HERSHEY_SIMPLEX, 0.45,
              cv::Scalar(255, 200, 0), 1, cv::LINE_AA);

  out.proc_ms = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - t_begin)
                    .count();

  return two_lanes;
}

// ============================================================================
// TIEN ICH HINH HOC
// ============================================================================

int CameraLane::lane_px(float meters, int dy) const {
  return static_cast<int>(std::lround(meters * static_cast<float>(dy) / k_));
}

double CameraLane::ground_dist_m(double y) const {
  // goc nhin xuong so voi phuong ngang cua tia qua hang y
  const double psi = pitch_ + std::atan((y - WORK_H / 2.0) / f_px_);
  return psi > 1e-3 ? h_ / std::tan(psi)
                    : std::numeric_limits<double>::infinity();
}

// Run cot cua 1 vach rong: be rong bang keo (theo phoi canh) + do loe do vach
// nghieng. Vach qua diem triet tieu (cx, horizon_y_) nghieng |x - cx| / dy px
// tren moi hang, qua cua so cao win_h hang no loe them slope * win_h px.
int CameraLane::run_limit_px(int x, int dy, int win_h) const {
  const int d = std::max(dy, 1);
  const double slope = std::abs(x - WORK_W / 2) / static_cast<double>(d);
  const double tape = TAPE_MAX_M * static_cast<double>(d) / k_;
  return static_cast<int>(std::lround(tape + slope * win_h)) + 4;
}

// ============================================================================
// OTSU CHI TREN VUNG MASK
// ============================================================================

int CameraLane::otsu_masked(const cv::Mat &gray, const cv::Mat &mask,
                            double &contrast) {
  int hist[256] = {0};
  for (int y = 0; y < gray.rows; ++y) {
    const uchar *g = gray.ptr<uchar>(y);
    const uchar *m = mask.ptr<uchar>(y);
    for (int x = 0; x < gray.cols; ++x) {
      if (m[x]) {
        ++hist[g[x]];
      }
    }
  }

  double total = 0.0;
  double sum = 0.0;
  for (int i = 0; i < 256; ++i) {
    total += hist[i];
    sum += static_cast<double>(i) * hist[i];
  }

  contrast = 0.0;
  if (total < 1.0) {
    return 128;
  }

  double w_b = 0.0;
  double sum_b = 0.0;
  double best = -1.0;
  int thr = 128;

  for (int t = 0; t < 256; ++t) {
    w_b += hist[t];
    if (w_b == 0.0) {
      continue;
    }
    const double w_f = total - w_b;
    if (w_f == 0.0) {
      break;
    }
    sum_b += static_cast<double>(t) * hist[t];
    const double m_b = sum_b / w_b;
    const double m_f = (sum - sum_b) / w_f;
    const double between = w_b * w_f * (m_b - m_f) * (m_b - m_f);
    if (between > best) {
      best = between;
      thr = t;
      contrast = m_f - m_b;
    }
  }

  return thr;
}

// ============================================================================
// TIM CAP 2 VANH
// ============================================================================

bool CameraLane::seed_pair_from(const cv::Mat &col_sum, int dy, int win_h,
                                int &seed_left, int &seed_right) const {
  if (dy <= 1) {
    return false;
  }

  std::vector<int> centers; // tam cac run du hep de la vach
  int start = 0;
  bool inside = false;

  auto close_run = [&](int end) {
    const int c = (start + end) / 2;
    if ((end - start + 1) <= run_limit_px(c, dy, win_h)) {
      centers.push_back(c);
    }
  };

  for (int x = 0; x < col_sum.cols; ++x) {
    const bool on = col_sum.at<int>(0, x) >= WINDOW_MIN_POINTS;
    if (on && !inside) {
      start = x;
      inside = true;
    } else if (!on && inside) {
      close_run(x - 1);
      inside = false;
    }
  }
  if (inside) {
    close_run(col_sum.cols - 1);
  }

  // Chon cap co khoang cach gan be rong lan da hoc nhat, thay vi lay 2 run
  // ngoai cung (de dinh bong, cot tru nam ngoai lan)
  const int min_gap = lane_px(MIN_LANE_GAP_M, dy);
  const int max_gap = lane_px(LANE_W_MAX_M, dy);
  const int want = lane_px(lane_w_m_, dy);

  int best_err = 1 << 30;
  bool found = false;

  for (size_t a = 0; a < centers.size(); ++a) {
    for (size_t b = a + 1; b < centers.size(); ++b) {
      const int gap = centers[b] - centers[a];
      if (gap < min_gap || gap > max_gap) {
        continue;
      }
      const int err = std::abs(gap - want);
      if (err < best_err) {
        best_err = err;
        seed_left = centers[a];
        seed_right = centers[b];
        found = true;
      }
    }
  }

  return found;
}

// ============================================================================
// TIM TAM CUA 1 VANH
// ============================================================================

bool CameraLane::peak_in(const cv::Mat &col_sum, int seed, int max_run,
                         int &peak) {
  const int lo = std::max(0, seed - WINDOW_MARGIN);
  const int hi = std::min(col_sum.cols - 1, seed + WINDOW_MARGIN);
  if (lo > hi) {
    return false;
  }

  int best_x = -1;
  int best_v = 0;
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

  // Vach nghieng lam dinh cot phang: lay giua doan cao tu nua dinh tro len,
  // khong lay cot dau tien dat max (lech ve mot phia)
  const int level = std::max(WINDOW_MIN_POINTS, (best_v + 1) / 2);
  int a = best_x;
  int b = best_x;
  while (a > 0 && col_sum.at<int>(0, a - 1) >= level) {
    --a;
  }
  while (b < col_sum.cols - 1 && col_sum.at<int>(0, b + 1) >= level) {
    ++b;
  }

  if ((b - a + 1) > max_run) {
    return false;
  }

  peak = (a + b) / 2;
  return true;
}

// ============================================================================
// FIT BAC 2 THEO MET: x = c0 + c1*t + c2*t^2
// ============================================================================

bool CameraLane::fit_quad(const std::vector<double> &t,
                          const std::vector<double> &x, double coef[3]) {
  coef[0] = coef[1] = coef[2] = 0.0;
  if (t.size() < 4 || t.size() != x.size()) {
    return false;
  }

  double s[5] = {0, 0, 0, 0, 0}; // sum t^0..t^4
  double b[3] = {0, 0, 0};
  for (size_t i = 0; i < t.size(); ++i) {
    double p = 1.0;
    for (int k = 0; k < 5; ++k) {
      s[k] += p;
      if (k < 3) {
        b[k] += p * x[i];
      }
      p *= t[i];
    }
  }

  cv::Mat A = (cv::Mat_<double>(3, 3) << s[0], s[1], s[2], s[1], s[2], s[3],
               s[2], s[3], s[4]);
  cv::Mat rhs = (cv::Mat_<double>(3, 1) << b[0], b[1], b[2]);
  cv::Mat sol;
  if (!cv::solve(A, rhs, sol, cv::DECOMP_SVD)) {
    return false;
  }
  for (int k = 0; k < 3; ++k) {
    coef[k] = sol.at<double>(k);
    if (!std::isfinite(coef[k])) {
      return false;
    }
  }
  return true;
}

// ============================================================================
// FIT DUONG BAC 2
// ============================================================================

// Giai x = a*t^2 + b*t + c bang binh phuong nho nhat.
// Toa do y duoc chuan hoa ve [0,1] truoc khi giai, nen he ma tran on dinh.
bool CameraLane::fit_poly2(const std::vector<cv::Point> &pts, double coef[3],
                           int &y0, int &y1) {
  coef[0] = coef[1] = coef[2] = 0.0;
  y0 = 0;
  y1 = 0;

  if (pts.size() < 3) {
    return false;
  }

  int lo = pts.front().y;
  int hi = pts.front().y;
  for (const auto &p : pts) {
    lo = std::min(lo, p.y);
    hi = std::max(hi, p.y);
  }

  // Cac diem sat nhau: duong cong khong xac dinh duoc
  if (hi - lo < 4) {
    return false;
  }

  y0 = lo;
  y1 = hi;
  const double inv = 1.0 / static_cast<double>(hi - lo);

  // Duoi 4 diem: chi fit duong thang, bac 2 qua 3 diem dao dong manh
  if (pts.size() < 4) {
    double st = 0.0, sxv = 0.0, stt = 0.0, stx = 0.0;
    const double n = static_cast<double>(pts.size());
    for (const auto &p : pts) {
      const double t = static_cast<double>(p.y - lo) * inv;
      st += t;
      sxv += p.x;
      stt += t * t;
      stx += t * p.x;
    }
    const double den = n * stt - st * st;
    if (std::fabs(den) < 1e-9) {
      return false;
    }
    coef[1] = (n * stx - st * sxv) / den;
    coef[2] = (sxv - coef[1] * st) / n;
    return std::isfinite(coef[1]) && std::isfinite(coef[2]);
  }

  cv::Mat A = cv::Mat::zeros(3, 3, CV_64F);
  cv::Mat b = cv::Mat::zeros(3, 1, CV_64F);
  double *Ad = A.ptr<double>();
  double *bd = b.ptr<double>();

  for (const auto &p : pts) {
    const double t = static_cast<double>(p.y - lo) * inv;
    const double t2 = t * t;
    const double x = p.x;

    Ad[0] += t2 * t2; // sum t^4
    Ad[1] += t2 * t;  // sum t^3
    Ad[2] += t2;      // sum t^2
    Ad[4] += t2;      // sum t^2
    Ad[5] += t;       // sum t
    Ad[8] += 1.0;     // sum 1

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

// Camera cao h, cui xuong pitch, chan troi o hang horizon_y_, mat dat phang.
// Mot doan rong w_px o hang y tuong ung W = w_px * h / (cos(pitch) * dy) met
// tren duong (pitch = 0 thi con W = w_px * h / dy). Cho cung ket qua nhu IPM
// ma khong can bien doi phoi canh.
float CameraLane::px_to_cm(float px, int y) const {
  const int dy = y - horizon_y_;
  if (dy <= 1) {
    return 0.0f; // gan chan troi: khong do duoc
  }
  return px * k_ * 100.0f / static_cast<float>(dy);
}