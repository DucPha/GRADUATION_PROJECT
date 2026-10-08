#include "camera_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>
#include <utility>

#include <csetjmp>
#include <jpeglib.h>

// ============================================================================
// GIAI MA MJPG BANG LIBJPEG-TURBO
// ----------------------------------------------------------------------------
// Ly do khong dung decoder cua OpenCV:
//  1. libjpeg giai ma thang o 1/2, 1/4 kich thuoc (bo bot he so DCT) -> nhanh
//     gap doi; detector chi can 320 cot.
//  2. Cam nay chen byte dem sau moi khung -> libjpeg in "Corrupt JPEG data:
//     N extraneous bytes" moi frame. Anh van dung, chi ngap log. Ham
//     output_message rong o day lam im canh bao nay.
// ============================================================================

namespace {

struct JpegError {
  jpeg_error_mgr pub;
  std::jmp_buf jump;
};

void jpeg_on_error(j_common_ptr cinfo) {
  std::longjmp(reinterpret_cast<JpegError *>(cinfo->err)->jump, 1);
}

void jpeg_silent(j_common_ptr) {}

// Giai ma goi MJPG `buf` ra BGR. Thu nho 2^k lan sao cho be ngang van
// >= min_width. full_w/full_h = kich thuoc goc ghi trong header JPEG.
bool decode_mjpg(const cv::Mat &buf, int min_width, cv::Mat &out, int &full_w,
                 int &full_h) {
  if (buf.empty()) {
    return false;
  }

  jpeg_decompress_struct cinfo;
  JpegError err;
  cinfo.err = jpeg_std_error(&err.pub);
  err.pub.error_exit = jpeg_on_error;
  err.pub.output_message = jpeg_silent;

  if (setjmp(err.jump)) {
    // Khung hong (cap USB nhieu, mat goi): bo khung nay
    jpeg_destroy_decompress(&cinfo);
    return false;
  }

  jpeg_create_decompress(&cinfo);
  jpeg_mem_src(&cinfo, buf.ptr<unsigned char>(),
               static_cast<unsigned long>(buf.total() * buf.elemSize()));
  if (jpeg_read_header(&cinfo, TRUE) != JPEG_HEADER_OK) {
    jpeg_destroy_decompress(&cinfo);
    return false;
  }

  full_w = static_cast<int>(cinfo.image_width);
  full_h = static_cast<int>(cinfo.image_height);

  unsigned int denom = 1;
  while (denom < 8 &&
         static_cast<int>(cinfo.image_width / (denom * 2)) >= min_width) {
    denom *= 2;
  }
  cinfo.scale_num = 1;
  cinfo.scale_denom = denom;
  cinfo.out_color_space = JCS_EXT_BGR; // ra thang thu tu kenh cua OpenCV
  cinfo.dct_method = JDCT_ISLOW;

  jpeg_start_decompress(&cinfo);
  out.create(static_cast<int>(cinfo.output_height),
             static_cast<int>(cinfo.output_width), CV_8UC3);
  while (cinfo.output_scanline < cinfo.output_height) {
    JSAMPROW row = out.ptr<unsigned char>(static_cast<int>(cinfo.output_scanline));
    jpeg_read_scanlines(&cinfo, &row, 1);
  }
  jpeg_finish_decompress(&cinfo);
  jpeg_destroy_decompress(&cinfo);
  return true;
}

// ---- Vector 2D tren mat dat: x = X (phai), y = Z (truoc) ----
inline float norm2(const cv::Point2f &v) { return std::sqrt(v.x * v.x + v.y * v.y); }

// > 0: b nam ben TRAI a (nhin theo huong a); < 0: ben PHAI
inline float cross2(const cv::Point2f &a, const cv::Point2f &b) {
  return a.x * b.y - a.y * b.x;
}

inline cv::Point2f unit(const cv::Point2f &v) {
  const float n = norm2(v);
  return n > 1e-6f ? v * (1.0f / n) : cv::Point2f(0.0f, 1.0f);
}

// 0 khi v <= lo, 1 khi v >= hi
inline float ramp(float v, float lo, float hi) {
  return std::clamp((v - lo) / (hi - lo), 0.0f, 1.0f);
}

float polyline_len(const std::vector<cv::Point2f> &g) {
  float s = 0.0f;
  for (size_t i = 1; i < g.size(); ++i) {
    s += norm2(g[i] - g[i - 1]);
  }
  return s;
}

// Goc huong (rad) so voi phia truoc xe: > 0 = huong sang phai
inline float heading_of(const cv::Point2f &t) { return std::atan2(t.x, t.y); }

} // namespace

// ============================================================================
// CONSTRUCTOR / DESTRUCTOR
// ============================================================================

CameraLane::CameraLane(int camera_index, int target_fps, int width, int height,
                       const CameraProfile &profile)
    : camera_index_(camera_index),
      target_fps_(target_fps > 0 ? target_fps : 30),
      req_w_(width > 0 ? width : 1920), req_h_(height > 0 ? height : 1080),
      profile_(profile),
      roi_top_frac_(std::clamp(profile.roi_top_frac, 0.0f, 0.95f)) {
  h_ = std::max(0.05f, profile_.height_m);
  roi_bottom_frac_ = std::clamp(profile_.roi_bottom_frac, 0.1f, 1.0f);

  // Tinh truoc theo do phan giai yeu cau; khi doc duoc frame that se tinh
  // lai neu camera tra kich thuoc khac
  update_geometry(req_w_, req_h_);
}

CameraLane::~CameraLane() { stop(); }

// ============================================================================
// HINH HOC THEO KICH THUOC ANH
// ============================================================================

void CameraLane::update_geometry(int frame_w, int frame_h) {
  if (frame_w <= 0 || frame_h <= 0 ||
      (frame_w == geom_w_ && frame_h == geom_h_)) {
    return;
  }
  geom_w_ = frame_w;
  geom_h_ = frame_h;

  // Giu dung ti le anh goc: 16:9 -> 320x180, 4:3 -> 320x240. Nho vay 1 px
  // ngang = 1 px doc tren mat cam va cong thuc pinhole van dung.
  work_h_ = std::clamp(
      static_cast<int>(std::lround(static_cast<double>(WORK_W) * frame_h /
                                   frame_w)),
      2 * ROI_MIN_ROWS + 20, 4 * WORK_W);

  const double hfov =
      std::clamp(static_cast<double>(profile_.hfov_deg), 20.0, 170.0);
  f_px_ = (WORK_W / 2.0) / std::tan(hfov * CV_PI / 360.0);

  // Chan troi = anh cua huong nam ngang. Cam cui xuong pitch thi chan troi
  // nam tren tam anh f * tan(pitch) hang. Biet 1 trong 2 thi suy ra cai kia.
  if (profile_.horizon_frac >= 0.0f || profile_.horizon_y != -9999) {
    horizon_y_ = profile_.horizon_frac >= 0.0f
                     ? static_cast<int>(
                           std::lround(profile_.horizon_frac * work_h_))
                     : profile_.horizon_y;
    pitch_ = std::atan((work_h_ / 2.0 - horizon_y_) / f_px_);
  } else {
    pitch_ = profile_.axis_ground_m > 0.0f
                 ? std::atan2(static_cast<double>(h_),
                              static_cast<double>(profile_.axis_ground_m))
                 : 0.0;
    horizon_y_ = static_cast<int>(
        std::lround(work_h_ / 2.0 - f_px_ * std::tan(pitch_)));
  }

  // Doan rong w_px o hang y <-> X = w_px * h / (cos(pitch) * (y - horizon))
  k_ = h_ / static_cast<float>(std::cos(pitch_));

  // Bang tra theo hang: Z (m) va met / px ngang. Dung cho moi pixel vach.
  row_z_.assign(static_cast<size_t>(work_h_), -1.0f);
  row_s_.assign(static_cast<size_t>(work_h_), -1.0f);
  for (int y = 0; y < work_h_; ++y) {
    const int dy = y - horizon_y_;
    const double z = ground_dist_m(y);
    if (dy > 1 && std::isfinite(z) && z > 0.0) {
      row_z_[static_cast<size_t>(y)] = static_cast<float>(z);
      row_s_[static_cast<size_t>(y)] = k_ / static_cast<float>(dy);
    }
  }

  // Thang do dev_px: met / px anh 640 tai khoang cach DEV_REF_DIST_M
  {
    const double psi = std::atan2(static_cast<double>(h_), DEV_REF_DIST_M);
    const double y_ref = work_h_ / 2.0 + f_px_ * std::tan(psi - pitch_);
    const double dy = std::max(2.0, y_ref - horizon_y_);
    dev_m_per_px_ = (k_ / dy) * WORK_W / static_cast<double>(DEV_REF_W);
  }

  logged_geometry_ = false;
}

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

  // Thu mo ngay de bao loi som. Mo khong duoc (camera dang cam lai, bi app
  // khac giu...) thi luong camera VAN chay va tu thu mo lai moi
  // REOPEN_PERIOD_MS, khong bo cuoc nhu truoc.
  const bool opened = open_camera();

  running_.store(true);
  worker_ = std::thread(&CameraLane::capture_loop, this);
  return opened;
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

    // MJPG: YUYV khong ho tro 1080p va an het bang thong USB
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, req_w_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, req_h_);
    cap_.set(cv::CAP_PROP_FPS, target_fps_);

    // Hang doi V4L2 mac dinh ~4 frame = tre 130 ms o 30 fps
    cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);

    if (exposure_ > 0) {
      cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 1); // V4L2: 1 = phoi sang tay
      cap_.set(cv::CAP_PROP_EXPOSURE, exposure_);
    }

    // Xin goi MJPG nguyen (khong de OpenCV giai ma) -> tu giai ma 1/2
    cap_.set(cv::CAP_PROP_CONVERT_RGB, 0);

    // Doc thu mot frame de chan loi "mo ra nhung khong co hinh"
    cv::Mat probe_raw;
    cv::Mat probe;
    raw_mjpg_ = true;
    if (!cap_.read(probe_raw) || probe_raw.empty()) {
      cap_.release();
      continue;
    }
    // Driver tra anh da giai ma (nhieu hang, 3 kenh) -> khong phai goi nen
    raw_mjpg_ = probe_raw.rows == 1 && probe_raw.type() == CV_8UC1;
    if (raw_mjpg_ && !decode_mjpg(probe_raw, VIS_W, probe, cam_w_, cam_h_)) {
      // Khong giai ma duoc -> tra lai cho OpenCV tu giai ma
      raw_mjpg_ = false;
      cap_.set(cv::CAP_PROP_CONVERT_RGB, 1);
      if (!cap_.read(probe) || probe.empty()) {
        cap_.release();
        continue;
      }
    }
    if (!raw_mjpg_) {
      if (probe.empty()) {
        probe = probe_raw;
      }
      cam_w_ = probe.cols;
      cam_h_ = probe.rows;
    }

    // Doc lai fourcc driver that su dang dung: set() co the bi bo qua
    const int fcc = static_cast<int>(cap_.get(cv::CAP_PROP_FOURCC));
    const char fcc_name[5] = {static_cast<char>(fcc & 0xFF),
                              static_cast<char>((fcc >> 8) & 0xFF),
                              static_cast<char>((fcc >> 16) & 0xFF),
                              static_cast<char>((fcc >> 24) & 0xFF), 0};

    quiet_open_ = false;
    update_geometry(probe.cols, probe.rows);

    std::cout << "[CameraLane] Camera opened at index " << idx << ", real size "
              << cam_w_ << "x" << cam_h_ << ", format " << fcc_name
              << ", driver fps " << cap_.get(cv::CAP_PROP_FPS) << ", decode "
              << (raw_mjpg_ ? "libjpeg " : "opencv ") << probe.cols << "x"
              << probe.rows << ", work frame " << WORK_W << "x" << work_h_
              << ", exposure "
              << (exposure_ > 0 ? std::to_string(exposure_) : std::string("auto"))
              << "\n";

    if (std::string(fcc_name) != "MJPG") {
      std::cout << "[CameraLane] Camera did not accept MJPG (got " << fcc_name
                << "): fps may be limited by USB bandwidth.\n";
    }
    if (cam_w_ != req_w_ || cam_h_ != req_h_) {
      std::cout << "[CameraLane] Requested " << req_w_ << "x" << req_h_
                << " but camera gives " << cam_w_ << "x" << cam_h_
                << ". Geometry was recomputed for the real size.\n";
    }

    return true;
  }

  if (!quiet_open_) {
    std::cerr << "[CameraLane] No camera found, retrying every "
              << REOPEN_PERIOD_MS << " ms\n";
  }
  quiet_open_ = true; // chi bao 1 lan cho toi khi mo duoc lai
  cap_.release();
  return false;
}

// ============================================================================
// LUONG DOC FRAME
// ============================================================================

bool CameraLane::grab_frame(cv::Mat &raw, cv::Mat &bgr) {
  if (!cap_.read(raw) || raw.empty()) {
    return false;
  }
  if (!raw_mjpg_) {
    bgr = raw;
    return true;
  }
  return decode_mjpg(raw, VIS_W, bgr, cam_w_, cam_h_);
}

void CameraLane::capture_loop() {
  cv::Mat raw;
  cv::Mat frame;
  int fail_count = 0;
  auto last_open_try = std::chrono::steady_clock::now();

  while (running_.load()) {
    // ---- Chua mo duoc / vua mat camera: thu mo lai dinh ky ----
    // Trong luc nay khong co frame moi -> age_ms tang -> stale -> node dieu
    // khien tu gui EMG. Cam lai camera la xe co hinh tro lai, khong can
    // khoi dong lai node.
    if (!cap_.isOpened()) {
      const auto t = std::chrono::steady_clock::now();
      if (t - last_open_try >= std::chrono::milliseconds(REOPEN_PERIOD_MS)) {
        last_open_try = t;
        if (open_camera()) {
          std::cout << "[CameraLane] Camera reconnected\n";
          fail_count = 0;
        }
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      continue;
    }

    if (!grab_frame(raw, frame) || frame.empty()) {
      if (++fail_count >= MAX_READ_FAIL) {
        std::cerr << "[CameraLane] Camera stopped delivering frames, "
                     "reopening...\n";
        cap_.release();
        fail_count = 0;
        last_open_try = std::chrono::steady_clock::now();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    // Moc thoi gian la luc nhan frame, de age_ms gom ca thoi gian xu ly
    const auto t_grab = std::chrono::steady_clock::now();
    fail_count = 0;

    update_geometry(frame.cols, frame.rows);

    // Chi ve anh quan sat khi da co nguoi lay anh truoc do (viz_tick)
    const bool draw = vis_wanted_.exchange(false);

    LaneOutput out;
    out.frame_id = ++frame_id_;
    detect(frame, out, draw);

    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (draw) {
        latest_vis_.vis = std::move(out.vis);
        latest_vis_.raw = std::move(out.raw);
        latest_vis_.roi = std::move(out.roi);
        latest_vis_.bin = std::move(out.bin);
        latest_vis_.vis_frame_id = out.frame_id;
      }
      latest_ = std::move(out);
      last_frame_time_ = t_grab;
    }
  }

  running_.store(false);
}

bool CameraLane::process(const cv::Mat &bgr, LaneOutput &out, bool draw) {
  if (bgr.empty()) {
    return false;
  }
  if (cam_w_ == 0) {
    cam_w_ = bgr.cols;
    cam_h_ = bgr.rows;
  }
  update_geometry(bgr.cols, bgr.rows);
  out = LaneOutput{};
  out.frame_id = ++frame_id_;
  return detect(bgr, out, draw);
}

void CameraLane::get_latest(LaneOutput &out, bool copy_vis) const {
  std::lock_guard<std::mutex> lock(mtx_);

  if (copy_vis) {
    // cv::Mat chi copy header (dem tham chieu). Luong camera luon gan Mat
    // MOI cho latest_vis_ chu khong ghi de tai cho, nen chia se an toan.
    out.vis = latest_vis_.vis;
    out.raw = latest_vis_.raw;
    out.roi = latest_vis_.roi;
    out.bin = latest_vis_.bin;
    out.vis_frame_id = latest_vis_.vis_frame_id;
    vis_wanted_.store(true);
  } else {
    out.vis.release();
    out.raw.release();
    out.roi.release();
    out.bin.release();
  }

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
  out.centre_g = latest_.centre_g;
  out.left_g = latest_.left_g;
  out.right_g = latest_.right_g;
  out.stamp = latest_.stamp;
  out.gated = latest_.gated;
  out.horizon_frac = latest_.horizon_frac;
  out.frame_w = latest_.frame_w;
  out.frame_h = latest_.frame_h;
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

bool CameraLane::detect(const cv::Mat &frame, LaneOutput &out, bool draw) {
  const auto t_begin = std::chrono::steady_clock::now();

  const int cx = WORK_W / 2;
  const int WH = work_h_;
  out.frame_w = cam_w_ > 0 ? cam_w_ : frame.cols;
  out.frame_h = cam_h_ > 0 ? cam_h_ : frame.rows;
  out.horizon_frac = static_cast<float>(horizon_y_) / WH;
  const double sy = frame.rows / static_cast<double>(WH);

  // ---- 1. ROI hinh chu nhat, PHU HET BE NGANG ----------------------------
  const int bottom = std::clamp(
      static_cast<int>(std::lround(roi_bottom_frac_ * WH)), ROI_MIN_ROWS + 1,
      WH);
  int top = static_cast<int>(std::lround(roi_top_frac_.load() * WH));
  top = std::max(top, horizon_y_ + ROI_MIN_DY);
  top = std::clamp(top, 0, bottom - ROI_MIN_ROWS);
  const int roi_h = bottom - top;

  // Diem tham chieu xe: giua xe, ngang hang day ROI
  {
    const float zb = row_z_[static_cast<size_t>(bottom - 1)];
    car_ref_ = cv::Point2f(0.0f, zb > 0.0f ? zb : 0.4f);
  }

  if (!logged_geometry_) {
    logged_geometry_ = true;
    std::cout << "[CameraLane] geometry: camera " << cam_w_ << "x" << cam_h_
              << ", decoded " << frame.cols << "x" << frame.rows << " -> work "
              << WORK_W << "x" << WH << ", h=" << h_
              << " m, pitch=" << pitch_ * 180.0 / CV_PI << " deg, f=" << f_px_
              << " px, horizon_y=" << horizon_y_ << "; ROI rows " << top
              << ".." << bottom << " (full width) = " << ground_dist_m(bottom)
              << ".." << ground_dist_m(top) << " m, dev scale "
              << dev_m_per_px_ * 1000.0 << " mm/px @ " << DEV_REF_DIST_M
              << " m\n";
  }

  // ---- 2. Cat ROI (them le tren/duoi cho buoc uoc luong nen), xam, blur --
  // Le BG_KERNEL_H/2 hang: kernel dong o mep ROI van co du lieu THAT ben
  // ngoai. Thieu le, mep ROI gap vung sang dan cua den tran sinh ra dai vach
  // gia nam ngang dinh vao vach that.
  const int pad = BG_KERNEL_H / 2 + 1;
  const int ext_top = std::max(0, top - pad);
  const int ext_bot = std::min(WH, bottom + pad);
  const int fy0 = std::clamp(static_cast<int>(std::lround(ext_top * sy)), 0,
                             frame.rows - 1);
  const int fy1 = std::clamp(static_cast<int>(std::lround(ext_bot * sy)),
                             fy0 + 1, frame.rows);

  cv::Mat gray;
  cv::Mat colored; // 1 = pixel co MAU ro (ghe cam, vat mau...) -> khong phai vach
  {
    // Thu nho anh mau truoc (it pixel hon 9 lan cho cac buoc sau)
    cv::Mat small_bgr;
    cv::resize(frame.rowRange(fy0, fy1), small_bgr,
               cv::Size(WORK_W, ext_bot - ext_top), 0, 0, cv::INTER_AREA);
    cv::cvtColor(small_bgr, gray, cv::COLOR_BGR2GRAY);
    // Do bao hoa mau S = (max - min) / max. Vach den / bang keo den: S thap.
    colored = cv::Mat::zeros(small_bgr.size(), CV_8U);
    for (int y = 0; y < small_bgr.rows; ++y) {
      const cv::Vec3b *p = small_bgr.ptr<cv::Vec3b>(y);
      uchar *c = colored.ptr<uchar>(y);
      for (int x = 0; x < small_bgr.cols; ++x) {
        const int mx = std::max({p[x][0], p[x][1], p[x][2]});
        const int mn = std::min({p[x][0], p[x][1], p[x][2]});
        c[x] = mx >= COLOR_MIN_V && (mx - mn) * 255 > COLOR_MAX_SAT * mx;
      }
    }
    static const cv::Mat k_col =
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::dilate(colored, colored, k_col);
  }
  cv::GaussianBlur(gray, gray, cv::Size(BLUR_KSIZE, BLUR_KSIZE), 0);

  // ---- 3. Mask nhi phan (tru nen + bu loa + hysteresis) -----------------
  cv::Mat bin;
  double contrast = 0.0;
  build_mask(gray, top - ext_top, roi_h, top, bin, contrast);
  // Bo pixel co mau ro (ghe, vat mau dat sat vach): vach lan la den/xam
  bin.setTo(0, colored.rowRange(top - ext_top, top - ext_top + roi_h));
  if (THICK_REMOVE) {
    remove_thick(bin, top);
  }

  // ---- 4-5. Doan vach tren mat dat -> noi thanh vach --------------------
  std::vector<LinePath> segs;
  extract_segments(bin, top, segs);
  std::vector<LinePath> lines;
  link_segments(segs, lines);

  // ---- 6. Nhan trai/phai, chon cap gan xe nhat --------------------------
  const bool prior_ok =
      prior_time_ != std::chrono::steady_clock::time_point{} &&
      std::chrono::duration_cast<std::chrono::milliseconds>(t_begin -
                                                            prior_time_)
              .count() <= PRIOR_MAX_AGE_MS;

  int best_l = -1;
  int best_r = -1;
  float score_l = std::numeric_limits<float>::max();
  float score_r = std::numeric_limits<float>::max();
  for (auto &ln : lines) {
    ln.side = classify_side(ln, prior_ok);
  }
  // Vach mo ho (xe de len vach): neu co vach khac ro ben cach no dung 1 be
  // rong lan thi no la vach doi dien; khong co thi xet theo X trung binh.
  for (auto &ln : lines) {
    if (ln.side != 0) {
      continue;
    }
    float mx = 0.0f;
    for (const auto &p : ln.g) {
      mx += p.x;
    }
    mx /= static_cast<float>(ln.g.size());
    ln.side = mx < 0.0f ? -1 : 1;
    for (const auto &other : lines) {
      if (&other == &ln || other.side == 0) {
        continue;
      }
      const float w = median_dist(ln.g, other.g, false);
      if (w >= LANE_W_MIN_M && w <= LANE_W_MAX_M) {
        ln.side = -other.side;
        break;
      }
    }
  }
  for (size_t i = 0; i < lines.size(); ++i) {
    LinePath &ln = lines[i];
    // Xe nam TRONG lan -> vach cua lan minh cach xe khong qua 1 lan
    const float d = dist_to_polyline(car_ref_, ln.g);
    if (d > LANE_W_MAX_M) {
      continue;
    }
    // Uu tien vach gan xe; vach khop frame truoc duoc cong diem (khong bi
    // vet ban / vach lan ben canh gan hon hut mat)
    const float score = d - (ln.from_prior ? 0.15f : 0.0f) -
                        0.05f * std::min(ln.length_m, 1.0f);
    if (ln.side < 0 && score < score_l) {
      score_l = score;
      best_l = static_cast<int>(i);
    } else if (ln.side > 0 && score < score_r) {
      score_r = score;
      best_r = static_cast<int>(i);
    }
  }

  // Du 2 vach: kiem tra be rong lan (khoang cach vuong goc tren mat dat)
  float width_m = 0.0f;
  bool two_lanes = false;
  if (best_l >= 0 && best_r >= 0) {
    const LinePath &L = lines[static_cast<size_t>(best_l)];
    const LinePath &R = lines[static_cast<size_t>(best_r)];
    width_m = median_dist(R.g, L.g, true);
    if (!std::isfinite(width_m)) {
      width_m = median_dist(R.g, L.g, false);
    }
    two_lanes = width_m >= LANE_W_MIN_M && width_m <= LANE_W_MAX_M;
    if (two_lanes) {
      lane_w_m_ += LANE_W_ALPHA * (width_m - lane_w_m_);
      out.lane_width_cm = width_m * 100.0f;
    } else {
      // Cap vach vo ly: giu vach dang tin hon (khop frame truoc, roi dai hon)
      const auto rank = [](const LinePath &p) {
        return (p.from_prior ? 10.0f : 0.0f) + p.length_m;
      };
      if (rank(L) >= rank(R)) {
        best_r = -1;
      } else {
        best_l = -1;
      }
    }
  }

  const LinePath *left = best_l >= 0 ? &lines[static_cast<size_t>(best_l)] : nullptr;
  const LinePath *right = best_r >= 0 ? &lines[static_cast<size_t>(best_r)] : nullptr;

  LaneState state = LaneState::LOST;
  if (two_lanes) {
    state = LaneState::TWO_LINES;
  } else if (left || right) {
    state = LaneState::ONE_LINE;
  }

  if (left) {
    out.left_pts = left->img;
  }
  if (right) {
    out.right_pts = right->img;
  }

  // ---- 7. Duong tam lan: vach doi vao trong nua be rong lan --------------
  const float half_w = 0.5f * (two_lanes ? width_m : lane_w_m_);
  std::vector<cv::Point2f> c_left;
  std::vector<cv::Point2f> c_right;
  if (left) {
    offset_path(*left, half_w, c_left);
  }
  if (right) {
    offset_path(*right, half_w, c_right);
  }
  // Duong tam dai hon dung de tinh do cong / he so toc do
  const std::vector<cv::Point2f> &cpath =
      polyline_len(c_left) >= polyline_len(c_right) ? c_left : c_right;

  // ---- 8. Pure pursuit -> dev_px -----------------------------------------
  cv::Point2f aim(0.0f, PURSUIT_L_M);
  bool have_aim = false;
  float e_near = 0.0f;
  {
    cv::Point2f sum(0.0f, 0.0f);
    float sum_e = 0.0f;
    int n = 0;
    int ne = 0;
    for (const auto *path : {&c_left, &c_right}) {
      cv::Point2f p;
      if (!pursuit_point(*path, PURSUIT_L_M, p)) {
        continue;
      }
      sum += p;
      ++n;
      // Lech ngang cua tam lan ngay truoc xe, chi khi duong tam o do con
      // huong ve phia truoc (vach nam ngang o cua gat thi bo khau nay)
      const cv::Point2f t0 = tangent_at(*path, 0);
      if (std::fabs(heading_of(t0)) < 35.0f * static_cast<float>(CV_PI) / 180.0f) {
        sum_e += path->front().x;
        ++ne;
      }
    }
    if (n > 0) {
      aim = sum * (1.0f / static_cast<float>(n));
      have_aim = true;
      // Khoang cach thuc toi diem ngam (duong ngan hon PURSUIT_L_M thi gan hon)
      const float Lp = std::max(0.25f, norm2(aim));
      // Pure pursuit: do cong = 2 X / L^2. Quy ve do lech tuong duong tai
      // DEV_REF_DIST_M de giu nguyen thang do firmware.
      const float r = DEV_REF_DIST_M / Lp;
      float x_cmd = aim.x * r * r;
      if (ne > 0) {
        e_near = sum_e / static_cast<float>(ne);
        x_cmd += LAT_GAIN * e_near;
      }
      out.dev_cm = x_cmd * 100.0f;

      const float raw = std::clamp(
          static_cast<float>(x_cmd / dev_m_per_px_),
          -static_cast<float>(DEV_MAX_REF_PX), static_cast<float>(DEV_MAX_REF_PX));

      // Chan nhay + EMA
      const float alpha = two_lanes ? EMA_ALPHA_TWO : EMA_ALPHA_ONE;
      if (!ema_primed_) {
        dev_ema_ = raw;
        ema_primed_ = true;
        jump_count_ = 0;
      } else if (std::fabs(raw - dev_ema_) > JUMP_GATE_REF_PX &&
                 ++jump_count_ < JUMP_CONFIRM_FRAMES) {
        // Nhay dot ngot 1 frame: nhieu kha nang nhan nham -> giu gia tri cu
        out.gated = true;
      } else {
        jump_count_ = 0;
        dev_ema_ += alpha * (raw - dev_ema_);
      }
    }
  }
  if (!have_aim && state != LaneState::LOST) {
    state = LaneState::LOST; // co vach nhung khong dung duoc duong tam
    two_lanes = false;
  }
  out.fit_ok = cpath.size() >= 3;

  // ---- 8b. Xuat duong tam + vach toa do mat dat cho PathTracker ---------
  if (have_aim) {
    // 2 vach: trung binh 2 duong tam suy tu tung vach (phan chong len nhau)
    std::vector<cv::Point2f> centre = cpath;
    const std::vector<cv::Point2f> &other = &cpath == &c_left ? c_right : c_left;
    if (other.size() >= 2) {
      for (auto &p : centre) {
        bool in = false;
        const float d = dist_to_polyline(p, other, &in);
        if (in && d < 0.15f) {
          // diem gan nhat tren duong kia ~ p dich ve phia no 1 doan d
          float best = std::numeric_limits<float>::max();
          cv::Point2f q = p;
          for (size_t k = 0; k + 1 < other.size(); ++k) {
            const cv::Point2f ab = other[k + 1] - other[k];
            const float L2 = ab.dot(ab);
            const float t =
                L2 > 1e-9f ? std::clamp((p - other[k]).dot(ab) / L2, 0.0f, 1.0f)
                           : 0.0f;
            const cv::Point2f c = other[k] + ab * t;
            if (norm2(p - c) < best) {
              best = norm2(p - c);
              q = c;
            }
          }
          p = 0.5f * (p + q);
        }
      }
    }
    out.centre_g = resample(centre, 0.05f);
    if (left) {
      out.left_g = left->g;
    }
    if (right) {
      out.right_g = right->g;
    }
  }
  out.stamp = t_begin;

  // Do cong + lech ngang xa/gan cua duong tam. Huong 2 dau lay tren day cung
  // CURVE_CHORD_M (khong theo tiep tuyen tung diem -> khong bi nhieu dau xa).
  float heading_far = 0.0f;
  const float total = polyline_len(cpath);
  if (cpath.size() >= 2 && total >= 0.05f) {
    const float chord = std::min(CURVE_CHORD_M, 0.5f * total);
    // Diem cach dau duong (hoac cuoi duong) `d` met theo chieu dai cung
    auto along = [&](float d, bool from_end) {
      float acc = 0.0f;
      const size_t n = cpath.size();
      for (size_t k = 1; k < n; ++k) {
        const cv::Point2f &a = from_end ? cpath[n - k] : cpath[k - 1];
        const cv::Point2f &b = from_end ? cpath[n - k - 1] : cpath[k];
        const float l = norm2(b - a);
        if (acc + l >= d) {
          return a + (b - a) * ((d - acc) / std::max(l, 1e-6f));
        }
        acc += l;
      }
      return from_end ? cpath.front() : cpath.back();
    };
    const cv::Point2f tn = unit(along(chord, false) - cpath.front());
    const cv::Point2f tf = unit(cpath.back() - along(chord, true));
    heading_far = heading_of(tf);
    float dth = heading_far - heading_of(tn);
    if (dth > static_cast<float>(CV_PI)) {
      dth -= 2.0f * static_cast<float>(CV_PI);
    } else if (dth < -static_cast<float>(CV_PI)) {
      dth += 2.0f * static_cast<float>(CV_PI);
    }
    // 2 day cung cach nhau (total - chord) theo chieu dai cung
    out.curvature = dth / std::max(0.15f, total - chord);
    out.path_dx_cm = 100.0f * (cpath.back().x - cpath.front().x);
  }

  // ---- 9. Giu gia tri khi mat lan, he so toc do --------------------------
  if (state != LaneState::LOST) {
    last_valid_time_ = t_begin;
  }
  const auto lost_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                           t_begin - last_valid_time_)
                           .count();
  const bool holding = state == LaneState::LOST && lost_ms <= HOLD_MS;

  if (state == LaneState::LOST && !holding) {
    ema_primed_ = false;
    jump_count_ = 0;
  }

  out.two_lanes = two_lanes;
  out.state = state;
  out.dev_px = (state != LaneState::LOST || holding)
                   ? static_cast<int>(std::lround(dev_ema_))
                   : 0;

  if (state == LaneState::LOST) {
    out.speed_scale = 0.0f;
  } else {
    const float deg = std::fabs(heading_far) * 180.0f / static_cast<float>(CV_PI);
    const float curve =
        std::max(ramp(deg, CURVE_HEADING_START_DEG, CURVE_HEADING_FULL_DEG),
                 ramp(std::fabs(out.curvature), CURVE_K_START, CURVE_K_FULL));
    out.speed_scale = 1.0f - curve;
  }

  // ---- 10. Luu vach lam goi y cho frame sau ------------------------------
  if (state != LaneState::LOST) {
    if (left) {
      prior_left_ = left->g;
    } else {
      prior_left_.clear();
    }
    if (right) {
      prior_right_ = right->g;
    } else {
      prior_right_.clear();
    }
    prior_time_ = t_begin;
  }

  out.proc_ms = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - t_begin)
                    .count();

  if (!draw) {
    return two_lanes;
  }

  // ---- 11. Anh quan sat cho GUI (chi khi co nguoi xem) -------------------
  const int vis_h = std::max(
      1, static_cast<int>(std::lround(static_cast<double>(VIS_W) * frame.rows /
                                      frame.cols)));
  cv::Mat small;
  cv::resize(frame, small, cv::Size(VIS_W, vis_h), 0, 0, cv::INTER_AREA);

  const double vx = static_cast<double>(VIS_W) / WORK_W;
  const double vy = static_cast<double>(vis_h) / WH;

  out.raw = small.clone();
  {
    const int ry0 = std::clamp(static_cast<int>(std::lround(top * vy)), 0,
                               vis_h - 1);
    const int ry1 = std::clamp(static_cast<int>(std::lround(bottom * vy)),
                               ry0 + 1, vis_h);
    out.roi = small.rowRange(ry0, ry1).clone();
  }
  out.bin = bin; // bin la Mat cuc bo, khong ai sua nua -> khong can clone

  cv::Mat &vis = small;
  auto to_vis = [&](double x, double y) {
    return cv::Point(cvRound(x * vx), cvRound(y * vy));
  };
  auto ground_poly = [&](const std::vector<cv::Point2f> &g) {
    std::vector<cv::Point> p;
    p.reserve(g.size());
    for (const auto &q : g) {
      cv::Point2d ip;
      if (ground_to_img(q, ip) && ip.y > top - roi_h && ip.y < WH + roi_h) {
        p.push_back(to_vis(ip.x, ip.y));
      }
    }
    return p;
  };

  // Vung lan (to mau trong suot giua 2 vach)
  if (two_lanes && left->img.size() >= 2 && right->img.size() >= 2) {
    std::vector<cv::Point> area;
    for (const auto &p : left->img) {
      area.push_back(to_vis(p.x, p.y));
    }
    for (auto it = right->img.rbegin(); it != right->img.rend(); ++it) {
      area.push_back(to_vis(it->x, it->y));
    }
    cv::Mat layer = vis.clone();
    cv::fillPoly(layer, std::vector<std::vector<cv::Point>>{area},
                 cv::Scalar(80, 200, 80), cv::LINE_AA);
    cv::addWeighted(layer, 0.28, vis, 0.72, 0.0, vis);
  }

  // ROI chu nhat full be ngang
  cv::rectangle(vis, to_vis(0, top), to_vis(WORK_W, bottom) - cv::Point(1, 1),
                cv::Scalar(255, 190, 0), 1, cv::LINE_AA);

  // Chan troi (de do lai horizon_frac)
  if (horizon_y_ >= 0 && horizon_y_ < WH) {
    cv::line(vis, to_vis(0, horizon_y_), to_vis(WORK_W, horizon_y_),
             cv::Scalar(255, 0, 255), 1, cv::LINE_AA);
  }

  // Vach ung vien khong duoc chon: xam
  for (size_t i = 0; i < lines.size(); ++i) {
    if (static_cast<int>(i) == best_l || static_cast<int>(i) == best_r) {
      continue;
    }
    std::vector<cv::Point> p;
    for (const auto &q : lines[i].img) {
      p.push_back(to_vis(q.x, q.y));
    }
    cv::polylines(vis, p, false, cv::Scalar(150, 150, 150), 1, cv::LINE_AA);
  }

  // Vach da chon + "cua so" doc theo vach (bam moi huong)
  auto draw_line = [&](const LinePath *ln, const cv::Scalar &c) {
    if (!ln) {
      return;
    }
    std::vector<cv::Point> p;
    for (size_t k = 0; k < ln->img.size(); ++k) {
      const cv::Point &q = ln->img[k];
      p.push_back(to_vis(q.x, q.y));
      if (k % 2 == 0) {
        const float s = row_s_[static_cast<size_t>(std::clamp(q.y, 0, WH - 1))];
        const int half = std::clamp(
            s > 0.0f ? static_cast<int>(0.05f / s) : 6, 3, 16);
        cv::rectangle(vis, to_vis(q.x - half, q.y - half),
                      to_vis(q.x + half, q.y + half), cv::Scalar(120, 120, 120), 1);
      }
    }
    cv::polylines(vis, p, false, c, 2, cv::LINE_AA);
  };
  draw_line(left, cv::Scalar(0, 230, 0));
  draw_line(right, cv::Scalar(255, 120, 0));

  // Duong tam lan (vang) + diem ngam pure pursuit
  for (const auto *path : {&c_left, &c_right}) {
    const auto p = ground_poly(*path);
    if (p.size() > 1) {
      cv::polylines(vis, p, false, cv::Scalar(0, 230, 255), 2, cv::LINE_AA);
    }
  }
  if (have_aim) {
    cv::Point2d ip;
    if (ground_to_img(aim, ip)) {
      cv::circle(vis, to_vis(ip.x, ip.y), 7, cv::Scalar(0, 230, 255), 2,
                 cv::LINE_AA);
      cv::line(vis, to_vis(cx, bottom), to_vis(ip.x, ip.y),
               cv::Scalar(0, 230, 255), 1, cv::LINE_AA);
    }
  }
  cv::line(vis, to_vis(cx, bottom - 6), to_vis(cx, bottom),
           cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

  // Thanh thong tin tren cung (nen toi trong suot)
  {
    const int bar_h = 26;
    cv::Mat bar = vis.rowRange(0, std::min(bar_h, vis.rows));
    bar.convertTo(bar, -1, 0.35, 0.0);

    cv::Scalar flag_color(60, 60, 255);
    std::string flag_text = holding ? "LOST (hold)" : "LOST";
    if (state == LaneState::TWO_LINES) {
      flag_color = cv::Scalar(90, 230, 90);
      flag_text = "2 LANES";
    } else if (state == LaneState::ONE_LINE) {
      flag_color = cv::Scalar(0, 200, 255);
      flag_text = left ? "1 LANE (L)" : "1 LANE (R)";
    }
    if (out.gated) {
      flag_text += " *";
    }
    cv::putText(vis, flag_text, cv::Point(8, 18), cv::FONT_HERSHEY_SIMPLEX,
                0.55, flag_color, 1, cv::LINE_AA);

    char info[200];
    std::snprintf(info, sizeof(info),
                  "dev %+dpx %+.0fcm  w %.0fcm  v %.0f%%  k %+.2f  ctr %.0f  "
                  "lines %zu",
                  out.dev_px, out.dev_cm, lane_w_m_ * 100.0f,
                  out.speed_scale * 100.0f, out.curvature, contrast,
                  lines.size());
    cv::putText(vis, info, cv::Point(118, 18), cv::FONT_HERSHEY_SIMPLEX, 0.40,
                cv::Scalar(235, 235, 235), 1, cv::LINE_AA);
  }

  out.vis = vis;
  return two_lanes;
}

// ============================================================================
// MASK NHI PHAN: TRU NEN + BU LOA + HYSTERESIS
// ----------------------------------------------------------------------------
// Den tran chieu xuong lam 2 viec: (1) "trang trang" ca vach lan san trong
// vung loa -> vach van toi hon san NGAY CANH no nhung co the sang hon nguong
// chung cua ca anh; (2) camera nen tuong phan o vung sang (gamma, gan bao
// hoa) -> chenh lech vach/san trong vung loa chi con vai muc xam.
//
//   nen  = dong hinh thai hoc (gian roi co) bang kernel lon hon be day vach
//          theo MOI huong -> vach doc/cheo/ngang deu bi "lap" bang mau san,
//          con loa va do sang chung giu nguyen.
//   diff = (nen - anh) * gain(nen): gain > 1 o vung nen sang hon trung vi,
//          bu lai tuong phan bi camera nen mat trong vung loa.
//   Otsu tren diff -> nguong manh; giu them pixel yeu (> BG_WEAK_DIFF) neu
//   NOI LIEN voi pixel manh (nguong tre): doan vach bi loa duoc noi lai voi
//   phan vach con ro, van san roi rac thi bi bo.
// ============================================================================

void CameraLane::build_mask(const cv::Mat &gray_ext, int row0, int rows,
                            int top, cv::Mat &bin, double &contrast) const {
  static const cv::Mat k_bg = cv::getStructuringElement(
      cv::MORPH_RECT, cv::Size(BG_KERNEL_W, BG_KERNEL_H));
  cv::Mat bg_ext;
  cv::morphologyEx(gray_ext, bg_ext, cv::MORPH_CLOSE, k_bg, cv::Point(-1, -1),
                   1, cv::BORDER_REPLICATE);
  // Tu day chi xet dung vung ROI
  const cv::Mat gray = gray_ext.rowRange(row0, row0 + rows);
  const cv::Mat bg = bg_ext.rowRange(row0, row0 + rows);

  // Do sang nen trung vi -> bang he so bu loa
  int hist_bg[256] = {0};
  for (int y = 0; y < bg.rows; ++y) {
    const uchar *b = bg.ptr<uchar>(y);
    for (int x = 0; x < bg.cols; ++x) {
      ++hist_bg[b[x]];
    }
  }
  const int half = bg.rows * bg.cols / 2;
  int med = 0;
  for (int acc = 0; med < 255; ++med) {
    acc += hist_bg[med];
    if (acc >= half) {
      break;
    }
  }
  float gain[256];
  for (int b = 0; b < 256; ++b) {
    gain[b] = std::clamp(static_cast<float>(255 - med) /
                             static_cast<float>(std::max(255 - b, 8)),
                         1.0f, GLARE_GAIN_MAX);
  }

  cv::Mat diff(gray.size(), CV_8U);
  int hist[256] = {0};
  for (int y = 0; y < gray.rows; ++y) {
    const uchar *g = gray.ptr<uchar>(y);
    const uchar *b = bg.ptr<uchar>(y);
    uchar *d = diff.ptr<uchar>(y);
    for (int x = 0; x < gray.cols; ++x) {
      int v = static_cast<int>(b[x]) - static_cast<int>(g[x]);
      if (v >= 3) {
        // Khong khuech dai nhieu nho (< 3 muc xam)
        v = std::min(255, static_cast<int>(v * gain[b[x]] + 0.5f));
      } else if (v < 0) {
        v = 0;
      }
      d[x] = static_cast<uchar>(v);
      ++hist[v];
    }
  }

  bin = cv::Mat::zeros(gray.size(), CV_8U);
  const int thr = std::max(otsu_hist(hist, contrast), BG_MIN_DIFF);
  if (contrast < MIN_CONTRAST) {
    return;
  }

  cv::Mat weak;
  cv::threshold(diff, weak, BG_WEAK_DIFF, 1, cv::THRESH_BINARY);
  cv::Mat labels;
  const int n_lab = cv::connectedComponents(weak, labels, 8, CV_32S);
  std::vector<uchar> keep(static_cast<size_t>(n_lab), 0); // nhan 0 = nen
  for (int y = 0; y < diff.rows; ++y) {
    const uchar *d = diff.ptr<uchar>(y);
    const int *l = labels.ptr<int>(y);
    for (int x = 0; x < diff.cols; ++x) {
      if (l[x] > 0 && d[x] > thr) {
        keep[static_cast<size_t>(l[x])] = 1; // thanh phan co pixel manh
      }
    }
  }
  for (int y = 0; y < bin.rows; ++y) {
    uchar *b = bin.ptr<uchar>(y);
    const int *l = labels.ptr<int>(y);
    for (int x = 0; x < bin.cols; ++x) {
      b[x] = keep[static_cast<size_t>(l[x])];
    }
  }

  // Lap lo nho trong vach, doi xung moi huong (vach ngang cung duoc lap)
  static const cv::Mat k_close =
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
  cv::morphologyEx(bin, bin, cv::MORPH_CLOSE, k_close);
}

// ============================================================================
// KHOET KHOI TOI DAY DINH VAO VACH
// ----------------------------------------------------------------------------
// Ghe, chan ban, vat can, bong do... toi nhu vach va thuong NAM SAT vach ->
// cung 1 thanh phan lien thong -> loc be day loai ca vach. Vach that khong
// day qua TAPE_MAX_M, nen diem nao cach mep mask xa hon nua be day do (doi
// ra px theo tung hang) la LOI cua 1 khoi day. Khoi day = loi gian ra dung ban
// kinh do; bo khoi day khoi mask, phan vach mong dinh vao no van con.
// Ban kinh doi theo phoi canh -> chia ROI thanh THICK_BANDS dai ngang.
// ============================================================================

void CameraLane::remove_thick(cv::Mat &bin, int top) const {
  if (cv::countNonZero(bin) == 0) {
    return;
  }
  // Do be day tren ban sao da lap lo nho: khoi ghe / bong lam tam lo (van be
  // mat) thi khoang cach toi "mep" luc nao cung nho -> khong thay loi day
  cv::Mat solid;
  static const cv::Mat k_fill =
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9));
  cv::morphologyEx(bin, solid, cv::MORPH_CLOSE, k_fill);
  cv::Mat dist;
  cv::distanceTransform(solid, dist, cv::DIST_L2, 3);

  cv::Mat thick = cv::Mat::zeros(bin.size(), CV_8U);
  const int band_h = std::max(1, (bin.rows + THICK_BANDS - 1) / THICK_BANDS);
  for (int y0 = 0; y0 < bin.rows; y0 += band_h) {
    const int y1 = std::min(bin.rows, y0 + band_h);
    // Ban kinh lon nhat cua vach o dai nay: lay hang GAN XE nhat (vach to nhat)
    const int yy = std::clamp(top + y1 - 1, 0, work_h_ - 1);
    const float s = row_s_[static_cast<size_t>(yy)];
    if (s <= 0.0f) {
      continue;
    }
    const float r = 0.5f * TAPE_MAX_M / s;
    cv::Mat seeds = cv::Mat::zeros(bin.size(), CV_8U);
    cv::Mat band_seeds;
    cv::threshold(dist.rowRange(y0, y1), band_seeds, 1.15 * r + 1.0, 1,
                  cv::THRESH_BINARY);
    band_seeds.convertTo(seeds.rowRange(y0, y1), CV_8U);
    if (cv::countNonZero(seeds) == 0) {
      continue;
    }
    const int k = 2 * static_cast<int>(std::ceil(1.15f * r + 2.0f)) + 1;
    cv::dilate(seeds, seeds,
               cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k)));
    thick |= seeds;
  }
  bin.setTo(0, thick);
}

int CameraLane::otsu_hist(const int hist[256], double &contrast) {
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
// THANH PHAN LIEN THONG -> DOAN VACH TREN MAT DAT
// ----------------------------------------------------------------------------
// Moi thanh phan: doi tung pixel sang (X, Z) met, PCA tim truc chinh, chia
// doan BIN_M doc truc -> diem tam vach tung doan + be day vuong goc truc.
// Khong gia dinh vach chay doc anh nhu cua so truot theo hang: vach nam
// ngang / cheo man hinh o cua gat duoc bam y nhu vach thang.
// ============================================================================

void CameraLane::extract_segments(const cv::Mat &bin, int top,
                                  std::vector<LinePath> &segs) const {
  segs.clear();

  cv::Mat labels;
  cv::Mat stats;
  cv::Mat cents;
  const int n = cv::connectedComponentsWithStats(bin, labels, stats, cents, 8,
                                                 CV_32S);
  if (n <= 1) {
    return;
  }

  std::vector<std::vector<cv::Point>> pix(static_cast<size_t>(n));
  std::vector<uchar> ok(static_cast<size_t>(n), 0);
  for (int l = 1; l < n; ++l) {
    const int area = stats.at<int>(l, cv::CC_STAT_AREA);
    if (area >= MIN_BLOB_PX) {
      ok[static_cast<size_t>(l)] = 1;
      pix[static_cast<size_t>(l)].reserve(static_cast<size_t>(area));
    }
  }
  for (int y = 0; y < bin.rows; ++y) {
    const int yy = y + top;
    if (yy < 0 || yy >= work_h_ || row_s_[static_cast<size_t>(yy)] <= 0.0f) {
      continue;
    }
    const int *lab = labels.ptr<int>(y);
    for (int x = 0; x < bin.cols; ++x) {
      if (ok[static_cast<size_t>(lab[x])]) {
        pix[static_cast<size_t>(lab[x])].emplace_back(x, yy);
      }
    }
  }

  const float cx = WORK_W / 2.0f;
  std::vector<cv::Point2f> gp;
  std::vector<float> thick;

  struct Bin {
    double gx = 0, gz = 0, ix = 0, iy = 0;
    int n = 0;
    float bmin = std::numeric_limits<float>::max();
    float bmax = -std::numeric_limits<float>::max();
  };
  std::vector<Bin> bins;

  for (int l = 1; l < n; ++l) {
    const auto &P = pix[static_cast<size_t>(l)];
    if (static_cast<int>(P.size()) < MIN_BLOB_PX) {
      continue;
    }

    // Pixel -> mat dat, trung binh + hiep phuong sai
    gp.resize(P.size());
    double mx = 0.0, mz = 0.0, my_img = 0.0;
    for (size_t i = 0; i < P.size(); ++i) {
      const size_t r = static_cast<size_t>(P[i].y);
      gp[i] = cv::Point2f((P[i].x - cx) * row_s_[r], row_z_[r]);
      mx += gp[i].x;
      mz += gp[i].y;
      my_img += P[i].y;
    }
    const double inv = 1.0 / static_cast<double>(P.size());
    mx *= inv;
    mz *= inv;
    my_img *= inv;
    double sxx = 0.0, szz = 0.0, sxz = 0.0;
    for (const auto &q : gp) {
      const double dx = q.x - mx;
      const double dz = q.y - mz;
      sxx += dx * dx;
      szz += dz * dz;
      sxz += dx * dz;
    }
    const double th = 0.5 * std::atan2(2.0 * sxz, sxx - szz);
    const cv::Point2f u(static_cast<float>(std::cos(th)),
                        static_cast<float>(std::sin(th)));
    const cv::Point2f nr(-u.y, u.x);
    const cv::Point2f m(static_cast<float>(mx), static_cast<float>(mz));

    float amin = std::numeric_limits<float>::max();
    float amax = -std::numeric_limits<float>::max();
    for (const auto &q : gp) {
      const float a = (q - m).dot(u);
      amin = std::min(amin, a);
      amax = std::max(amax, a);
    }
    const float len = amax - amin;
    if (len < MIN_SEG_LEN_M) {
      continue;
    }

    // Chia doan doc truc chinh
    const int nb = std::min(400, static_cast<int>(len / BIN_M) + 1);
    bins.assign(static_cast<size_t>(nb), Bin{});
    for (size_t i = 0; i < gp.size(); ++i) {
      const cv::Point2f d = gp[i] - m;
      const int k = std::clamp(static_cast<int>((d.dot(u) - amin) / BIN_M), 0,
                               nb - 1);
      Bin &b = bins[static_cast<size_t>(k)];
      const float bn = d.dot(nr);
      b.gx += gp[i].x;
      b.gz += gp[i].y;
      b.ix += P[i].x;
      b.iy += P[i].y;
      ++b.n;
      b.bmin = std::min(b.bmin, bn);
      b.bmax = std::max(b.bmax, bn);
    }

    // Bo cac doan o 2 DAU vach chi co 1 phan mat cat (vach cheo bi mep ROI
    // cat ngang): tam cua chung lech sang 1 ben -> dau vach bi "moc cau".
    int k0 = 0;
    int k1 = nb - 1;
    {
      std::vector<int> cnt;
      for (const Bin &b : bins) {
        if (b.n >= 2) {
          cnt.push_back(b.n);
        }
      }
      if (cnt.size() >= 5) {
        std::nth_element(cnt.begin(), cnt.begin() + cnt.size() / 2, cnt.end());
        const int low = std::max(2, static_cast<int>(0.5 * cnt[cnt.size() / 2]));
        while (k0 < k1 && bins[static_cast<size_t>(k0)].n < low) {
          ++k0;
        }
        while (k1 > k0 && bins[static_cast<size_t>(k1)].n < low) {
          --k1;
        }
      }
    }

    LinePath seg;
    thick.clear();
    for (int kb = k0; kb <= k1; ++kb) {
      const Bin &b = bins[static_cast<size_t>(kb)];
      if (b.n < 2) {
        continue;
      }
      seg.g.emplace_back(static_cast<float>(b.gx / b.n),
                         static_cast<float>(b.gz / b.n));
      seg.img.emplace_back(static_cast<int>(std::lround(b.ix / b.n)),
                           static_cast<int>(std::lround(b.iy / b.n)));
      thick.push_back(b.bmax - b.bmin);
    }
    if (seg.g.size() < 2) {
      continue;
    }

    // Be day trung vi (1 cuc nhieu dinh vao vach khong lam hong ca vach),
    // cong sai so luong tu 2 pixel tai hang trung binh
    std::nth_element(thick.begin(), thick.begin() + thick.size() / 2,
                     thick.end());
    const float med_thick = thick[thick.size() / 2];
    const int ym = std::clamp(static_cast<int>(std::lround(my_img)), 0,
                              work_h_ - 2);
    const float pz = std::fabs(row_z_[static_cast<size_t>(ym)] -
                               row_z_[static_cast<size_t>(ym + 1)]);
    const float px = row_s_[static_cast<size_t>(ym)];
    if (med_thick > TAPE_MAX_M + 2.0f * std::max(px, pz)) {
      continue; // mang toi rong (bong, vat can), khong phai vach
    }
    seg.thick_m = med_thick;

    // Sap xep gan xe -> xa xe
    if (norm2(seg.g.front() - car_ref_) > norm2(seg.g.back() - car_ref_)) {
      std::reverse(seg.g.begin(), seg.g.end());
      std::reverse(seg.img.begin(), seg.img.end());
    }
    seg.length_m = polyline_len(seg.g);
    segs.push_back(std::move(seg));
  }
}

// ============================================================================
// NOI DOAN VACH DUT (den tran rua mat 1 doan, bang keo mon...)
// ----------------------------------------------------------------------------
// Tren mat dat, 2 doan cung 1 vach thi thang hang: dau doan sau nam gan
// duong keo dai cua doan truoc va cung huong. Noi tham lam tu doan gan xe ra
// xa. Khong ve lai vao mask, duong tam / pure pursuit tu noi suy qua cho ho.
// ============================================================================

void CameraLane::link_segments(std::vector<LinePath> &segs,
                               std::vector<LinePath> &lines) const {
  lines.clear();
  std::sort(segs.begin(), segs.end(),
            [this](const LinePath &a, const LinePath &b) {
              return norm2(a.g.front() - car_ref_) <
                     norm2(b.g.front() - car_ref_);
            });

  const float kDeg = static_cast<float>(CV_PI) / 180.0f;
  const float cos_link = std::cos(LINK_ANGLE_DEG * kDeg);
  const float cos_seg = std::cos((LINK_ANGLE_DEG + 10.0f) * kDeg);

  // Huong o dau xa cua chuoi: lui lai toi khi cach >= 6 cm
  auto end_dir = [](const std::vector<cv::Point2f> &g) {
    const cv::Point2f &e = g.back();
    for (int i = static_cast<int>(g.size()) - 2; i >= 0; --i) {
      const cv::Point2f d = e - g[static_cast<size_t>(i)];
      if (norm2(d) >= 0.06f || i == 0) {
        return unit(d);
      }
    }
    return cv::Point2f(0.0f, 1.0f);
  };

  std::vector<uchar> used(segs.size(), 0);
  for (size_t i = 0; i < segs.size(); ++i) {
    if (used[i]) {
      continue;
    }
    used[i] = 1;
    LinePath chain = segs[i];

    for (;;) {
      const cv::Point2f end = chain.g.back();
      const cv::Point2f dir = end_dir(chain.g);
      int best = -1;
      bool best_rev = false;
      float best_cost = std::numeric_limits<float>::max();

      for (size_t j = 0; j < segs.size(); ++j) {
        if (used[j]) {
          continue;
        }
        for (int r = 0; r < 2; ++r) {
          const auto &s = segs[j].g;
          const cv::Point2f st = r ? s.back() : s.front();
          const cv::Point2f fa = r ? s.front() : s.back();
          const cv::Point2f v = st - end;
          const float gap = norm2(v);
          if (gap > LINK_GAP_M) {
            continue;
          }
          if (gap > 0.04f && dir.dot(v) < cos_link * gap) {
            continue; // doan sau khong nam phia truoc dau xa
          }
          const cv::Point2f sd = fa - st;
          const float sl = norm2(sd);
          if (sl > 1e-6f && dir.dot(sd) < cos_seg * sl) {
            continue; // doan sau khac huong
          }
          const float lat = std::fabs(cross2(dir, v));
          if (lat > 0.05f + 0.3f * gap) {
            continue; // lech khoi duong keo dai
          }
          const float cost = gap + 2.0f * lat;
          if (cost < best_cost) {
            best_cost = cost;
            best = static_cast<int>(j);
            best_rev = r == 1;
          }
        }
      }

      if (best < 0) {
        break;
      }
      used[static_cast<size_t>(best)] = 1;
      LinePath s = segs[static_cast<size_t>(best)];
      if (best_rev) {
        std::reverse(s.g.begin(), s.g.end());
        std::reverse(s.img.begin(), s.img.end());
      }
      chain.g.insert(chain.g.end(), s.g.begin(), s.g.end());
      chain.img.insert(chain.img.end(), s.img.begin(), s.img.end());
      chain.thick_m = std::max(chain.thick_m, s.thick_m);
    }

    chain.length_m = polyline_len(chain.g);
    if (chain.length_m >= MIN_LINE_LEN_M) {
      lines.push_back(std::move(chain));
    }
  }
}

// ============================================================================
// NHAN TRAI / PHAI
// ----------------------------------------------------------------------------
// 1. Khop vach frame truoc (trung vi khoang cach < PRIOR_MATCH_M): giu nhan
//    cu -> khong lat nhan khi vach nam ngang truoc mui xe.
// 2. Khong khop: xe nam ben nao cua vach. Huong vach = gan xe -> xa xe; xe
//    o ben PHAI vach -> vach TRAI. Dung ca khi vach ngoai cua vat ngang truoc
//    xe (cua phai gat: vach trai chay tu trai-gan sang phai-xa, xe van o ben
//    phai cua no).
// ============================================================================

int CameraLane::classify_side(LinePath &line, bool prior_ok) const {
  line.from_prior = false;
  if (prior_ok) {
    const float inf = std::numeric_limits<float>::max();
    const float dl =
        prior_left_.size() >= 2 ? median_dist(line.g, prior_left_, false) : inf;
    const float dr = prior_right_.size() >= 2
                         ? median_dist(line.g, prior_right_, false)
                         : inf;
    if (std::min(dl, dr) < PRIOR_MATCH_M) {
      line.from_prior = true;
      return dl <= dr ? -1 : 1;
    }
  }

  const float c = car_side(line);
  if (std::fabs(c) < 0.03f) {
    return 0; // xe gan nhu nam tren vach: de detect() xet theo vach con lai
  }
  return c < 0.0f ? -1 : 1;
}

// Xe nam ben nao cua vach, xet tren DOAN GAN XE (NEAR_SIDE_M dau tien): < 0
// xe ben PHAI vach, > 0 ben TRAI, |gia tri| ~ khoang cach (m). Khong lay ca
// vach: vach trong cua cua rat gat cuon thanh moc cau, huong ca vach lech han
// -> gan nham vach phai thanh vach trai.
float CameraLane::car_side(const LinePath &line) const {
  const auto &g = line.g;
  size_t k = 1;
  float acc = 0.0f;
  while (k + 1 < g.size() && acc + norm2(g[k] - g[k - 1]) < NEAR_SIDE_M) {
    acc += norm2(g[k] - g[k - 1]);
    ++k;
  }
  const cv::Point2f t = unit(g[k] - g[0]);
  const cv::Point2f mid = 0.5f * (g[0] + g[k]);
  // Xe = CHAN CAMERA (0, 0), khong phai hang day ROI: vach ngoai cua cua gat
  // cat ngang ngay hang day ROI thi phep xet tai do ra ~0 va lat dau.
  return cross2(t, cv::Point2f(0.0f, 0.0f) - mid);
}

// ============================================================================
// TIEN ICH HINH HOC MAT DAT
// ============================================================================

double CameraLane::ground_dist_m(double y) const {
  // goc nhin xuong so voi phuong ngang cua tia qua hang y
  const double psi = pitch_ + std::atan((y - work_h_ / 2.0) / f_px_);
  return psi > 1e-3 ? h_ / std::tan(psi)
                    : std::numeric_limits<double>::infinity();
}

bool CameraLane::ground_to_img(const cv::Point2f &g, cv::Point2d &p) const {
  if (g.y <= 0.02f) {
    return false;
  }
  const double psi = std::atan2(static_cast<double>(h_), static_cast<double>(g.y));
  const double y = work_h_ / 2.0 + f_px_ * std::tan(psi - pitch_);
  const double dy = y - horizon_y_;
  if (dy <= 1.0) {
    return false;
  }
  p = cv::Point2d(WORK_W / 2.0 + g.x * dy / k_, y);
  return true;
}

// Khoang cach tu p toi duong gap khuc. interior = diem gan nhat KHONG phai
// 2 dau mut (hinh chieu roi vao giua duong).
float CameraLane::dist_to_polyline(const cv::Point2f &p,
                                   const std::vector<cv::Point2f> &poly,
                                   bool *interior) {
  if (poly.empty()) {
    if (interior) {
      *interior = false;
    }
    return std::numeric_limits<float>::max();
  }
  if (poly.size() == 1) {
    if (interior) {
      *interior = false;
    }
    return norm2(p - poly[0]);
  }
  float best = std::numeric_limits<float>::max();
  bool best_in = false;
  const size_t last = poly.size() - 2;
  for (size_t i = 0; i + 1 < poly.size(); ++i) {
    const cv::Point2f a = poly[i];
    const cv::Point2f ab = poly[i + 1] - a;
    const float L2 = ab.dot(ab);
    float t = L2 > 1e-9f ? (p - a).dot(ab) / L2 : 0.0f;
    bool in = true;
    if (t <= 0.0f) {
      t = 0.0f;
      in = i > 0;
    } else if (t >= 1.0f) {
      t = 1.0f;
      in = i < last;
    }
    const float d = norm2(p - (a + ab * t));
    if (d < best) {
      best = d;
      best_in = in;
    }
  }
  if (interior) {
    *interior = best_in;
  }
  return best;
}

// Trung vi khoang cach tu cac diem toi duong gap khuc. interior_only: chi
// tinh diem co hinh chieu nam giua duong (phan 2 vach chong len nhau);
// it hon 3 diem nhu vay -> NaN.
float CameraLane::median_dist(const std::vector<cv::Point2f> &pts,
                              const std::vector<cv::Point2f> &poly,
                              bool interior_only) {
  std::vector<float> d;
  d.reserve(pts.size());
  for (const auto &p : pts) {
    bool in = false;
    const float v = dist_to_polyline(p, poly, &in);
    if (!interior_only || in) {
      d.push_back(v);
    }
  }
  if (d.empty() || (interior_only && d.size() < 3)) {
    return interior_only ? std::numeric_limits<float>::quiet_NaN()
                         : std::numeric_limits<float>::max();
  }
  std::nth_element(d.begin(), d.begin() + d.size() / 2, d.end());
  return d[d.size() / 2];
}

// Tiep tuyen don vi tai diem i, lay tren cung +/-TANGENT_HALF_M (tam vach
// o doan xa nhay vai cm, lay tren 2 diem ke nhau thi phap tuyen rang cua)
cv::Point2f CameraLane::tangent_at(const std::vector<cv::Point2f> &p,
                                   size_t i) {
  if (p.size() < 2) {
    return cv::Point2f(0.0f, 1.0f);
  }
  i = std::min(i, p.size() - 1);
  size_t a = i;
  while (a > 0 && norm2(p[i] - p[a]) < TANGENT_HALF_M) {
    --a;
  }
  size_t b = i;
  while (b + 1 < p.size() && norm2(p[b] - p[i]) < TANGENT_HALF_M) {
    ++b;
  }
  return unit(p[b] - p[a]);
}

// Doi vach vao TRONG lan offset_m theo phap tuyen. "Trong" = ben co xe (xe
// luon nam trong lan), nen khong phu thuoc nhan trai/phai hay chieu vach.
void CameraLane::offset_path(const LinePath &line, float offset_m,
                             std::vector<cv::Point2f> &out) const {
  out.clear();
  if (line.g.size() < 2) {
    return;
  }
  float c = car_side(line);
  if (std::fabs(c) < 0.05f) {
    // Xe sat / de len vach: "ben co xe" khong tin duoc, theo nhan trai/phai
    // (luc nay vach chay doc xe nen chieu gan -> xa la chac chan)
    c = line.side < 0 ? -1.0f : 1.0f;
  }
  // c < 0: xe ben PHAI vach -> phap tuyen phai (t.y, -t.x)
  const float sgn = c < 0.0f ? 1.0f : -1.0f;

  out.reserve(line.g.size());
  for (size_t i = 0; i < line.g.size(); ++i) {
    const cv::Point2f t = tangent_at(line.g, i);
    out.push_back(line.g[i] + cv::Point2f(t.y, -t.x) * (sgn * offset_m));
  }
}

// Lay mau lai duong gap khuc deu `step` met, lam tron nhe (TB 3 diem)
std::vector<cv::Point2f> CameraLane::resample(const std::vector<cv::Point2f> &p,
                                              float step) {
  std::vector<cv::Point2f> r;
  if (p.size() < 2) {
    return p;
  }
  r.push_back(p.front());
  float carry = 0.0f; // quang duong da di tu diem lay mau truoc
  for (size_t i = 1; i < p.size(); ++i) {
    const cv::Point2f a = p[i - 1];
    const cv::Point2f d = p[i] - a;
    const float L = norm2(d);
    float pos = step - carry;
    while (pos <= L) {
      r.push_back(a + d * (pos / std::max(L, 1e-6f)));
      pos += step;
    }
    carry = L - (pos - step);
  }
  if (norm2(r.back() - p.back()) > 0.3f * step) {
    r.push_back(p.back());
  }
  if (r.size() >= 3) {
    std::vector<cv::Point2f> s = r;
    for (size_t i = 1; i + 1 < r.size(); ++i) {
      s[i] = (r[i - 1] + r[i] + r[i + 1]) * (1.0f / 3.0f);
    }
    r.swap(s);
  }
  return r;
}

// Diem tren duong (gan -> xa) cach chan camera L met. Duong bat dau xa hon L
// -> lay diem dau; ket thuc truoc L -> lay diem cuoi.
bool CameraLane::pursuit_point(const std::vector<cv::Point2f> &path, float L,
                               cv::Point2f &pt) {
  if (path.size() < 2) {
    return false;
  }
  float d_prev = norm2(path.front());
  if (d_prev >= L) {
    pt = path.front();
    return true;
  }
  for (size_t i = 1; i < path.size(); ++i) {
    const float d = norm2(path[i]);
    if (d >= L) {
      const float t = (L - d_prev) / std::max(1e-6f, d - d_prev);
      pt = path[i - 1] + (path[i] - path[i - 1]) * t;
      return true;
    }
    d_prev = d;
  }
  pt = path.back();
  return true;
}
