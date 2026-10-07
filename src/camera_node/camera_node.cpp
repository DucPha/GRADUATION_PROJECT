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
  prior_left_.fill(-1);
  prior_right_.fill(-1);

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
      4 * N_WINDOWS + 20, 4 * WORK_W);

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
  const double sy = frame.rows / static_cast<double>(WH);

  // ---- 1. Vung lam viec (toa do khung WORK_W x work_h_) ------------------
  // Day ROI: theo profile, nhung khong thap hon hang ma be ngang anh con
  // chua duoc 1 lan LANE_W_FIT_M cong le 2 ben
  const int fit_dy =
      static_cast<int>(WORK_W * k_ / (LANE_W_FIT_M + 2.0f * ROI_SIDE_MARGIN_M));
  const int bottom =
      std::clamp(std::min(static_cast<int>(std::lround(roi_bottom_frac_ * WH)),
                          horizon_y_ + fit_dy),
                 2 * N_WINDOWS + 1, WH);

  int top = static_cast<int>(std::lround(roi_top_frac_.load() * WH));

  // Dinh ROI khong duoc cao hon chan troi + ROI_MIN_DY
  if (top < horizon_y_ + ROI_MIN_DY) {
    top = horizon_y_ + ROI_MIN_DY;
    if (!warned_roi_) {
      warned_roi_ = true;
      std::cerr << "[CameraLane] ROI top is above horizon+" << ROI_MIN_DY
                << ", clamped to row " << top << " of " << WH
                << ". Check horizon_y / axis_ground_m (purple line on vis).\n";
    }
  }
  top = std::clamp(top, 0, bottom - 2 * N_WINDOWS);

  const int roi_h = bottom - top;
  const int win_h = roi_h / N_WINDOWS;

  if (!logged_geometry_) {
    logged_geometry_ = true;
    std::cout << "[CameraLane] geometry: camera " << cam_w_ << "x" << cam_h_
              << ", decoded " << frame.cols << "x" << frame.rows << " -> work " << WORK_W << "x" << WH
              << ", h=" << h_ << " m, pitch=" << pitch_ * 180.0 / CV_PI
              << " deg, f=" << f_px_ << " px, horizon_y=" << horizon_y_
              << "; ROI rows " << top << ".." << bottom << " = "
              << ground_dist_m(bottom) << ".." << ground_dist_m(top)
              << " m, window height " << win_h << " px\n";
  }

  // Cua so i: i = 0 gan xe nhat (duoi cung), i = N-1 xa nhat
  auto band_of = [&](int i, int &y0, int &y1) {
    y1 = bottom - i * win_h;
    y0 = y1 - win_h;
  };

  // ---- 2. Cat ROI, thu ve 320 cot, xam -> blur -> CLAHE ----------------
  // Chi xu ly phan ROI cua anh da giai ma (~35% so pixel)
  const int fy0 =
      std::clamp(static_cast<int>(std::lround(top * sy)), 0, frame.rows - 1);
  const int fy1 = std::clamp(static_cast<int>(std::lround(bottom * sy)),
                             fy0 + 1, frame.rows);

  cv::Mat gray;
  {
    // Doi sang xam TRUOC khi thu nho: it du lieu hon 3 lan cho buoc resize
    cv::Mat roi_gray;
    cv::cvtColor(frame.rowRange(fy0, fy1), roi_gray, cv::COLOR_BGR2GRAY);
    cv::resize(roi_gray, gray, cv::Size(WORK_W, roi_h), 0, 0, cv::INTER_AREA);
  }
  cv::GaussianBlur(gray, gray, cv::Size(BLUR_KSIZE, BLUR_KSIZE), 0);

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

  // ---- 4. TRU NEN (background subtraction) + Otsu + morphology ----------
  // Den tran chieu xuong lam 2 viec: (1) "trang trang" ca vach lan san trong
  // vung loa -> vach van toi hon san NGAY CANH no nhung co the sang hon nguong
  // chung cua ca anh -> nguong toan cuc lam mat doan vach; (2) tao rim sang/
  // toi o mep vung loa -> nguong toan cuc sinh vet gia hinh cung.
  //
  // Cach giai: uoc luong ANH NEN = do sang san + loa, KHONG co vach, bang phep
  // dong hinh thai hoc (gian roi co) voi cua so BG_KERNEL_W x BG_KERNEL_H rong
  // hon be ngang vach: moi vat toi hep hon cua so (vach) bi "lap" bang mau san
  // xung quanh, con loa va do sang chung giu nguyen. Lay
  //       diff = nen - anh       (>= 0)
  // thi: vach (ke ca trong vung loa) -> diff lon; san, loa, bien thien sang
  // tu tu -> diff ~ 0; vat toi RONG hon cua so (bong ghe, vat can) -> nen cung
  // toi theo -> diff ~ 0, tu dong bi loai. Otsu tren diff (khong tren anh xam)
  // nen nguong tu thich nghi theo do tuong phan vach / san con lai.
  cv::Mat bg;
  static const cv::Mat k_bg = cv::getStructuringElement(
      cv::MORPH_RECT, cv::Size(BG_KERNEL_W, BG_KERNEL_H));
  cv::morphologyEx(gray, bg, cv::MORPH_CLOSE, k_bg, cv::Point(-1, -1), 1,
                   cv::BORDER_REPLICATE);
  cv::Mat diff;
  cv::subtract(bg, gray, diff);

  // bin chi co 0/1 nen tong cot = so pixel vach
  double contrast = 0.0;
  const int thr = std::max(otsu_masked(diff, mask, contrast), BG_MIN_DIFF);

  cv::Mat bin = cv::Mat::zeros(roi_h, WORK_W, CV_8U);
  if (contrast >= MIN_CONTRAST) {
    // NGUONG TRE (hysteresis): doan vach nam trong vung loa chi con diff
    // 15-35 trong khi doan ngoai loa ~100, mot nguong Otsu duy nhat se cat mat
    // doan yeu. Giu pixel YEU (diff > BG_WEAK_DIFF) neu no NOI LIEN voi pixel
    // MANH (diff > nguong Otsu): doan vach bi loa duoc noi lai voi phan vach
    // con ro; van go, vet ban roi rac khong noi voi vach that thi bi bo.
    cv::Mat weak;
    cv::threshold(diff, weak, BG_WEAK_DIFF, 1, cv::THRESH_BINARY);
    cv::bitwise_and(weak, mask, weak);

    cv::Mat labels;
    const int n_lab = cv::connectedComponents(weak, labels, 8, CV_32S);
    std::vector<uchar> keep(static_cast<size_t>(n_lab), 0); // nhan 0 = nen, luon 0
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

    static const cv::Mat k_close =
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 15));
    static const cv::Mat k_open =
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(bin, bin, cv::MORPH_CLOSE, k_close);
    cv::morphologyEx(bin, bin, cv::MORPH_OPEN, k_open);
  }

  // ---- 5. Cua so truot ----------------------------------------------------
  // Vach cach truc camera x px o hang cach chan troi d0 thi o hang d1 se
  // cach x * d1 / d0 (phoi canh). WINDOW_MARGIN chi con bu cho do cong.
  auto predict = [&](int x, int y_from, int y_to) {
    const int d0 = y_from - horizon_y_;
    const int d1 = y_to - horizon_y_;
    if (d0 <= 1 || d1 <= 1) {
      return x;
    }
    return cx + static_cast<int>(
                    std::lround((x - cx) * static_cast<double>(d1) / d0));
  };

  // Tong cot cua tung cua so tinh 1 lan, dung chung cho moi luot quet
  std::vector<cv::Mat> col_sums(N_WINDOWS);
  auto compute_col_sums = [&]() {
    for (int i = 0; i < N_WINDOWS; ++i) {
      int y0 = 0;
      int y1 = 0;
      band_of(i, y0, y1);
      cv::reduce(bin.rowRange(y0 - top, y1 - top), col_sums[i], 0,
                 cv::REDUCE_SUM, CV_32S);
    }
  };
  compute_col_sums();

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

  // 3 cach quet:
  //  PRIOR      : tim quanh vi tri vach cua frame truoc (ban hep) -> on dinh,
  //               khong bi hut sang vat la khi xe dang bam lan tot.
  //  FAR_TO_NEAR: tim cap vach dau tien o cua so xa (lan hep, thay du 2 vach).
  //  NEAR_TO_FAR: tim o cua so gan xe, dung khi vao cua gap.
  enum class Pass { PRIOR, FAR_TO_NEAR, NEAR_TO_FAR };

  const bool prior_ok =
      prior_time_ != std::chrono::steady_clock::time_point{} &&
      std::chrono::duration_cast<std::chrono::milliseconds>(t_begin -
                                                            prior_time_)
              .count() <= PRIOR_MAX_AGE_MS;

  auto run_pass = [&](Pass mode) {
    Track t;
    t.band_ok.assign(N_WINDOWS, false);
    t.peak_left.assign(N_WINDOWS, -1);
    t.peak_right.assign(N_WINDOWS, -1);

    int seed_left = -1;
    int seed_right = -1;
    int yl = 0;
    int yr = 0;
    const bool far_to_near = mode != Pass::NEAR_TO_FAR;

    for (int n = 0; n < N_WINDOWS; ++n) {
      const int i = far_to_near ? (N_WINDOWS - 1 - n) : n;

      int y0 = 0;
      int y1 = 0;
      band_of(i, y0, y1);
      const int y_mid = (y0 + y1) / 2;
      const int dy = y_mid - horizon_y_;
      const cv::Mat &col_sum = col_sums[i];

      int pl = -1;
      int pr = -1;
      int ml = WINDOW_MARGIN;
      int mr = WINDOW_MARGIN;

      if (mode == Pass::PRIOR) {
        // Uu tien vi tri cua frame truoc o dung cua so nay; khong co thi du
        // doan theo phoi canh tu vach vua tim duoc o cua so tren
        if (prior_left_[i] >= 0) {
          pl = prior_left_[i];
          ml = PRIOR_MARGIN;
        } else if (seed_left >= 0) {
          pl = predict(seed_left, yl, y_mid);
        }
        if (prior_right_[i] >= 0) {
          pr = prior_right_[i];
          mr = PRIOR_MARGIN;
        } else if (seed_right >= 0) {
          pr = predict(seed_right, yr, y_mid);
        }
        if (pl < 0 && pr < 0) {
          continue;
        }
      } else {
        if (seed_left < 0) {
          if (n >= MAX_SEED_SEARCH) {
            break;
          }
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
        pl = predict(seed_left, yl, y_mid);
        pr = predict(seed_right, yr, y_mid);
      }

      int left = 0;
      int right = 0;
      const bool okL =
          pl >= 0 && peak_in(col_sum, pl, ml, run_limit_px(pl, dy, win_h), left);
      const bool okR = pr >= 0 && peak_in(col_sum, pr, mr,
                                          run_limit_px(pr, dy, win_h), right);

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

  // Chay cac luot quet, lay luot diem cao nhat. Bang diem thi uu tien PRIOR
  // (giu lien tuc voi frame truoc = khong giat).
  auto run_all = [&]() {
    Track best;
    bool have = false;
    if (prior_ok) {
      best = run_pass(Pass::PRIOR);
      have = true;
    }
    Track fresh = run_pass(Pass::FAR_TO_NEAR);
    if (!have || fresh.score() > best.score()) {
      best = std::move(fresh);
    }
    if (best.matched < MIN_MATCHED_WINDOWS) {
      Track alt = run_pass(Pass::NEAR_TO_FAR);
      if (alt.score() > best.score()) {
        best = std::move(alt);
      }
    }
    return best;
  };

  Track trk = run_all();

  // ---- 5b. Phuc hoi doan vach bi ngat do den tran -----------------------
  // Den tran rua sang lam vach toi thoat len -> THRESH_BINARY_INV mat doan
  // vach. Neu 2 ben doan dut con vach lam moc thi noi suy tuyen tinh, ve
  // vach lai vao bin roi chay lai tracking. Doan dai hon BRIDGE_MAX_BANDS
  // thi khong noi (khong doan duong gia).
  auto bridge_gaps = [&](const Track &t) -> bool {
    bool painted = false;
    const std::vector<int> *sides[2] = {&t.peak_left, &t.peak_right};

    for (const std::vector<int> *peaks : sides) {
      for (int i = 0; i < N_WINDOWS; ++i) {
        if ((*peaks)[i] < 0) {
          continue;
        }

        // Moc hop le tiep theo phia tren doan dut
        int j = i + 1;
        while (j < N_WINDOWS && (*peaks)[j] < 0) {
          ++j;
        }
        if (j >= N_WINDOWS) {
          break;
        }
        if (j - i - 1 > BRIDGE_MAX_BANDS || j == i + 1) {
          continue; // khong co doan dut, hoac doan qua dai
        }

        int ya0 = 0, ya1 = 0, yb0 = 0, yb1 = 0;
        band_of(i, ya0, ya1);
        band_of(j, yb0, yb1);
        const int xa = (*peaks)[i];
        const int xb = (*peaks)[j];
        const int y_a = (ya0 + ya1) / 2;
        const int y_b = (yb0 + yb1) / 2;
        const double span = static_cast<double>(y_b - y_a);
        if (std::fabs(span) < 1.0) {
          continue;
        }

        for (int b = i + 1; b < j; ++b) {
          int y0 = 0;
          int y1 = 0;
          band_of(b, y0, y1);
          for (int y = y0; y < y1; ++y) {
            const int dy = y - horizon_y_;
            const int yy = y - top;
            if (dy <= 1 || yy < 0 || yy >= bin.rows) {
              continue;
            }
            const int x = cvRound(xa + (xb - xa) * (y - y_a) / span);
            const int half = std::max(1, lane_px(0.5f * TAPE_MAX_M, dy));
            const int x0 = std::max(0, x - half);
            const int x1 = std::min(WORK_W - 1, x + half);
            if (x1 >= x0) {
              bin.row(yy).colRange(x0, x1 + 1).setTo(1);
              painted = true;
            }
          }
        }
      }
    }

    return painted;
  };

  if (bridge_gaps(trk)) {
    compute_col_sums();
    Track fixed = run_all();
    if (fixed.score() > trk.score()) {
      trk = std::move(fixed);
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
  // Do o cua so GAN XE nhat co du 2 vach (vach rong nhat, chinh xac nhat)
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

  out.horizon_frac = static_cast<float>(horizon_y_) / WH;

  // ---- 7. Diem tam lan cua tung cua so ----------------------------------
  // 2 vach: trung diem. 1 vach: vach +/- nua be rong lan (da hoc) doi ra
  // pixel theo hang do. Khong du 2 vach thi chi dung ben co nhieu cua so hon.
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
  // Hệ số đổi px khung làm việc -> px ảnh tham chiếu 640 (firmware)
  constexpr double kRef = static_cast<double>(DEV_REF_W) / WORK_W;

  if (state != LaneState::LOST) {
    bool have = false;

    // Fit co loai diem lech: 1 diem tam sai (bong, vat la) khong keo ca
    // duong tam theo
    out.fit_ok = fit_poly2_robust(cpts, cf, cy0, cy1);
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
      // Fit that bai: lay diem tam gan hang nhin truoc nhat, quy ve hang
      // y_look bang ti le phoi canh
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

    // Lech ngang (cm) cua duong tam o xa so voi gan. > 0: duong re phai.
    {
      const cv::Point &pf = cpts.front();
      const cv::Point &pn = cpts.back();
      const double xf = out.fit_ok ? eval_fit(cf, pf.y, cy0, cy1) : pf.x;
      const double xn = out.fit_ok ? eval_fit(cf, pn.y, cy0, cy1) : pn.x;
      const int df = std::max(2, pf.y - horizon_y_);
      const int dn = std::max(2, pn.y - horizon_y_);
      const double Xf = (xf - cx) * k_ / df;
      const double Xn = (xn - cx) * k_ / dn;
      out.path_dx_cm = static_cast<float>(100.0 * (Xf - Xn));
    }

    // Do cong: fit X(Z) = c0 + c1*Z + c2*Z^2 bang met that, 1/R = 2*c2
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

    // ---- 8b. Chan nhay + loc EMA (don vi px anh tham chieu 640) --------
    const int raw = static_cast<int>(std::lround((centre_look - cx) * kRef));

    if (!ema_primed_) {
      dev_ema_ = raw;
      ema_primed_ = true;
      jump_count_ = 0;
    } else {
      const bool jump = std::abs(raw - dev_ema_) > JUMP_GATE_REF_PX;
      if (jump && ++jump_count_ < JUMP_CONFIRM_FRAMES) {
        // Nhay dot ngot 1 frame: nhieu nhieu kha nang la nhan nham -> giu
        // gia tri cu. Neu frame sau van lech nhu vay thi chap nhan.
        out.gated = true;
      } else {
        jump_count_ = 0;
        dev_ema_ = static_cast<int>(
            std::lround(EMA_ALPHA * raw + (1.0 - EMA_ALPHA) * dev_ema_));
      }
    }

    out.dev_cm = px_to_cm(static_cast<float>(dev_ema_ / kRef), y_look);
  }

  // ---- 9. Giu gia tri khi mat lan, he so toc do ------------------------
  if (state != LaneState::LOST) {
    last_valid_time_ = t_begin;
  }
  const auto lost_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                           t_begin - last_valid_time_)
                           .count();
  const bool holding = state == LaneState::LOST && lost_ms <= HOLD_MS;

  // Mat lan qua lau: lan sau bat lai thi bo qua EMA cu
  if (state == LaneState::LOST && !holding) {
    ema_primed_ = false;
    jump_count_ = 0;
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

    // It cua so thay vach = camera dang kho do: giam toc
    if (state == LaneState::ONE_LINE ||
        static_cast<int>(cpts.size()) <= MIN_MATCHED_WINDOWS) {
      speed = std::min(speed, SPEED_ONE_LINE);
    }
    // Fit that bai: khong biet duong thang hay cua -> khong day toc
    if (!out.fit_ok) {
      speed = std::min(speed, SPEED_ONE_LINE);
    }
  }
  out.speed_scale = speed;

  // ---- 10. Luu vi tri vach lam goi y cho frame sau ----------------------
  // Chi luu khi ket qua dang tin: du 2 vach (ca 2 ben) hoac 1 vach (ben do).
  if (state == LaneState::TWO_LINES) {
    std::copy(peak_left.begin(), peak_left.end(), prior_left_.begin());
    std::copy(peak_right.begin(), peak_right.end(), prior_right_.begin());
    prior_time_ = t_begin;
  } else if (state == LaneState::ONE_LINE) {
    if (use_left) {
      std::copy(peak_left.begin(), peak_left.end(), prior_left_.begin());
      prior_right_.fill(-1);
    } else {
      std::copy(peak_right.begin(), peak_right.end(), prior_right_.begin());
      prior_left_.fill(-1);
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
  // Ve tren anh thu nho rong VIS_W de nhe CPU va encode JPEG nhanh.
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

  // Vung lan (to mau trong suot giua 2 vach) - chi khi du 2 vach
  if (two_lanes && trk.left_pts.size() >= 2) {
    std::vector<cv::Point> area;
    for (const auto &p : trk.left_pts) {
      area.push_back(to_vis(p.x, p.y));
    }
    for (auto it = trk.right_pts.rbegin(); it != trk.right_pts.rend(); ++it) {
      area.push_back(to_vis(it->x, it->y));
    }
    cv::Mat layer = vis.clone();
    cv::fillPoly(layer, std::vector<std::vector<cv::Point>>{area},
                 cv::Scalar(80, 200, 80), cv::LINE_AA);
    cv::addWeighted(layer, 0.28, vis, 0.72, 0.0, vis);
  }

  // Hinh thang ROI
  {
    std::vector<cv::Point> roi_vis;
    for (const auto &p : roi_poly) {
      roi_vis.push_back(to_vis(p.x, p.y));
    }
    cv::polylines(vis, roi_vis, true, cv::Scalar(255, 190, 0), 1, cv::LINE_AA);
  }

  // Chan troi (de do lai horizon_y). Cam cui xuong thi nam ngoai anh.
  if (horizon_y_ >= 0 && horizon_y_ < WH) {
    cv::line(vis, to_vis(0, horizon_y_), to_vis(WORK_W, horizon_y_),
             cv::Scalar(255, 0, 255), 1, cv::LINE_AA);
  }

  // Hang lay do lech
  cv::line(vis, to_vis(0, y_look), to_vis(WORK_W, y_look),
           cv::Scalar(0, 220, 255), 1, cv::LINE_AA);

  // Cua so: xam = du 2 vach, cam = 1 vach, do = khong co
  for (int i = 0; i < N_WINDOWS; ++i) {
    int y0 = 0;
    int y1 = 0;
    band_of(i, y0, y1);
    const bool seen = peak_left[i] >= 0 || peak_right[i] >= 0;
    const cv::Scalar c = band_ok[i] ? cv::Scalar(120, 120, 120)
                                    : (seen ? cv::Scalar(0, 160, 255)
                                            : cv::Scalar(0, 0, 220));
    const int y_mid = (y0 + y1) / 2;
    if (peak_left[i] >= 0) {
      cv::rectangle(vis, to_vis(peak_left[i] - WINDOW_MARGIN / 2, y0),
                    to_vis(peak_left[i] + WINDOW_MARGIN / 2, y1), c, 1);
      cv::circle(vis, to_vis(peak_left[i], y_mid), 4, cv::Scalar(0, 230, 0),
                 -1, cv::LINE_AA);
    }
    if (peak_right[i] >= 0) {
      cv::rectangle(vis, to_vis(peak_right[i] - WINDOW_MARGIN / 2, y0),
                    to_vis(peak_right[i] + WINDOW_MARGIN / 2, y1), c, 1);
      cv::circle(vis, to_vis(peak_right[i], y_mid), 4, cv::Scalar(255, 120, 0),
                 -1, cv::LINE_AA);
    }
    if (!seen) {
      cv::rectangle(vis, to_vis(cx - 4, y0), to_vis(cx + 4, y1), c, 1);
    }
  }

  for (const auto &p : cpts) {
    cv::circle(vis, to_vis(p.x, p.y), 3, cv::Scalar(0, 230, 255), -1,
               cv::LINE_AA);
  }

  if (out.fit_ok) {
    std::vector<cv::Point> poly;
    poly.reserve(21);
    for (int s = 0; s <= 20; ++s) {
      const int y = cy0 + (cy1 - cy0) * s / 20;
      const double x = eval_fit(cf, y, cy0, cy1);
      if (std::isfinite(x)) {
        poly.push_back(to_vis(x, y));
      }
    }
    if (poly.size() > 1) {
      cv::polylines(vis, poly, false, cv::Scalar(0, 230, 255), 2, cv::LINE_AA);
    }
  }

  // Truc giua anh va diem lay do lech (tam lan sau loc)
  cv::line(vis, to_vis(cx, y_look - 6), to_vis(cx, y_look + 6),
           cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
  if (state != LaneState::LOST || holding) {
    const double cx_f = cx + dev_ema_ / kRef;
    cv::circle(vis, to_vis(cx_f, y_look), 7, cv::Scalar(0, 230, 255), 2,
               cv::LINE_AA);
  }

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
      flag_text = use_left ? "1 LANE (L)" : "1 LANE (R)";
    }
    if (out.gated) {
      flag_text += " *";
    }
    cv::putText(vis, flag_text, cv::Point(8, 18), cv::FONT_HERSHEY_SIMPLEX,
                0.55, flag_color, 1, cv::LINE_AA);

    char info[200];
    std::snprintf(info, sizeof(info),
                  "dev %+dpx %+.0fcm  w %.0fcm  v %.0f%%  ctr %.0f  "
                  "horizon %.2f",
                  out.dev_px, out.dev_cm, lane_w_m_ * 100.0f,
                  out.speed_scale * 100.0f, contrast,
                  static_cast<double>(horizon_y_) / WH);
    cv::putText(vis, info, cv::Point(118, 18), cv::FONT_HERSHEY_SIMPLEX, 0.40,
                cv::Scalar(235, 235, 235), 1, cv::LINE_AA);
  }

  out.vis = vis;
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
  const double psi = pitch_ + std::atan((y - work_h_ / 2.0) / f_px_);
  return psi > 1e-3 ? h_ / std::tan(psi)
                    : std::numeric_limits<double>::infinity();
}

// Run cot cua 1 vach: be rong bang keo (theo phoi canh) + do loe do vach
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
// TIM CAP 2 VACH
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

  const int *cs = col_sum.ptr<int>(0);
  for (int x = 0; x < col_sum.cols; ++x) {
    const bool on = cs[x] >= WINDOW_MIN_POINTS;
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
      // Cung sai so be rong: uu tien cap nam can doi quanh giua anh
      const int err = 2 * std::abs(gap - want) +
                      std::abs((centers[a] + centers[b]) / 2 - WORK_W / 2) / 4;
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
// TIM TAM CUA 1 VACH
// ============================================================================

bool CameraLane::peak_in(const cv::Mat &col_sum, int seed, int margin,
                         int max_run, int &peak) {
  const int lo = std::max(0, seed - margin);
  const int hi = std::min(col_sum.cols - 1, seed + margin);
  if (lo > hi) {
    return false;
  }

  const int *cs = col_sum.ptr<int>(0);

  // Cot cao nhat; bang nhau thi lay cot gan vi tri du doan nhat
  int best_x = -1;
  int best_v = 0;
  for (int x = lo; x <= hi; ++x) {
    const int v = cs[x];
    if (v > best_v ||
        (v == best_v && best_x >= 0 &&
         std::abs(x - seed) < std::abs(best_x - seed))) {
      best_v = v;
      best_x = x;
    }
  }

  if (best_x < 0 || best_v < WINDOW_MIN_POINTS) {
    return false;
  }

  // Vach nghieng lam dinh cot phang: lay giua doan cao tu nua dinh tro len
  const int level = std::max(WINDOW_MIN_POINTS, (best_v + 1) / 2);
  int a = best_x;
  int b = best_x;
  while (a > 0 && cs[a - 1] >= level) {
    --a;
  }
  while (b < col_sum.cols - 1 && cs[b + 1] >= level) {
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
// FIT DUONG BAC 2 THEO HANG ANH
// ============================================================================

// Giai x = a*t^2 + b*t + c bang binh phuong nho nhat, t = (y-y0)/(y1-y0).
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

  std::vector<double> ts;
  std::vector<double> xs;
  ts.reserve(pts.size());
  xs.reserve(pts.size());
  for (const auto &p : pts) {
    ts.push_back(static_cast<double>(p.y - lo) * inv);
    xs.push_back(p.x);
  }

  // fit_quad tra ve he so theo thu tu [c, b, a] (bac 0, 1, 2)
  double q[3];
  if (!fit_quad(ts, xs, q)) {
    return false;
  }
  coef[0] = q[2];
  coef[1] = q[1];
  coef[2] = q[0];

  // Duong cong uon qua manh trong khung: so ao
  return std::fabs(coef[0]) <= 4.0 * WORK_W;
}

bool CameraLane::fit_poly2_robust(std::vector<cv::Point> &pts, double coef[3],
                                  int &y0, int &y1) {
  if (!fit_poly2(pts, coef, y0, y1)) {
    return false;
  }

  // Loai toi da 2 diem lech nhat (> CENTRE_OUTLIER_PX), giu toi thieu 4 diem
  for (int round = 0; round < 2 && pts.size() > 4; ++round) {
    const double span = std::max(1, y1 - y0);
    size_t worst = 0;
    double worst_err = 0.0;
    for (size_t i = 0; i < pts.size(); ++i) {
      const double t = (pts[i].y - y0) / span;
      const double fx = coef[0] * t * t + coef[1] * t + coef[2];
      const double e = std::fabs(fx - pts[i].x);
      if (e > worst_err) {
        worst_err = e;
        worst = i;
      }
    }
    if (worst_err <= CENTRE_OUTLIER_PX) {
      break;
    }
    pts.erase(pts.begin() + static_cast<long>(worst));
    if (!fit_poly2(pts, coef, y0, y1)) {
      return false;
    }
  }
  return true;
}

// ============================================================================
// PX SANG CENTIMET
// ============================================================================

// Camera cao h, cui xuong pitch, chan troi o hang horizon_y_, mat dat phang.
// Doan rong w_px o hang y <-> W = w_px * h / (cos(pitch) * dy) met tren duong.
float CameraLane::px_to_cm(float px, int y) const {
  const int dy = y - horizon_y_;
  if (dy <= 1) {
    return 0.0f; // gan chan troi: khong do duoc
  }
  return px * k_ * 100.0f / static_cast<float>(dy);
}
