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
      roi_top_frac_(std::clamp(profile.roi_top_frac, 0.0f, 0.90f)) {
  h_ = std::max(0.05f, profile_.height_m);
  roi_bottom_frac_ = std::clamp(profile_.roi_bottom_frac, 0.1f, 1.0f);
  lane_w_m_ = std::clamp(profile_.lane_width_m, 0.2f, 1.0f);
  pitch_ = std::clamp(static_cast<double>(profile_.pitch_deg), 1.0, 85.0) *
           CV_PI / 180.0;

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
      40, 4 * WORK_W);

  // Goc DOC giong nhau o moi do phan giai cua cam nay -> tieu cu theo chieu cao
  const double vfov =
      std::clamp(static_cast<double>(profile_.vfov_deg), 10.0, 150.0);
  f_px_ = (work_h_ / 2.0) / std::tan(vfov * CV_PI / 360.0);
  cy_ = work_h_ / 2.0;

  set_pitch(pitch_ * 180.0 / CV_PI);
  logged_geometry_ = false;
}

void CameraLane::set_pitch(double pitch_deg) {
  pitch_ = pitch_deg * CV_PI / 180.0;
  horizon_y_ = cy_ - f_px_ * std::tan(pitch_);
  build_bev();
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
  roi_top_frac_.store(std::clamp(frac, 0.0f, 0.90f));
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
    if (exposure_ >= 0) {
      lock_exposure();
    }

    std::cout << "[CameraLane] Camera opened at index " << idx << ", real size "
              << cam_w_ << "x" << cam_h_ << ", format " << fcc_name
              << ", driver fps " << cap_.get(cv::CAP_PROP_FPS) << ", decode "
              << (raw_mjpg_ ? "libjpeg " : "opencv ") << probe.cols << "x"
              << probe.rows << ", work frame " << WORK_W << "x" << work_h_
              << ", exposure "
              << (exposure_ >= 0 ? std::to_string(locked_exposure_) + " (khoa)"
                                 : std::string("auto"))
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
// KHOA PHOI SANG (giong camera_node.py: CAMERA_LOCK_EXPOSURE)
// ----------------------------------------------------------------------------
// Phoi sang tu dong cua cam nay keo dai thoi gian chup khi phong toi (do: 25.8
// thay vi 30 fps) va "bom" sang/toi moi khi vet loa den tran lot vao khung ->
// do tuong phan vach doi theo frame. Do 1 lan luc mo camera: chinh exposure
// toi khi do sang san (trung vi nua duoi anh) ~ EXPOSURE_TARGET roi giu co
// dinh. Mo lai camera (rut cap) thi dung lai gia tri da khoa.
// ============================================================================

double CameraLane::floor_brightness() {
  cv::Mat raw;
  cv::Mat bgr;
  bool ok = false;
  // Bo vai frame cho exposure moi co hieu luc
  for (int i = 0; i < 4; ++i) {
    ok = grab_frame(raw, bgr) && !bgr.empty();
  }
  if (!ok) {
    return -1.0;
  }
  cv::Mat gray;
  cv::cvtColor(bgr.rowRange(bgr.rows / 2, bgr.rows), gray, cv::COLOR_BGR2GRAY);
  int hist[256] = {0};
  for (int y = 0; y < gray.rows; ++y) {
    const uchar *g = gray.ptr<uchar>(y);
    for (int x = 0; x < gray.cols; ++x) {
      ++hist[g[x]];
    }
  }
  const long half = static_cast<long>(gray.total()) / 2;
  long acc = 0;
  for (int v = 0; v < 256; ++v) {
    acc += hist[v];
    if (acc >= half) {
      return v;
    }
  }
  return 255.0;
}

void CameraLane::lock_exposure() {
  cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 1); // V4L2: 1 = phoi sang tay
  int e = exposure_ > 0 ? exposure_ : locked_exposure_;
  if (e > 0) {
    cap_.set(cv::CAP_PROP_EXPOSURE, e);
    locked_exposure_ = e;
    return;
  }
  e = EXPOSURE_START;
  double b = -1.0;
  for (int it = 0; it < 8; ++it) {
    cap_.set(cv::CAP_PROP_EXPOSURE, e);
    b = floor_brightness();
    if (b < 0.0 || std::fabs(b - EXPOSURE_TARGET) <= 8.0) {
      break;
    }
    // Do sang tang cham hon tuyen tinh theo exposure
    const double ratio =
        std::clamp(std::pow(EXPOSURE_TARGET / std::max(b, 5.0), 1.5), 0.33, 3.0);
    const int ne = std::clamp(static_cast<int>(std::lround(e * ratio)), EXPOSURE_MIN,
                              EXPOSURE_MAX);
    if (ne == e) {
      break;
    }
    e = ne;
  }
  locked_exposure_ = e;
  std::cout << "[CameraLane] Khoa phoi sang: exposure=" << e << " (do sang san " << b
            << ", muc tieu " << EXPOSURE_TARGET << ")\n";
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

bool CameraLane::process(const cv::Mat &bgr, LaneOutput &out, bool draw,
                         std::chrono::steady_clock::time_point stamp) {
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
  return detect(bgr, out, draw, stamp);
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
  out.kink_dist_m = latest_.kink_dist_m;
  out.kink_dir = latest_.kink_dir;
  out.kink_deg = latest_.kink_deg;
  out.pitch_deg = latest_.pitch_deg;
  out.pitch_confirmed = latest_.pitch_confirmed;
  out.near_z_m = latest_.near_z_m;
  out.dark_ratio = latest_.dark_ratio;
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
