#pragma once

#include <opencv2/opencv.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// ============================================================================
// CAMERA LANE - bam lan bang cua so truot, khong dung IPM
// ----------------------------------------------------------------------------
// Camera doc MJPG (mac dinh 1920x1080). Khung MJPG duoc tu giai ma bang
// libjpeg-turbo o 1/2 kich thuoc (1920x1080 -> 960x540, ~7 ms thay vi ~15 ms)
// vi detector chi can 320 cot; nho vay camera giu duoc 25-30 fps va log khong
// bi ngap canh bao "Corrupt JPEG data" cua cam nay. Detector chi cat vung ROI roi thu ve
// WORK_W = 320 cot. Chieu cao khung lam viec GIU DUNG TI LE anh goc
// (1920x1080 -> 320x180, 640x480 -> 320x240) nen tieu cu doc = tieu cu ngang
// va cong thuc pinhole ben duoi dung cho moi do phan giai.
//
// Quy trinh moi frame:
//   cat ROI -> xam -> blur -> TRU NEN chong loa (nen = dong hinh thai hoc)
//   -> Otsu tren anh "nen - anh" + nguong tre (hysteresis) trong hinh thang
//   -> morphology -> phuc hoi doan vach bi ngat (den tran rua sang)
//   -> cua so truot: 3 cach tim (theo frame truoc / xa->gan / gan->xa),
//      chon cach co diem cao nhat
//   -> diem tam tung cua so (2 vach: trung diem, 1 vach: vach +/- nua lan)
//   -> fit tam lan bac 2, loai diem lech (outlier) roi fit lai
//   -> do lech tai hang nhin truoc -> chan nhay dot ngot -> EMA.
//
// Doi pixel sang met bang cong thuc pinhole:
//   W = w_px * h / (cos(pitch) * (y - horizon_y))
//
// QUY UOC dev_px: luon tinh theo anh THAM CHIEU rong DEV_REF_W = 640 px, bat
// ke camera chay 640x480 hay 1920x1080. Firmware (deadzone 10 px, bao hoa
// 50 px) duoc chinh theo 640 px; neu nhan theo anh goc 1920 thi dev lon gap
// 3 lan -> danh lai qua tay, xe lac.
// ============================================================================

enum class LaneState {
  LOST = 0,     // khong thay vach nao dung duoc
  ONE_LINE = 1, // chi thay 1 vach, tam lan suy ra tu be rong lan da hoc
  TWO_LINES = 2 // thay du 2 vach
};

// ----------------------------------------------------------------------------
// Thong so lap dat camera. Doi cam chi can doi struct nay, khong sua .cpp.
// ----------------------------------------------------------------------------
struct CameraProfile {
  // Do cao tam cam so voi mat duong (m)
  float height_m = 0.30f;

  // Khoang cach NGANG (m) tu chan cam toi diem truc quang cham mat dat.
  // 0 = truc quang nam ngang (chan troi o giua anh).
  float axis_ground_m = 0.0f;

  // Goc nhin ngang (do), theo thong so cam
  float hfov_deg = 70.0f;

  // VI TRI CHAN TROI = ti le chieu cao anh (0 = mep tren, 1 = mep duoi).
  // Quyet dinh moi phep doi pixel <-> met (be rong lan, be rong vach). Sai
  // chan troi -> be rong lan tinh sai -> ghep nham cap vach.
  // Cach chinh: dat xe giua 2 vach THANG, nhin anh overlay: duong tim phai
  // di qua diem 2 vach keo dai gap nhau. Cam tren xe cui xuong ~13 do ->
  // ~0.20 (do tu anh that ngay 07/10). < 0: tinh tu horizon_y/axis_ground_m.
  float horizon_frac = 0.20f;

  // Hang chan troi tren khung lam viec (320 x work_h), chi dung khi
  // horizon_frac < 0. -9999 = tinh tu axis_ground_m.
  int horizon_y = -9999;

  // ROI theo ti le chieu cao anh, tinh tu tren xuong. Nho hon = thay xa hon.
  float roi_top_frac = 0.52f;
  float roi_bottom_frac = 0.93f;
};

class CameraLane {

public:
  // ------------------------------------------------------------------------
  // Kich thuoc xu ly
  // ------------------------------------------------------------------------

  // Be ngang khung lam viec. Chieu cao = WORK_W * frame_h / frame_w.
  static constexpr int WORK_W = 320;

  // Anh tham chieu cho dev_px (xem dau file)
  static constexpr int DEV_REF_W = 640;

  // Anh quan sat gui cho GUI duoc thu ve be ngang nay (nhe, encode nhanh)
  static constexpr int VIS_W = 640;

  // ------------------------------------------------------------------------
  // Cua so truot
  // ------------------------------------------------------------------------

  static constexpr int N_WINDOWS = 6;

  // So cua so (tinh tu cua so dau tien cua luot quet) duoc phep de tim cap
  // vach dau tien
  static constexpr int MAX_SEED_SEARCH = 4;

  // Can it nhat bao nhieu cua so thay CA 2 vach moi tinh la TWO_LINES
  static constexpr int MIN_MATCHED_WINDOWS = 3;

  // Can it nhat bao nhieu diem tam moi tinh la ONE_LINE
  static constexpr int MIN_ONE_SIDE_WINDOWS = 3;

  // Ban cot tim vach quanh vi tri du doan (px, trong anh 320 cot)
  static constexpr int WINDOW_MARGIN = 30;

  // Khi da co vi tri vach cua frame truoc: chi tim trong ban hep hon, vach
  // khong the nhay xa trong 33 ms -> khong bi hut sang bong do / vat la.
  static constexpr int PRIOR_MARGIN = 18;

  // Vi tri vach cua frame truoc chi dung lam goi y neu con moi hon (ms)
  static constexpr int PRIOR_MAX_AGE_MS = 250;

  // So pixel vach toi thieu trong 1 cot cua cua so
  static constexpr int WINDOW_MIN_POINTS = 2;

  // ------------------------------------------------------------------------
  // Kich thuoc that (met) - doi ra pixel theo tung hang
  // ------------------------------------------------------------------------

  // Be rong toi da 1 vach. 12 cm: bang keo ban rong + sai so chieu cao cam
  // (6 cm cu loai mat bang keo trai ban rong tren xe that).
  static constexpr float TAPE_MAX_M = 0.12f;
  static constexpr float MIN_LANE_GAP_M = 0.30f; // 2 vach cach nhau it nhat
  static constexpr float LANE_W_MIN_M = 0.40f;   // be rong lan chap nhan
  static constexpr float LANE_W_MAX_M = 0.80f;
  static constexpr float LANE_W_DEFAULT_M = 0.55f; // tu hoc lai khi du 2 vach
  static constexpr float LANE_W_ALPHA = 0.2f;

  // ------------------------------------------------------------------------
  // Tien xu ly anh
  // ------------------------------------------------------------------------

  static constexpr int BLUR_KSIZE = 5;

  // Chenh lech toi thieu giua nhom "vach" va nhom "nen" (Otsu tren anh
  // diff = nen - anh). Duoi muc nay coi nhu khong co vach.
  static constexpr double MIN_CONTRAST = 15.0;

  // TRU NEN CHONG LOA (xem buoc 4 trong detect()). Cua so uoc luong nen phai
  // RONG HON be ngang vach lon nhat tren anh 320 cot (bang keo ban rong nam
  // nghieng sat xe ~50 px) nhung HEP HON vat toi lon can loai (bong, vat can).
  static constexpr int BG_KERNEL_W = 61;
  static constexpr int BG_KERNEL_H = 5;
  // Pixel phai toi hon nen it nhat bay nhieu muc xam moi la vach (chan nhieu
  // khi khong co vach, Otsu chia doi nhieu)
  static constexpr int BG_MIN_DIFF = 12;
  // Nguong YEU cho hysteresis: doan vach bi loa con diff tren muc nay ma noi
  // lien voi doan vach ro thi van giu
  static constexpr int BG_WEAK_DIFF = 8;

  // Noi doan vach bi ngat toi da bay nhieu cua so lien tiep
  static constexpr int BRIDGE_MAX_BANDS = 3;

  // ------------------------------------------------------------------------
  // Hinh hoc
  // ------------------------------------------------------------------------

  // Dinh ROI khong duoc cao hon chan troi + ROI_MIN_DY hang
  static constexpr int ROI_MIN_DY = 12;

  // Day ROI tu keo len de 1 lan LANE_W_FIT_M + 2 le van nam tron be ngang
  static constexpr float LANE_W_FIT_M = 0.61f;
  static constexpr float ROI_SIDE_MARGIN_M = 0.10f;

  // Hang lay do lech, tinh tu dinh ROI xuong
  static constexpr float LOOKAHEAD_FRAC = 0.58f;

  // Nua do rong dinh hinh thang theo WORK_W
  static constexpr float ROI_TOP_HALF_FRAC = 0.30f;

  // ------------------------------------------------------------------------
  // On dinh hoa ket qua
  // ------------------------------------------------------------------------

  // Diem tam lech khoi duong fit qua nguong nay (px khung lam viec) bi loai
  // roi fit lai (toi da 2 diem moi frame)
  static constexpr double CENTRE_OUTLIER_PX = 7.0;

  // Chan nhay: dev moi lech dev dang loc qua JUMP_GATE_REF_PX (px anh tham
  // chieu 640) thi giu gia tri cu; neu lech nhu vay JUMP_CONFIRM_FRAMES
  // frame lien tiep thi moi tin (vao cua that).
  static constexpr int JUMP_GATE_REF_PX = 60;
  static constexpr int JUMP_CONFIRM_FRAMES = 2;

  static constexpr float EMA_ALPHA = 0.40f;

  // Tuoi frame vuot qua gia tri nay -> coi nhu mat camera
  static constexpr int STALE_AGE_MS = 200;

  static constexpr int MAX_READ_FAIL = 25;

  // Mat camera / chua mo duoc: thu mo lai sau moi khoang nay (ms)
  static constexpr int REOPEN_PERIOD_MS = 2000;

  // Mat lan: giu dev_px cuoi toi da bay nhieu ms
  static constexpr int HOLD_MS = 500;

  // ------------------------------------------------------------------------
  // He so toc do de nghi (speed_scale, 0..1)
  // ------------------------------------------------------------------------

  static constexpr float PATH_DX_START_CM = 10.0f;
  static constexpr float PATH_DX_FULL_CM = 35.0f;
  static constexpr float CURV_START = 0.15f; // R 6.7 m
  static constexpr float CURV_FULL = 0.60f;  // R 1.7 m
  static constexpr float SPEED_MIN_CURVE = 0.45f;
  static constexpr float SPEED_ONE_LINE = 0.55f;
  static constexpr float SPEED_LOST = 0.30f;

  // ------------------------------------------------------------------------
  // Ket qua mot lan nhan
  // ------------------------------------------------------------------------

  struct LaneOutput {
    // Diem 2 vach (chi cua so thay du 2 vach), toa do khung lam viec
    std::vector<cv::Point> left_pts;
    std::vector<cv::Point> right_pts;

    bool two_lanes = false; // true chi khi state == TWO_LINES
    LaneState state = LaneState::LOST;

    // Do lech tam lan so voi giua anh tai hang nhin truoc, px anh THAM
    // CHIEU 640 (xem dau file). Khi LOST: giu gia tri cuoi toi da HOLD_MS.
    int dev_px = 0;

    // Cung thong tin theo cm, chi de hien thi
    float dev_cm = 0.0f;
    float lane_width_cm = 0.0f;

    // Do cong duong tam (1/m). > 0: re phai, < 0: re trai.
    float curvature = 0.0f;

    // Lech ngang (cm) cua duong tam doan xa so voi doan gan
    float path_dx_cm = 0.0f;

    // 0..1: nhan voi toc do dat de vao cua cham lai
    float speed_scale = 1.0f;

    bool fit_ok = false;

    // True khi frame nay bi chan nhay (dev giu gia tri cu)
    bool gated = false;

    float horizon_frac = 0.0f; // chan troi dang dung (ti le chieu cao anh)

    // Tuoi frame moi nhat (ms), -1 neu chua co frame nao
    unsigned long age_ms = 0;
    bool stale = true;

    // Kich thuoc anh goc camera
    int frame_w = 0;
    int frame_h = 0;

    // Anh cho GUI, rong VIS_W. Chi co khi get_latest(copy_vis = true).
    cv::Mat vis; // anh camera + overlay detector
    cv::Mat raw; // anh camera chua ve
    cv::Mat roi; // vung ROI cua anh camera
    cv::Mat bin; // mask nhi phan 0/1 sau morphology, WORK_W x roi_h
    long vis_frame_id = 0;

    double proc_ms = 0.0;
    long frame_id = 0;
  };

  // ------------------------------------------------------------------------
  // Vong doi
  // ------------------------------------------------------------------------

  // camera_index < 0 -> tu do 0..3. width/height = do phan giai yeu cau.
  // start() tra false neu chua mo duoc camera ngay, nhung luong camera van
  // chay va tu thu mo lai moi REOPEN_PERIOD_MS (cam lai camera la co hinh).
  explicit CameraLane(int camera_index = -1, int target_fps = 30,
                      int width = 1920, int height = 1080,
                      const CameraProfile &profile = CameraProfile{});

  ~CameraLane();

  CameraLane(const CameraLane &) = delete;
  CameraLane &operator=(const CameraLane &) = delete;

  bool start();
  void stop();
  bool is_running() const;

  // copy_vis = false: vong dieu khien khong copy anh. copy_vis = true con
  // bao cho luong camera ve anh quan sat o frame ke tiep (chi ve khi co
  // nguoi xem -> tiet kiem CPU).
  void get_latest(LaneOutput &out, bool copy_vis = false) const;

  void set_roi_top_frac(float frac);

  // Xu ly 1 anh BGR co san (khong can camera): dung de thu/chinh detector
  // voi anh chup san. Goi tuan tu tu 1 luong, KHONG goi khi start() dang chay.
  bool process(const cv::Mat &bgr, LaneOutput &out, bool draw = true);

  // Phoi sang tay (don vi V4L2 exposure_time_absolute = 100 us). <= 0 = tu
  // dong. Che do tu dong ha fps khi thieu sang (cam nay: ~25 fps trong
  // phong); dat vd 250 (25 ms) de giu 30 fps. Goi TRUOC start().
  void set_manual_exposure(int value) { exposure_ = value; }

private:
  bool open_camera();
  void capture_loop();

  // Doc 1 khung va dua ve BGR (tu giai ma MJPG neu driver tra goi nen)
  bool grab_frame(cv::Mat &raw, cv::Mat &bgr);

  // Tinh lai horizon, chieu cao khung lam viec khi biet kich thuoc anh
  void update_geometry(int frame_w, int frame_h);

  bool detect(const cv::Mat &frame, LaneOutput &out, bool draw);

  bool seed_pair_from(const cv::Mat &col_sum, int dy, int win_h, int &seed_left,
                      int &seed_right) const;

  static bool peak_in(const cv::Mat &col_sum, int seed, int margin,
                      int max_run, int &peak);

  int run_limit_px(int x, int dy, int win_h) const;

  int lane_px(float meters, int dy) const;

  static int otsu_masked(const cv::Mat &gray, const cv::Mat &mask,
                         double &contrast);

  static bool fit_quad(const std::vector<double> &t,
                       const std::vector<double> &x, double coef[3]);

  static bool fit_poly2(const std::vector<cv::Point> &pts, double coef[3],
                        int &y0, int &y1);

  // Fit roi loai toi da 2 diem lech > CENTRE_OUTLIER_PX, fit lai
  static bool fit_poly2_robust(std::vector<cv::Point> &pts, double coef[3],
                               int &y0, int &y1);

  float px_to_cm(float px, int y) const;

  double ground_dist_m(double y) const;

  // ------------------------------------------------------------------------
  // Trang thai
  // ------------------------------------------------------------------------

  int camera_index_;
  int target_fps_;
  int req_w_;
  int req_h_;
  CameraProfile profile_;
  int exposure_ = -1;

  // true: driver tra goi MJPG nguyen (CAP_PROP_CONVERT_RGB = 0), node tu
  // giai ma. false: driver tu giai ma (camera khong ho tro MJPG).
  bool raw_mjpg_ = false;
  bool quiet_open_ = false; // da bao "No camera found", chua in lai
  int cam_w_ = 0; // do phan giai that cua camera (truoc khi giai ma 1/2)
  int cam_h_ = 0;

  // Hinh hoc suy ra tu CameraProfile + kich thuoc anh
  int work_h_ = 180;    // chieu cao khung lam viec
  int geom_w_ = 0;      // kich thuoc anh goc da tinh hinh hoc
  int geom_h_ = 0;
  float h_ = 0.30f;     // do cao cam (m)
  double pitch_ = 0.0;  // goc cui cua truc quang (rad)
  double f_px_ = 0.0;   // tieu cu tren khung lam viec (px)
  float k_ = 0.30f;     // h / cos(pitch): met = px * k_ / dy
  int horizon_y_ = 90;  // hang chan troi (co the am)
  float roi_bottom_frac_ = 0.93f;
  bool logged_geometry_ = false;

  std::atomic<float> roi_top_frac_{0.52f};

  cv::VideoCapture cap_;

  std::atomic<bool> running_{false};
  std::thread worker_;

  // Luong dieu khien/GUI yeu cau anh quan sat cho frame ke tiep
  mutable std::atomic<bool> vis_wanted_{true};

  mutable std::mutex mtx_;
  LaneOutput latest_;     // ket qua moi nhat (khong kem anh)
  LaneOutput latest_vis_; // bo anh quan sat moi nhat (vis/raw/roi/bin)

  // ---- Cac bien duoi day chi dung trong luong camera ----
  int dev_ema_ = 0;
  bool ema_primed_ = false;
  int jump_count_ = 0;
  float lane_w_m_ = LANE_W_DEFAULT_M;
  bool warned_roi_ = false;
  std::chrono::steady_clock::time_point last_valid_time_{};

  // Vi tri vach tung cua so cua frame truoc (-1 = khong co)
  std::array<int, N_WINDOWS> prior_left_{};
  std::array<int, N_WINDOWS> prior_right_{};
  std::chrono::steady_clock::time_point prior_time_{};

  long frame_id_ = 0;

  std::chrono::steady_clock::time_point last_frame_time_{};
};
