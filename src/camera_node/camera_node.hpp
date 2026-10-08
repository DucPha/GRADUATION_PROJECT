#pragma once

#include <opencv2/opencv.hpp>

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// ============================================================================
// CAMERA LANE - bam vach lan MOI HUONG tren mat dat (khong warp IPM)
// ----------------------------------------------------------------------------
// Camera doc MJPG (mac dinh 1920x1080), tu giai ma bang libjpeg-turbo o 1/2
// kich thuoc. Detector cat ROI roi thu ve WORK_W = 320 cot; chieu cao khung lam
// viec GIU DUNG TI LE anh goc (1920x1080 -> 320x180) nen tieu cu doc = ngang.
//
// ROI la HINH CHU NHAT PHU HET BE NGANG anh (khong con hinh thang): xe lech
// sat 1 vach hay vao cua gat thi vach van nam trong vung xu ly.
//
// Quy trinh moi frame:
//   1. cat ROI -> xam -> blur
//   2. TRU NEN chong loa: nen = dong hinh thai hoc bang kernel VUONG-ish (lap
//      ca vach doc, cheo lan NGANG), diff = nen - anh
//   3. BU LOA: vung nen sang (den tran) bi camera nen tuong phan -> nhan diff
//      voi he so theo do sang nen, roi Otsu + nguong tre (hysteresis)
//   4. Thanh phan lien thong -> doi tung pixel sang toa do MAT DAT (met) ->
//      PCA -> chia doan doc truc chinh -> duong tam vach + be day vach.
//      Khong phu thuoc huong: vach doc, cheo, nam ngang (cua gat) deu bam duoc.
//   5. Noi cac doan vach bi dut (loa, mon bang keo) neu thang hang tren mat dat
//   6. Gan nhan TRAI/PHAI (uu tien khop vach frame truoc, khong thi xet xe
//      nam ben nao cua vach) -> chon cap gan xe nhat, kiem tra be rong lan
//   7. Duong tam lan = vach doi vao trong nua be rong lan theo PHAP TUYEN
//      (dung ca khi vach nam ngang truoc mui xe)
//   8. Pure pursuit: diem tren duong tam cach xe PURSUIT_L_M -> do cong ->
//      quy ve do lech tuong duong tai DEV_REF_DIST_M (giu nguyen thang do
//      firmware) + khau keo ve giua lan theo sai lech ngang gan xe
//   9. Chan nhay -> EMA -> he so toc do theo do cong.
//
// Toa do mat dat: X (m) sang PHAI, Z (m) ve PHIA TRUOC, goc = chan camera.
//   X = (x - cx) * h / (cos(pitch) * (y - horizon_y))
//   Z = h / tan(pitch + atan((y - h_img/2) / f))
//
// QUY UOC dev_px: luon tinh theo anh THAM CHIEU rong DEV_REF_W = 640 px tai
// khoang cach DEV_REF_DIST_M, bat ke camera chay do phan giai nao. Firmware
// (deadzone, bao hoa) duoc chinh theo thang do nay.
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
  // Quyet dinh moi phep doi pixel <-> met. Cach chinh: dat xe giua 2 vach
  // THANG, duong tim tren overlay phai di qua diem 2 vach keo dai gap nhau.
  // < 0: tinh tu horizon_y/axis_ground_m.
  float horizon_frac = 0.20f;

  // Hang chan troi tren khung lam viec, chi dung khi horizon_frac < 0.
  // -9999 = tinh tu axis_ground_m.
  int horizon_y = -9999;

  // ROI theo ti le chieu cao anh, tinh tu tren xuong. Nho hon = thay xa hon.
  float roi_top_frac = 0.48f;
  float roi_bottom_frac = 0.95f;
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

  // Anh quan sat gui cho GUI duoc thu ve be ngang nay
  static constexpr int VIS_W = 640;

  // ROI it nhat bay nhieu hang
  static constexpr int ROI_MIN_ROWS = 16;

  // Dinh ROI khong duoc cao hon chan troi + ROI_MIN_DY hang
  static constexpr int ROI_MIN_DY = 10;

  // ------------------------------------------------------------------------
  // Tien xu ly anh
  // ------------------------------------------------------------------------

  static constexpr int BLUR_KSIZE = 5;

  // Chenh lech toi thieu giua nhom "vach" va nhom "nen" (Otsu tren diff)
  static constexpr double MIN_CONTRAST = 12.0;

  // TRU NEN: kernel dong phai lon hon BE DAY vach theo MOI huong. Vach doc
  // sat xe rong toi ~40 px, vach NGANG (cua gat) day toi ~20 hang -> 45x25.
  // Kernel cu 61x5 chi lap duoc vach doc: vach nam ngang day hon 5 hang bi
  // coi la nen -> mat sach o anh nhi phan dung luc vao cua gat.
  static constexpr int BG_KERNEL_W = 45;
  static constexpr int BG_KERNEL_H = 25;

  // Nguong manh toi thieu / nguong yeu cua hysteresis (muc xam cua diff)
  static constexpr int BG_MIN_DIFF = 12;
  static constexpr int BG_WEAK_DIFF = 7;

  // BU LOA: diff duoc nhan he so (255 - nen_trung_vi) / (255 - nen), kep
  // [1, GLARE_GAIN_MAX]. Vung den tran rua sang (nen ~240) con diff 5-10 thi
  // duoc keo len ngang vach ngoai vung loa.
  static constexpr float GLARE_GAIN_MAX = 2.5f;

  // So dai ngang khi khoet khoi day (ban kinh doi theo phoi canh)
  static constexpr int THICK_BANDS = 4;
  static constexpr bool THICK_REMOVE = true;

  // LOC MAU: vach la cao su / bang keo DEN (bao hoa mau thap). Pixel du sang
  // (max kenh >= COLOR_MIN_V) ma bao hoa > COLOR_MAX_SAT (0..255) -> khong
  // phai vach (ghe cam, vat mau). Do tren anh that: cao su S 40-70, ghe cam
  // S 130-150, mat ban S < 10.
  static constexpr int COLOR_MAX_SAT = 100;
  static constexpr int COLOR_MIN_V = 60;

  // Thanh phan lien thong nho hon so pixel nay la nhieu
  static constexpr int MIN_BLOB_PX = 10;

  // ------------------------------------------------------------------------
  // Bam vach tren mat dat (met)
  // ------------------------------------------------------------------------

  // Buoc chia doan doc truc vach
  static constexpr float BIN_M = 0.03f;

  // Be day toi da 1 vach (doan nghieng/cong duoc cong them sai so luong tu)
  static constexpr float TAPE_MAX_M = 0.12f;

  // Tiep tuyen tai 1 diem lay tren cung +/- bay nhieu met (phap tuyen de doi
  // vach ra duong tam)
  static constexpr float TANGENT_HALF_M = 0.06f;

  // Doan vach ngan hon muc nay bi bo; vach (sau khi noi) phai dai hon
  // MIN_LINE_LEN_M moi dung de lai
  static constexpr float MIN_SEG_LEN_M = 0.06f;
  static constexpr float MIN_LINE_LEN_M = 0.15f;

  // Noi doan vach dut: khoang ho toi da, goc lech huong toi da (do)
  static constexpr float LINK_GAP_M = 0.35f;
  static constexpr float LINK_ANGLE_DEG = 40.0f;

  // Be rong lan chap nhan / mac dinh (tu hoc lai khi du 2 vach)
  static constexpr float LANE_W_MIN_M = 0.35f;
  static constexpr float LANE_W_MAX_M = 0.85f;
  static constexpr float LANE_W_DEFAULT_M = 0.55f;
  static constexpr float LANE_W_ALPHA = 0.2f;

  // Vach frame truoc: khop neu trung vi khoang cach < PRIOR_MATCH_M
  static constexpr float PRIOR_MATCH_M = 0.12f;

  // Xet xe nam ben nao cua vach tren doan dai bay nhieu tinh tu dau gan xe
  static constexpr float NEAR_SIDE_M = 0.25f;
  static constexpr int PRIOR_MAX_AGE_MS = 300;

  // ------------------------------------------------------------------------
  // Lai (pure pursuit) -> dev_px
  // ------------------------------------------------------------------------

  // Khoang nhin truoc cua pure pursuit (m tinh tu chan camera)
  static constexpr float PURSUIT_L_M = 0.70f;

  // dev_px duoc quy ve do lech ngang tai khoang cach nay (thang do firmware
  // da chinh: 1 px anh 640 ~ 1.5 mm tai 0.65 m)
  static constexpr float DEV_REF_DIST_M = 0.65f;

  // Khau keo ve giua lan: cong them LAT_GAIN * (lech ngang cua tam lan o
  // gan xe). Tang -> xe ve giua nhanh hon; qua lon -> lac.
  static constexpr float LAT_GAIN = 0.4f;

  // Gioi han dev gui xuong (px anh 640)
  static constexpr int DEV_MAX_REF_PX = 200;

  // Chan nhay: dev moi lech dev dang loc qua JUMP_GATE_REF_PX thi giu gia tri
  // cu; lech nhu vay JUMP_CONFIRM_FRAMES frame lien tiep thi moi tin.
  static constexpr int JUMP_GATE_REF_PX = 90;
  static constexpr int JUMP_CONFIRM_FRAMES = 2;

  // EMA cua dev: lon = nhay. 2 vach (tin cay) loc nhe, 1 vach loc vua.
  static constexpr float EMA_ALPHA_TWO = 0.70f;
  static constexpr float EMA_ALPHA_ONE = 0.50f;

  // ------------------------------------------------------------------------
  // He so toc do de nghi (speed_scale: 1 = duong thang, 0 = cua gat nhat)
  // ------------------------------------------------------------------------

  static constexpr float CURVE_HEADING_START_DEG = 15.0f;
  static constexpr float CURVE_HEADING_FULL_DEG = 50.0f;
  // Huong dau gan / dau xa cua duong tam do tren day cung dai bay nhieu
  static constexpr float CURVE_CHORD_M = 0.20f;
  static constexpr float CURVE_K_START = 0.40f; // 1/m (R 2.5 m)
  static constexpr float CURVE_K_FULL = 1.50f;  // 1/m (R 0.67 m)

  // ------------------------------------------------------------------------
  // Camera
  // ------------------------------------------------------------------------

  static constexpr int STALE_AGE_MS = 200;
  static constexpr int MAX_READ_FAIL = 25;
  static constexpr int REOPEN_PERIOD_MS = 2000;

  // Mat lan: giu dev_px cuoi toi da bay nhieu ms
  static constexpr int HOLD_MS = 500;

  // ------------------------------------------------------------------------
  // Ket qua mot lan nhan
  // ------------------------------------------------------------------------

  struct LaneOutput {
    // Duong tam 2 vach da chon, toa do khung lam viec (gan -> xa)
    std::vector<cv::Point> left_pts;
    std::vector<cv::Point> right_pts;

    bool two_lanes = false; // true chi khi state == TWO_LINES
    LaneState state = LaneState::LOST;

    // Lenh lai, px anh THAM CHIEU 640 (xem dau file). > 0: lai phai.
    // Khi LOST: giu gia tri cuoi toi da HOLD_MS.
    int dev_px = 0;

    // Do lech ngang tuong duong (cm), chi de hien thi
    float dev_cm = 0.0f;
    float lane_width_cm = 0.0f;

    // Do cong duong tam (1/m). > 0: re phai, < 0: re trai.
    float curvature = 0.0f;

    // Lech ngang (cm) cua duong tam doan xa so voi doan gan
    float path_dx_cm = 0.0f;

    // 1 = duong thang, 0 = cua gat nhat. Node dieu khien noi suy toc do
    // giua speed_corner va speed theo he so nay.
    float speed_scale = 1.0f;

    bool fit_ok = false;

    // Toa do MAT DAT (m, goc = chan camera, X phai, Z truoc), gan -> xa.
    // PathTracker dung de nho duong qua vung mu truoc xe. Rong = khong thay.
    std::vector<cv::Point2f> centre_g; // duong tam lan, lay mau 5 cm
    std::vector<cv::Point2f> left_g;   // vach trai
    std::vector<cv::Point2f> right_g;  // vach phai
    std::chrono::steady_clock::time_point stamp{}; // luc nhan frame

    // True khi frame nay bi chan nhay (dev giu gia tri cu)
    bool gated = false;

    float horizon_frac = 0.0f; // chan troi dang dung (ti le chieu cao anh)

    unsigned long age_ms = 0;
    bool stale = true;

    // Kich thuoc anh goc camera
    int frame_w = 0;
    int frame_h = 0;

    // Anh cho GUI, rong VIS_W. Chi co khi get_latest(copy_vis = true).
    cv::Mat vis; // anh camera + overlay detector
    cv::Mat raw; // anh camera chua ve
    cv::Mat roi; // vung ROI cua anh camera
    cv::Mat bin; // mask nhi phan 0/1, WORK_W x roi_h
    long vis_frame_id = 0;

    double proc_ms = 0.0;
    long frame_id = 0;
  };

  // ------------------------------------------------------------------------
  // Vong doi
  // ------------------------------------------------------------------------

  // camera_index < 0 -> tu do 0..3. width/height = do phan giai yeu cau.
  // start() tra false neu chua mo duoc camera ngay, nhung luong camera van
  // chay va tu thu mo lai moi REOPEN_PERIOD_MS.
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
  // bao cho luong camera ve anh quan sat o frame ke tiep.
  void get_latest(LaneOutput &out, bool copy_vis = false) const;

  void set_roi_top_frac(float frac);

  // Xu ly 1 anh BGR co san (khong can camera). Goi tuan tu tu 1 luong,
  // KHONG goi khi start() dang chay.
  bool process(const cv::Mat &bgr, LaneOutput &out, bool draw = true);

  // Phoi sang tay (V4L2 exposure_time_absolute = 100 us). <= 0 = tu dong.
  // Goi TRUOC start().
  void set_manual_exposure(int value) { exposure_ = value; }

private:
  // Mot vach tren mat dat: chuoi diem tam vach, sap xep GAN XE -> XA XE
  struct LinePath {
    std::vector<cv::Point2f> g; // toa do mat dat (m)
    std::vector<cv::Point> img; // toa do khung lam viec, cung thu tu
    float length_m = 0.0f;
    float thick_m = 0.0f;
    int side = 0; // -1 trai, +1 phai, 0 chua biet
    bool from_prior = false;
  };

  bool open_camera();
  void capture_loop();

  // Doc 1 khung va dua ve BGR (tu giai ma MJPG neu driver tra goi nen)
  bool grab_frame(cv::Mat &raw, cv::Mat &bgr);

  // Tinh lai horizon, bang tra hang -> met khi biet kich thuoc anh
  void update_geometry(int frame_w, int frame_h);

  bool detect(const cv::Mat &frame, LaneOutput &out, bool draw);

  // Buoc 2-3: anh xam ROI co le (gray_ext) -> mask nhi phan 0/1 cua ROI
  // (rows hang, bat dau tu hang row0 cua gray_ext)
  void build_mask(const cv::Mat &gray_ext, int row0, int rows, int top,
                  cv::Mat &bin, double &contrast) const;

  // Khoet khoi toi DAY hon vach (ghe, vat can, bong) dinh vao vach
  void remove_thick(cv::Mat &bin, int top) const;

  // Buoc 4: thanh phan lien thong -> doan vach tren mat dat
  void extract_segments(const cv::Mat &bin, int top,
                        std::vector<LinePath> &segs) const;

  // Buoc 5: noi doan vach dut thanh vach dai
  void link_segments(std::vector<LinePath> &segs,
                     std::vector<LinePath> &lines) const;

  // Xe nam ben nao cua vach (xet doan gan xe), < 0: xe ben phai vach
  float car_side(const LinePath &line) const;

  // Buoc 6: nhan trai/phai
  int classify_side(LinePath &line, bool prior_ok) const;

  static int otsu_hist(const int hist[256], double &contrast);

  // ---- Hinh hoc mat dat ----
  bool ground_to_img(const cv::Point2f &g, cv::Point2d &p) const;
  double ground_dist_m(double y) const;

  static float dist_to_polyline(const cv::Point2f &p,
                                const std::vector<cv::Point2f> &poly,
                                bool *interior = nullptr);
  static float median_dist(const std::vector<cv::Point2f> &pts,
                           const std::vector<cv::Point2f> &poly,
                           bool interior_only);
  static cv::Point2f tangent_at(const std::vector<cv::Point2f> &p, size_t i);
  void offset_path(const LinePath &line, float offset_m,
                   std::vector<cv::Point2f> &out) const;
  static std::vector<cv::Point2f> resample(const std::vector<cv::Point2f> &p,
                                           float step);
  static bool pursuit_point(const std::vector<cv::Point2f> &path, float L,
                            cv::Point2f &pt);

  // ------------------------------------------------------------------------
  // Trang thai
  // ------------------------------------------------------------------------

  int camera_index_;
  int target_fps_;
  int req_w_;
  int req_h_;
  CameraProfile profile_;
  int exposure_ = -1;

  bool raw_mjpg_ = false;
  bool quiet_open_ = false;
  int cam_w_ = 0; // do phan giai that cua camera
  int cam_h_ = 0;

  // Hinh hoc suy ra tu CameraProfile + kich thuoc anh
  int work_h_ = 180;
  int geom_w_ = 0;
  int geom_h_ = 0;
  float h_ = 0.30f;     // do cao cam (m)
  double pitch_ = 0.0;  // goc cui cua truc quang (rad)
  double f_px_ = 0.0;   // tieu cu tren khung lam viec (px)
  float k_ = 0.30f;     // h / cos(pitch): X = (x - cx) * k_ / dy
  int horizon_y_ = 90;
  float roi_bottom_frac_ = 0.95f;
  double dev_m_per_px_ = 0.0015; // met / px anh 640 tai DEV_REF_DIST_M
  bool logged_geometry_ = false;

  // Bang tra theo hang khung lam viec: Z (m) va met/px ngang. < 0: khong hop le
  std::vector<float> row_z_;
  std::vector<float> row_s_;

  std::atomic<float> roi_top_frac_{0.48f};

  cv::VideoCapture cap_;

  std::atomic<bool> running_{false};
  std::thread worker_;

  mutable std::atomic<bool> vis_wanted_{true};

  mutable std::mutex mtx_;
  LaneOutput latest_;     // ket qua moi nhat (khong kem anh)
  LaneOutput latest_vis_; // bo anh quan sat moi nhat

  // ---- Cac bien duoi day chi dung trong luong camera ----
  float dev_ema_ = 0.0f;
  bool ema_primed_ = false;
  int jump_count_ = 0;
  float lane_w_m_ = LANE_W_DEFAULT_M;
  std::chrono::steady_clock::time_point last_valid_time_{};

  // Vach trai/phai cua frame truoc (mat dat)
  std::vector<cv::Point2f> prior_left_;
  std::vector<cv::Point2f> prior_right_;
  std::chrono::steady_clock::time_point prior_time_{};

  // Diem tham chieu xe tren mat dat (0, Z hang day ROI) - de chon vach gan xe
  cv::Point2f car_ref_{0.0f, 0.5f};

  long frame_id_ = 0;

  std::chrono::steady_clock::time_point last_frame_time_{};
};
