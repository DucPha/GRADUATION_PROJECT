#pragma once

#include <opencv2/opencv.hpp>

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// ============================================================================
// CAMERA LANE - bam lan bang cua so truot, khong dung IPM
// ----------------------------------------------------------------------------
// Camera doc 640x480 (MJPG), chi cat vung ROI roi thu ve 320 cot de xu ly.
// Thu ve bang INTER_AREA = trung binh 2x2, giam nhieu ~2 lan khi thieu sang.
//
// Quy trinh: cat ROI -> xam -> blur -> CLAHE -> Otsu (chi tinh trong hinh
// thang)
//   -> morphology -> phuc hoi doan vach bi ngat (anh den cham rua sang)
//   -> cua so truot (xa -> gan, du doan vi tri theo phoi canh)
//   -> diem tam tung cua so (2 vach: trung diem, 1 vach: vach +/- nua lan)
//   -> fit tam lan -> do lech tai hang nhin truoc.
//
// Doi pixel sang met bang cong thuc pinhole:
//   W = w_px * h / (cos(pitch) * (y - horizon_y))
// h, pitch, horizon_y lay tu CameraProfile (xem ben duoi).
// dev_px van la pixel anh goc, giu nguyen quy uoc firmware dang dung.
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

  // Khoang cach NGANG (m) tu chan cam toi diem truc quang cham mat dat,
  // tinh den xe. Cam cui xuong ngam vao diem cach xe 0.50 m -> 0.50.
  // 0 = truc quang nam ngang (chan troi o giua anh, nhu cam cu).
  float axis_ground_m = 0.0f;

  // Goc nhin ngang (do), theo thong so cam
  float hfov_deg = 70.0f;

  // Hang chan troi do tren anh 320x240 (duong tim "horizon" tren anh vis).
  // -9999 = tu tinh tu 3 so tren. Do duoc thi nen dien, chinh xac hon tinh.
  int horizon_y = -9999;

  // ROI theo ti le chieu cao anh, tinh tu tren xuong. Nho hon = dinh ROI
  // cao hon, thay xa hon (lan rong, 2 vach xa nhau). Day ROI con bi gioi
  // han them boi LANE_W_FIT_M va ROI_MIN_DY, xem tren.
  float roi_top_frac = 0.52f;
  float roi_bottom_frac = 0.93f;
};

class CameraLane {

public:
  // ------------------------------------------------------------------------
  // Kich thuoc xu ly
  // ------------------------------------------------------------------------

  static constexpr int WORK_W = 320;
  static constexpr int WORK_H = 240;

  // ------------------------------------------------------------------------
  // Cua so truot
  // ------------------------------------------------------------------------

  static constexpr int N_WINDOWS = 6;

  // So cua so (tinh tu cua so xa nhat) duoc phep de tim cap vach dau tien
  static constexpr int MAX_SEED_SEARCH = 4;

  // Can it nhat bao nhieu cua so thay CA 2 vach moi tinh la TWO_LINES
  static constexpr int MIN_MATCHED_WINDOWS = 3;

  // Can it nhat bao nhieu diem tam moi tinh la ONE_LINE
  static constexpr int MIN_ONE_SIDE_WINDOWS = 3;

  // Ban cot tim vach quanh vi tri du doan (px, trong anh 320 cot).
  // Vi tri du doan da tinh phoi canh nen chi con bu cho do cong.
  static constexpr int WINDOW_MARGIN = 30;

  // So pixel vach toi thieu trong 1 cot cua cua so. Bang keo hep va vach
  // xa chi phu ~2 hang moi cot, tang len 3 neu anh nhi phan con nhieu.
  static constexpr int WINDOW_MIN_POINTS = 2;

  // ------------------------------------------------------------------------
  // Kich thuoc that (met) - doi ra pixel theo tung hang, khong phu thuoc ROI
  // ------------------------------------------------------------------------

  // Be rong toi da cua 1 vach (bang keo). Cong them do nghieng cua vach
  // trong cua so khi kiem tra run, xem run_limit_px().
  static constexpr float TAPE_MAX_M = 0.06f;

  // Hai vach phai cach nhau it nhat (tranh 1 vach bi tach thanh 2 dinh)
  static constexpr float MIN_LANE_GAP_M = 0.30f;

  // Khoang be rong lan chap nhan. Lan 0.50 m va 0.61 m deu nam trong.
  static constexpr float LANE_W_MIN_M = 0.40f;
  static constexpr float LANE_W_MAX_M = 0.80f;

  // Be rong lan ban dau, sau do tu hoc lai moi khi thay du 2 vach
  static constexpr float LANE_W_DEFAULT_M = 0.55f;
  static constexpr float LANE_W_ALPHA = 0.2f;

  // ------------------------------------------------------------------------
  // Tien xu ly anh
  // ------------------------------------------------------------------------

  static constexpr int BLUR_KSIZE = 5;
  static constexpr double CLAHE_CLIP = 2.0;
  static constexpr int CLAHE_TILE = 8;

  // Chenh sang giua nhom toi (vach) va nhom sang (san) trong ROI, theo
  // Otsu. Duoi gia tri nay coi nhu khong co vach, tranh bat nhieu khi san
  // dong deu. Gia tri hien tren anh vis (ctr=...), chinh theo do do thuc te.
  static constexpr double MIN_CONTRAST = 15.0;

  // Doan vach bi ngat do anh den cham rua sang (THRESH_BINARY_INV loi di vach
  // toi): noi suy tuyen tinh giua 2 moc vach va ve lai vao bin, toi da
  // BRIDGE_MAX_BANDS cua so lien tiep. Doan dai hon thi khong noi (khong
  // doan duong gia).
  static constexpr int BRIDGE_MAX_BANDS = 3;

  // ------------------------------------------------------------------------
  // Hinh hoc
  // ------------------------------------------------------------------------

  // Dinh ROI khong duoc cao hon chan troi + ROI_MIN_DY hang: qua gan chan
  // troi thi vach chi con ~2 px (bi MORPH_OPEN(3,3) an mat) va px_to_cm
  // cua doan xat nhieu loi hon. 12 cho phep thay xa them ~2 m duong dat
  // so voi 20 cu (f ~ 230 px, h = 0.3 m: dy 20 -> 3.4 m, dy 12 -> 5.7 m).
  static constexpr int ROI_MIN_DY = 12;

  // Day ROI tu dong keo len de 1 lan rong LANE_W_FIT_M cong moi ben
  // ROI_SIDE_MARGIN_M van nam tron trong be ngang anh. Cam cui xuong thi
  // hang gan xe chi thay duoc ~0.7 m be ngang, khong du chua ca 2 vach.
  static constexpr float LANE_W_FIT_M = 0.61f;
  static constexpr float ROI_SIDE_MARGIN_M = 0.10f;

  // Hang lay do lech, tinh tu dinh ROI xuong
  static constexpr float LOOKAHEAD_FRAC = 0.58f;

  // Nua do rong dinh hinh thang theo WORK_W (day hinh thang phu het be ngang)
  static constexpr float ROI_TOP_HALF_FRAC = 0.30f;

  // ------------------------------------------------------------------------
  // Loc, tre va giu gia tri
  // ------------------------------------------------------------------------

  static constexpr float EMA_ALPHA = 0.35f;

  // Tuoi frame vuot qua gia tri nay -> coi nhu mat camera
  static constexpr int STALE_AGE_MS = 200;

  static constexpr int MAX_READ_FAIL = 25;

  // Mat lan: giu dev_px cuoi toi da bay nhieu ms (tinh theo thoi gian vi
  // cam fps thap, 15 frame co the la hon 1 giay)
  static constexpr int HOLD_MS = 500;

  // ------------------------------------------------------------------------
  // He so toc do de nghi (speed_scale, 0..1). Firmware nhan voi toc do dat.
  // Giam theo |path_dx_cm|: duong tam phia truoc lech khoi huong xe bao nhieu
  // ------------------------------------------------------------------------

  static constexpr float PATH_DX_START_CM = 10.0f; // bat dau giam toc
  static constexpr float PATH_DX_FULL_CM = 35.0f;  // giam het muc

  // Giam theo do cong |1/R| (1/m) cua duong tam: khong phu thuoc cam nhin
  // xa hay gan nhu path_dx. 0.15 = R 6.7 m, 0.60 = R 1.7 m.
  static constexpr float CURV_START = 0.15f;
  static constexpr float CURV_FULL = 0.60f;
  static constexpr float SPEED_MIN_CURVE = 0.45f; // toc do nho nhat trong cua
  static constexpr float SPEED_ONE_LINE = 0.55f;  // chi thay 1 vach / it cua so
  static constexpr float SPEED_LOST = 0.30f;      // dang giu dev cu

  // ------------------------------------------------------------------------
  // Ket qua mot lan nhan
  // ------------------------------------------------------------------------

  struct LaneOutput {
    // Diem 2 vach (chi cua so thay du 2 vach), toa do WORK_W x WORK_H
    std::vector<cv::Point> left_pts;
    std::vector<cv::Point> right_pts;

    // True chi khi state == TWO_LINES
    bool two_lanes = false;

    // Firmware nen dung truong nay: ONE_LINE van co dev_px dung duoc
    LaneState state = LaneState::LOST;

    // Do lech tam lan so voi giua anh, pixel anh goc, tai hang nhin truoc.
    // Khi LOST: giu gia tri cuoi toi da HOLD_MS, qua do = 0.
    int dev_px = 0;

    // Cung thong tin theo cm, chi de hien thi
    float dev_cm = 0.0f;
    float lane_width_cm = 0.0f;

    // Do cong duong tam (1/m, = 1/R). > 0: re phai, < 0: re trai. Do bang
    // met nen khong phu thuoc cam nhin xa hay gan. Chi co khi >= 4 diem
    // tam trai du 0.3 m chieu sau, con lai = 0.
    float curvature = 0.0f;

    // Do lech ngang (cm) cua duong tam o doan xa so voi doan gan, do bang
    // met. ~0: duong cung huong xe. > 0: duong phia truoc re sang phai,
    // < 0: re sang trai. Lon khi vao cua hoac xe dang chech huong.
    float path_dx_cm = 0.0f;

    // 0..1: nhan voi toc do dat de vao cua cham lai
    float speed_scale = 1.0f;

    // True neu fit duong tam thanh cong. Neu false, dev lay tu diem tam
    // gan hang nhin truoc nhat.
    bool fit_ok = false;

    // Tuoi frame moi nhat (ms), -1 neu chua co frame nao
    unsigned long age_ms = 0;
    bool stale = true;

    cv::Mat vis;

    // Anh debug cho GUI: chi duoc dien khi copy_vis = true
    cv::Mat raw;   // khung camera goc, chua ve gi
    cv::Mat roi;   // vung ROI goc (do phan giai camera), tu fy0..fy1
    cv::Mat bin;   // mask nhi phan 0/1 sau morphology, WORK_W x roi_h

    double proc_ms = 0.0;
    long frame_id = 0;
  };

  // ------------------------------------------------------------------------
  // Vong doi
  // ------------------------------------------------------------------------

  // camera_index < 0 -> tu doi kiem tra cac cong may 0..3
  explicit CameraLane(int camera_index = -1, int target_fps = 30,
                      const CameraProfile &profile = CameraProfile{});

  ~CameraLane();

  CameraLane(const CameraLane &) = delete;
  CameraLane &operator=(const CameraLane &) = delete;

  bool start();
  void stop();
  bool is_running() const;

  // copy_vis = false de vong dieu khien khong copy anh moi frame
  void get_latest(LaneOutput &out, bool copy_vis = false) const;

  // 0.0..1.0, ti le tren cua khung anh. Mac dinh profile.roi_top_frac.
  void set_roi_top_frac(float frac);

private:
  bool open_camera();
  void capture_loop();

  bool detect(const cv::Mat &frame, LaneOutput &out);

  // Tim cap vach dau tien trong cua so xa, chon cap co be rong gan be rong
  // lan da hoc nhat. dy = hang cua so - horizon_y_.
  bool seed_pair_from(const cv::Mat &col_sum, int dy, int win_h, int &seed_left,
                      int &seed_right) const;

  // Tim tam 1 vach quanh seed. max_run: be rong toi da cua run (px).
  static bool peak_in(const cv::Mat &col_sum, int seed, int max_run, int &peak);

  // Be rong toi da cua run cot cua 1 vach: be rong bang keo theo phoi canh
  // cong do loe ngang do vach nghieng trong 1 cua so.
  int run_limit_px(int x, int dy, int win_h) const;

  // met -> pixel ngang tai hang cach chan troi dy
  int lane_px(float meters, int dy) const;

  // Nguong Otsu chi tinh tren pixel trong mask. contrast = chenh trung binh
  // giua nhom sang va nhom toi.
  static int otsu_masked(const cv::Mat &gray, const cv::Mat &mask,
                         double &contrast);

  // Fit x = a*t^2 + b*t + c, t = (y - y0) / (y1 - y0). Duoi 4 diem chi fit
  // duong thang (a = 0) vi bac 2 qua 3 diem rat de dao dong.
  static bool fit_quad(const std::vector<double> &t,
                       const std::vector<double> &x, double coef[3]);

  static bool fit_poly2(const std::vector<cv::Point> &pts, double coef[3],
                        int &y0, int &y1);

  float px_to_cm(float px, int y) const;

  // Khoang cach ngang (m) tu chan cam toi diem mat dat o hang y cua anh
  // WORK_H. Dung de kiem tra hinh hoc khi khoi dong (hang giua anh phai ra
  // dung axis_ground_m).
  double ground_dist_m(double y) const;

  // ------------------------------------------------------------------------
  // Trang thai
  // ------------------------------------------------------------------------

  int camera_index_;
  int target_fps_;

  // Hinh hoc suy ra tu CameraProfile (cap nhat 1 lan trong constructor)
  float h_ = 0.30f;     // do cao cam (m)
  double pitch_ = 0.0;  // goc cui cua truc quang (rad)
  double f_px_ = 0.0;   // tieu cu tren anh WORK_W (px)
  float k_ = 0.30f;     // h / cos(pitch): met = px * k_ / dy
  int horizon_y_ = 120; // hang chan troi (co the am: nam ngoai anh)
  float roi_bottom_frac_ = 0.93f;
  bool logged_geometry_ = false;

  std::atomic<float> roi_top_frac_{0.52f};

  cv::VideoCapture cap_;

  std::atomic<bool> running_{false};
  std::thread worker_;

  mutable std::mutex mtx_;

  LaneOutput latest_;

  // Cac bien duoi day chi dung trong luong doc
  int dev_ema_ = 0;
  bool ema_primed_ = false;
  float lane_w_m_ = LANE_W_DEFAULT_M;
  bool warned_roi_ = false;
  std::chrono::steady_clock::time_point last_valid_time_{};

  // CLAHE tao 1 lan, dung lai moi frame
  cv::Ptr<cv::CLAHE> clahe_;

  long frame_id_ = 0;

  std::chrono::steady_clock::time_point last_frame_time_{};
};