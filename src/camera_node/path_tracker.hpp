#pragma once

#include <opencv2/core.hpp>

#include <chrono>
#include <deque>
#include <vector>

// ============================================================================
// PATH TRACKER - nho duong tam lan + odometry, lai pure pursuit tu TRUC SAU
// ----------------------------------------------------------------------------
// Van de: camera chi thay mat dat tu ~0.47 m truoc chan camera tro di, chan
// camera lai nam truoc truc sau. Lai thang theo diem nhin truoc cua camera
// (0.70 m tu camera ~ 1.0 m tu truc sau) thi xe be lai khi con cach cua
// ~0.5 m -> cat cua, de vach trong. Day diem nhin xa them thi toi cua lai
// khong con vach trong khung hinh.
//
// Cach giai:
//  1. NHO duong tam lan va 2 vach tren he toa do THE GIOI. Vi tri xe cap nhat
//     100 Hz bang odometry (mo hinh xe dap: toc do lenh + goc banh). Moi
//     frame camera duoc dat dung vi tri xe LUC CHUP (bu tre camera) roi noi
//     vao doan da nho phia sau -> vung mu giua xe va mep ROI van co duong.
//  2. Pure pursuit tinh tu TRUC SAU, khoang nhin truoc ngan theo toc do
//     (0.35-0.8 m) -> be lai dung luc toi cua, khong som. Bu tre servo bang
//     cach du doan vi tri xe sau actuator_latency_s.
//  3. Vach khong con thay (vach trong cua cua gat ra khoi khung) van con
//     trong bo nho = VACH AO. Banh truoc qua sat vach (that hay ao) -> cong
//     them goc day ra (rao chan) -> chay 1 vach khong de vach con lai.
//  4. Mat vach: van chay theo duong da nho toi khi het duong phia truoc.
//
// Toa do xe: goc = giua truc sau, X sang PHAI, Z ve PHIA TRUOC (m).
// Goc banh / goc huong: > 0 = sang phai.
// ============================================================================

struct VehicleParams {
  double wheelbase_m = 0.26;       // truc truoc - truc sau
  double cam_to_rear_m = 0.18;     // chan camera nam truoc truc sau (do lai tren xe 2026-10-08)
  double max_steer_deg = 30.0;     // goc banh lon nhat
  double lookahead_min_m = 0.35;   // Ld = clamp(min + gain * v, min, max)
  double lookahead_max_m = 0.80;
  double lookahead_gain_s = 0.20;
  // Trong cua nhan Ld voi he so nay (khong nho hon lookahead_min_m), 1 = tat
  double lookahead_corner_scale = 0.8;
  double camera_latency_s = 0.04;   // chup -> nhan diem (phoi sang + USB + giai ma)
  double actuator_latency_s = 0.08; // lenh -> servo quay toi
  // Xe rong 25 cm (do 2026-10-09), bang keo 7 cm, lan 0.42 m tam-tam: xe
  // giua lan chi con ~5 cm moi ben toi MEP bang keo.
  double car_half_width_m = 0.125;  // nua be ngang xe
  double tape_half_m = 0.035;       // nua be rong bang keo (vach = tam bang keo)
  double line_margin_m = 0.02;      // khoang trong toi thieu con lai toi mep vach
  double guard_gain_deg_per_m = 150.0;
  // Phan hoi LECH NGANG (kieu Stanley): cong them
  // atan(xte_gain * e / (v + xte_soft_mps)), e = khoang cach tu chan camera
  // toi duong tam. Pure pursuit thuan chi "nhin" lech ngang qua diem ngam xa
  // nen rat mem (lech 5 cm ~ 3 do banh); ty so lai / vi tri camera khai bao
  // lech thuc te (chua do tren xe) la xe chay lech han 1 ben, de len vach.
  double xte_gain = 2.0;     // 1/s, 0 = tat
  double xte_soft_mps = 0.5; // m/s, tranh chia 0 khi xe cham
  // Lech xa hon xte_far_m: phan vuot qua nhan them xte_far_gain (1/s, cong
  // vao xte_gain) -> xe lech xa (vd 15 cm sau cua) ve tam nhanh hon, lech
  // nho tren duong thang giu nhu cu. 0 = tat.
  double xte_far_m = 0.05;
  double xte_far_gain = 0.0; // 10/10 19:27: 2.0 -> xe chao tren duong thang
  // Khau TICH PHAN lech ngang (STEER_KI cua Python): bu lech trim servo,
  // camera lap lech / xoay vai do, ty so lai sai -> het lech 1 ben tren duong
  // thang va trong cua dai. Don vi: do banh / (m * s); gioi han +-xte_i_max_deg.
  double xte_ki = 50.0;
  double xte_i_max_deg = 6.0;
  // DANH LAI MANH HON TRONG CUA: nhan phan pure pursuit voi he so nay khi
  // duong phia truoc dang cong (noi suy theo do cong 0.4 -> 1.0 1/m). Bu tre
  // servo / loc lai lam xe be lai thieu, bat ra phia ngoai cua. 1 = tat.
  double corner_gain = 1.15;
  // GOC GAP: ban kinh cung re (m) dung de tinh diem bat dau be lai truoc goc
  // gap: d = R * tan(goc / 2) tinh tu truc sau. Truoc diem do xe di thang theo
  // doan truoc goc (khong cat goc). Lon hon -> be lai som hon; 0 = tat.
  // ~ ban kinh nho nhat cua xe (servo +-35, banh ~0.42 x servo: ~1.0 m).
  double kink_radius_m = 1.0;
  // PID DUONG THANG (sieu muot): tren duong thang thay pure pursuit + xte bang
  //   steer = kp * e + kd * psi + I
  // e = lech ngang chan camera -> duong tam (m, > 0 tam ben phai), psi = goc
  // huong duong tam so voi xe (do) - day chinh la khau D (de/dt = v sin psi)
  // nhung DO TRUC TIEP tu camera, khong dao ham tin hieu nhieu. Ca hai lay tu
  // duong thang khop binh phuong toi thieu tren pid_fit_m truoc chan camera.
  // Chay 10/10 19:44: PP + xte + I (bao hoa -6 do 40 s) thieu giam chan ->
  // banh -21 <-> +21 tren duong thang. Gain chon cho he xe dap (L 0.26, banh
  // that ~0.7 x lenh): bước sóng ~4 m, ty so tat ~1 -> ve tam khong vot.
  // Vao cua (do cong phia truoc 0.4 -> 1.0, goc gap phia truoc) chuyen dan ve
  // pure pursuit nhu cu. pid_kp_deg_per_m = 0 -> tat.
  double pid_kp_deg_per_m = 54.0;  // do banh / m lech (0.54 do / cm)
  double pid_kd = 1.0;             // do banh / do lech huong
  double pid_ki = 10.0;            // do banh / (m * s)
  double pid_i_max_deg = 3.0;
  double pid_max_deg = 15.0;       // gioi han dau ra PID tren duong thang
  double pid_filter_s = 0.12;      // loc e / psi (s)
  double pid_fit_m = 0.6;          // chieu dai doan khop duong thang (m)
};

class PathTracker {
public:
  struct Output {
    bool valid = false;
    double steer_deg = 0.0;   // goc banh mong muon, > 0 = phai
    double lookahead_m = 0.0; // khoang nhin truoc dang dung
    double ahead_m = 0.0;     // chieu dai duong da nho con o truoc banh truoc
    double curv_ahead = 0.0;  // do cong duong phia truoc (1/m), > 0 = re phai
    // Quang duong doc duong tam tu banh truoc toi CUA phia truoc (m), < 0 =
    // chua thay cua. Dung de giam toc TRUOC khi vao cua (xem compute()).
    double corner_dist_m = -1.0;
    // Goc gap phia truoc (tu ngang truc sau, < 0 = khong co), goc quay (do,
    // > 0 phai) va dang giu thang cho toi diem vao cua
    double kink_dist_m = -1.0;
    double kink_deg = 0.0;
    bool kink_hold = false;
    int turn = 0;             // du doan cua: -1 trai, 0 thang, +1 phai
    double guard_deg = 0.0;   // phan goc do rao chan vach cong them
    double xte_m = 0.0;       // lech ngang chan camera -> duong tam (> 0: tam ben phai)
    double xte_deg = 0.0;     // phan goc do phan hoi lech ngang cong them (P + I)
    double xte_i_deg = 0.0;   // rieng phan tich phan
    // PID duong thang: trong so (1 = thang, 0 = cua), e (m), psi (do), I (do)
    double pid_w = 0.0;
    double pid_e_m = 0.0;
    double pid_psi_deg = 0.0;
    double pid_i_deg = 0.0;
    double pid_deg = 0.0;
    bool virt_left = false;   // vach trai dang la vach ao (khong con thay)
    bool virt_right = false;
    // Toa do xe, de ve ban do nho tren GUI
    std::vector<cv::Point2f> centre_v;
    std::vector<cv::Point2f> left_v;
    std::vector<cv::Point2f> right_v;
    cv::Point2f target_v{0.0f, 0.0f};
  };

  explicit PathTracker(const VehicleParams &p = VehicleParams{});

  void reset();

  // Tich phan odometry. v (m/s), steer_deg = goc banh hien tai (> 0 phai).
  void predict(double v, double steer_deg,
               std::chrono::steady_clock::time_point now);

  // Them ket qua 1 frame camera (toa do mat dat, goc = chan camera, X phai,
  // Z truoc), chup luc `stamp`. Vach rong = khong thay -> giu vach da nho.
  void add_observation(const std::vector<cv::Point2f> &centre_cam,
                       const std::vector<cv::Point2f> &left_cam,
                       const std::vector<cv::Point2f> &right_cam,
                       std::chrono::steady_clock::time_point stamp);

  // Tinh goc lai cho vi tri hien tai. bias_m: doi duong tam sang PHAI (m)
  Output compute(double v, double bias_m = 0.0);

  const VehicleParams &params() const { return p_; }

private:
  struct Pose {
    double x = 0.0, z = 0.0, th = 0.0;
  };
  struct Memory {
    std::vector<cv::Point2f> pts; // toa do the gioi, sau -> truoc
    std::chrono::steady_clock::time_point seen{};
    int reject = 0; // so frame lien tiep khong khop bo nho
  };

  static Pose step(const Pose &p, double v, double steer_rad, double L,
                   double dt);
  Pose pose_at(std::chrono::steady_clock::time_point t) const;
  static cv::Point2f to_world(const Pose &p, const cv::Point2f &v);
  static cv::Point2f to_vehicle(const Pose &p, const cv::Point2f &w);

  void merge(Memory &mem, const std::vector<cv::Point2f> &cam, const Pose &pc,
             std::chrono::steady_clock::time_point stamp);
  void prune(Memory &mem) const;
  std::vector<cv::Point2f> in_vehicle(const Memory &mem, const Pose &p) const;
  double line_clearance(const std::vector<cv::Point2f> &line_v, double z,
                        double x_wheel, int side, bool &found) const;

  VehicleParams p_;
  Pose pose_;
  double last_v_ = 0.0;
  double last_steer_rad_ = 0.0;
  bool started_ = false;
  std::chrono::steady_clock::time_point now_{};
  std::deque<std::pair<std::chrono::steady_clock::time_point, Pose>> hist_;

  Memory centre_;
  Memory left_;
  Memory right_;
  int turn_ = 0;
  double curv_prev_ = 0.0; // do cong phia truoc o lan compute truoc
  double xte_i_deg_ = 0.0;
  bool pid_primed_ = false;
  double pid_e_f_ = 0.0;
  double pid_psi_f_ = 0.0;
  double pid_i_deg_ = 0.0;
  std::chrono::steady_clock::time_point last_compute_{};
};
