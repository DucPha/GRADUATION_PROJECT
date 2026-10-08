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
  double cam_to_rear_m = 0.30;     // chan camera nam truoc truc sau bay nhieu
  double max_steer_deg = 30.0;     // goc banh lon nhat
  double lookahead_min_m = 0.35;   // Ld = clamp(min + gain * v, min, max)
  double lookahead_max_m = 0.80;
  double lookahead_gain_s = 0.20;
  double camera_latency_s = 0.04;   // chup -> nhan diem (phoi sang + USB + giai ma)
  double actuator_latency_s = 0.08; // lenh -> servo quay toi
  double car_half_width_m = 0.10;   // nua be ngang xe (tinh tu tam banh)
  double line_margin_m = 0.04;      // khoang trong toi thieu con lai toi vach
  double guard_gain_deg_per_m = 150.0;
};

class PathTracker {
public:
  struct Output {
    bool valid = false;
    double steer_deg = 0.0;   // goc banh mong muon, > 0 = phai
    double lookahead_m = 0.0; // khoang nhin truoc dang dung
    double ahead_m = 0.0;     // chieu dai duong da nho con o truoc banh truoc
    double curv_ahead = 0.0;  // do cong duong phia truoc (1/m), > 0 = re phai
    int turn = 0;             // du doan cua: -1 trai, 0 thang, +1 phai
    double guard_deg = 0.0;   // phan goc do rao chan vach cong them
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

  // Tinh goc lai cho vi tri hien tai
  Output compute(double v);

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
};
