#pragma once

// ============================================================================
// lane_keeping.hpp - GIU LAN: nho duong + MPC (khoi CONTROL cua xe)
// ----------------------------------------------------------------------------
// Vao : duong tam / 2 vach tu lane_detector (toa do mat san duoi camera),
//       toc do that va goc banh dang lenh (odometry 100 Hz).
// Ra  : PathTracker::Output - goc banh mong muon (do, > 0 = phai), do cong /
//       khoang cach toi cua phia truoc (de giam toc), du lieu ve ban do.
//
// Thanh phan:
//   1. LkMpcCfg, lk_mpc_solve()  - MPC giu lan (mo hinh xe dap tuong doi voi
//      lan, xem truoc do cong, bu tre servo), giai lai moi chu ky.
//   2. LkOracle                  - hoc online ti so lai g va do lech lai b that
//      cua xe (binh phuong toi thieu de quy, mien gia tri bi chan).
//      1 + 2 theo Bian et al., "An Advanced Lane-Keeping Assistance System
//      with Switchable Assistance Modes" (learning-based MPC).
//   3. VehicleParams             - thong so xe va bo dieu khien.
//   4. PathTracker               - nho duong tam + 2 vach tren toa do THE GIOI
//      (qua vung mu truoc xe), odometry, lai (MPC; pure pursuit du phong),
//      rao chan vach, du doan cua.
// ============================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <vector>
#include <opencv2/core.hpp>

// ============================================================================
// 1-2. MPC GIU LAN + HOC SAI MO HINH ONLINE
// ----------------------------------------------------------------------------
// Theo Bian, Ding, Hu, Xu, Wang, Li, "An Advanced Lane-Keeping Assistance
// System with Switchable Assistance Modes" (LBMPC): mo hinh danh nghia tuyen
// tinh theo trang thai TUONG DOI voi lan (lech ngang e_y, lech huong e_psi),
// do cong duong rho la dau vao ngoai, cong them "oracle" hoc phan sai mo hinh
// online; giai lai moi chu ky (receding horizon); trong so doi theo vung: xa
// tam -> uu tien keo ve tam (e_y), gan tam -> uu tien huong (e_psi) cho em.
//
// Ban xe nho (khong co nguoi lai, 0.4 m/s): mo hinh xe dap dong hoc
//   e_y'   = v * e_psi
//   e_psi' = v * (kappa - rho),  kappa = (g * d_act + b) / L
//   d_act' = (u - d_act) / tau            (servo tre bac 1)
// u = goc banh LENH (rad, theo ti so lai danh nghia cua node), g / b = he so
// va do lech lai THAT ma oracle hoc (13/10: ti so that ~0.42 so voi 0.6 khai
// bao -> g ~0.7; xe bi keo lech phai khi chay -> b != 0).
// Dau: e_y > 0 = xe ben PHAI duong tam, e_psi > 0 = mui xe lech PHAI,
// rho > 0 = duong re phai, u > 0 = lai phai.
// ============================================================================

struct LkMpcCfg {
  double dt = 0.05;           // buoc du doan (s)
  int n = 24;                 // so buoc (24 x 0.05 = 1.2 s)
  double wheelbase = 0.26;    // m
  double tau = 0.15;          // tre servo bac 1 (s)
  double max_steer = 30.0 * CV_PI / 180.0;
  double zone_m = 0.05;       // |e_y| tu muc nay = "xa tam" (D_offset cua bai bao)
  double q_y_far = 30.0;      // trong so e_y khi xa tam
  double q_psi_far = 1.0;
  double q_y_near = 8.0;      // gan tam: giam e_y, tang e_psi (lai em)
  double q_psi_near = 3.0;
  double r = 0.05;            // trong so goc lai
  double rd = 8.0;            // trong so thay doi goc lai giua 2 buoc
  // LQI (tich phan lech ngang): them trang thai s = ∫ e_y dt vao bai toan, phat
  // q_i * s^2. MPC thuong (khong co tich phan) chi la kieu-2 voi NHIEU vao khop;
  // sai so mo hinh dang LECH LAI (b, g sai, deadband servo, camera lech) lam xe
  // dung o lech du tai truc (do bang mo phong: b lech 3 do -> lech ~3.9 cm).
  // Tich phan khu dung lech do -> xe luon ve giua lan. q_i = 0 = tat (nhu cu).
  double q_i = 0.0;           // trong so tich phan (m.s)
  double i_max = 0.06;        // chan tich phan, chong wind-up (m.s)
};

// Goc banh lenh u0 (rad) toi uu cho trang thai hien tai. rho[k]: do cong duong
// tai buoc k (thieu thi lay phan tu cuoi). u_prev: lenh chu ky truoc.
inline double lk_mpc_solve(double ey, double epsi, double dact, double u_prev, double v,
                           const std::vector<double> &rho, double g, double b,
                           const LkMpcCfg &c, double s0 = 0.0) {
  const int n = std::max(2, c.n);
  const double vdt = std::max(0.05, v) * c.dt;
  const double a_tau = std::clamp(c.dt / std::max(1e-3, c.tau), 0.0, 1.0);
  // Trong so theo vung, noi suy muot quanh zone_m
  const double s = std::clamp((std::fabs(ey) - 0.5 * c.zone_m) / c.zone_m, 0.0, 1.0);
  const double qy = c.q_y_near + (c.q_y_far - c.q_y_near) * s;
  const double qp = c.q_psi_near + (c.q_psi_far - c.q_psi_near) * s;

  // x_{k+1} = A x_k + B u_k + w_k. Lan truyen: x_k = free_k + sum_j Su_k[j] u_j
  cv::Matx33d A(1.0, vdt, 0.0,
                0.0, 1.0, vdt * g / c.wheelbase,
                0.0, 0.0, 1.0 - a_tau);
  const cv::Vec3d B(0.0, 0.0, a_tau);
  std::vector<cv::Vec3d> free(n);
  std::vector<std::vector<cv::Vec3d>> su(n, std::vector<cv::Vec3d>(n, cv::Vec3d(0, 0, 0)));
  cv::Vec3d x(ey, epsi, dact);
  std::vector<cv::Vec3d> prev_su(n, cv::Vec3d(0, 0, 0));
  for (int k = 0; k < n; ++k) {
    const double rk = rho.empty() ? 0.0 : rho[std::min<size_t>(k, rho.size() - 1)];
    const cv::Vec3d w(0.0, vdt * (b / c.wheelbase - rk), 0.0);
    x = A * x + w;
    free[k] = x;
    for (int j = 0; j < n; ++j) {
      cv::Vec3d t = A * prev_su[j];
      if (j == k) {
        t += B;
      }
      su[k][j] = t;
    }
    prev_su = su[k];
  }

  // Cost = sum_k qy*ey_k^2 + qp*epsi_k^2 + r*u_k^2 + rd*(u_k - u_{k-1})^2
  //        + q_i * s_k^2   (s_k = tich phan lech ngang toi buoc k, LQI)
  cv::Mat H = cv::Mat::zeros(n, n, CV_64F);
  cv::Mat f = cv::Mat::zeros(n, 1, CV_64F);
  // Trang thai tich phan khong anh huong e_y/e_psi/d_act nen phan lan truyen
  // free/su van dung: s_k = si + dt * sum_{j<=k} ey_j, tuyen tinh theo u.
  const double si = std::clamp(s0, -c.i_max, c.i_max);
  const bool use_i = c.q_i > 0.0;
  double cum_free = 0.0;
  std::vector<double> cum_su(n, 0.0);
  for (int k = 0; k < n; ++k) {
    for (int i = 0; i < n; ++i) {
      const double yi = su[k][i][0], pi = su[k][i][1];
      f.at<double>(i) += qy * yi * free[k][0] + qp * pi * free[k][1];
      for (int j = i; j < n; ++j) {
        H.at<double>(i, j) += qy * yi * su[k][j][0] + qp * pi * su[k][j][1];
      }
    }
    if (use_i) {
      cum_free += free[k][0];
      for (int i = 0; i < n; ++i) {
        cum_su[i] += su[k][i][0];
      }
      const double cc = si + c.dt * cum_free;
      for (int i = 0; i < n; ++i) {
        const double gi = c.dt * cum_su[i];
        f.at<double>(i) += c.q_i * gi * cc;
        for (int j = i; j < n; ++j) {
          H.at<double>(i, j) += c.q_i * gi * (c.dt * cum_su[j]);
        }
      }
    }
  }
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < i; ++j) {
      H.at<double>(i, j) = H.at<double>(j, i);
    }
    H.at<double>(i, i) += c.r + 2.0 * c.rd;
    if (i + 1 < n) {
      H.at<double>(i, i + 1) -= c.rd;
      H.at<double>(i + 1, i) -= c.rd;
    }
  }
  H.at<double>(n - 1, n - 1) -= c.rd; // u_{n-1} chi co 1 hieu phia truoc
  f.at<double>(0) -= c.rd * u_prev;
  cv::Mat u;
  if (!cv::solve(H, -f, u, cv::DECOMP_CHOLESKY)) {
    cv::solve(H, -f, u, cv::DECOMP_SVD);
  }
  return std::clamp(u.at<double>(0), -c.max_steer, c.max_steer);
}

// "Oracle": hoc ti so lai g va do lech b tu do cong THAT cua xe (do bang
// camera): L * kappa_do = g * d + b. Binh phuong toi thieu de quy co he so
// quen (tuong duong Kalman cho tham so tuyen tinh, buoc nhay ngau nhien).
struct LkOracle {
  double g = 1.0, b = 0.0;
  double P[2][2] = {{0.5, 0.0}, {0.0, 0.01}};
  double lambda = 0.998;  // he so quen moi mau (~500 mau ~ vai phut)
  // Mien bi chan cua oracle (nhu "tube" cua LBMPC): du lieu that 13/10 chu yeu
  // re 1 chieu -> g va b bu tru nhau, khong chan thi troi 0.4..1.2
  double g_min = 0.55, g_max = 0.95;
  double b_max = 6.0 * CV_PI / 180.0;
  double resid_max = 0.12; // |L*kappa - du doan| lon hon = mau nhieu, bo
  int used = 0;

  void reset(double g0, double b0) {
    g = g0;
    b = b0;
    P[0][0] = 0.5;
    P[0][1] = P[1][0] = 0.0;
    P[1][1] = 0.01;
    used = 0;
  }

  // d: goc banh lenh TB trong cua so (rad), y = L * kappa do duoc
  bool update(double d, double y) {
    const double e = y - (g * d + b);
    if (!std::isfinite(e) || std::fabs(e) > resid_max) {
      return false;
    }
    const double pd0 = P[0][0] * d + P[0][1];
    const double pd1 = P[1][0] * d + P[1][1];
    const double den = lambda + d * pd0 + pd1;
    const double k0 = pd0 / den, k1 = pd1 / den;
    g = std::clamp(g + k0 * e, g_min, g_max);
    b = std::clamp(b + k1 * e, -b_max, b_max);
    const double p00 = (P[0][0] - k0 * pd0) / lambda;
    const double p01 = (P[0][1] - k0 * pd1) / lambda;
    const double p10 = (P[1][0] - k1 * pd0) / lambda;
    const double p11 = (P[1][1] - k1 * pd1) / lambda;
    P[0][0] = std::min(p00, 1.0);
    P[0][1] = p01;
    P[1][0] = p10;
    P[1][1] = std::min(p11, 0.05);
    ++used;
    return true;
  }
};

// ============================================================================
// 3-4. PATH TRACKER - nho duong tam lan + odometry, lai tu TRUC SAU
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
  // Khau TICH PHAN lech ngang (STEER_KI cua Python): bu lech trim servo,
  // camera lap lech / xoay vai do, ty so lai sai -> het lech 1 ben tren duong
  // thang va trong cua dai. Don vi: do banh / (m * s); gioi han +-xte_i_max_deg.
  double xte_ki = 50.0;
  double xte_i_max_deg = 12.0;
  // DANH LAI MANH HON TRONG CUA: nhan phan pure pursuit voi he so nay khi
  // duong phia truoc dang cong (noi suy theo do cong 0.4 -> 1.0 1/m). Bu tre
  // servo / loc lai lam xe be lai thieu, bat ra phia ngoai cua. 1 = tat.
  double corner_gain = 1.15;
  // MPC GIU LAN + HOC SAI MO HINH (phan 1-2 o tren, theo Bian et al. LBMPC): thay
  // pure pursuit + phan hoi lech ngang khi co duong tam phu toi truc sau.
  // Rao chan vach van cong them. false = lai nhu cu.
  bool mpc_enable = true;
  double mpc_tau_s = 0.15;     // tre servo + loc (s)
  double mpc_horizon_s = 1.2;  // tam du doan (s)
  double mpc_zone_m = 0.05;    // |e_y| tu muc nay uu tien keo ve tam
  double mpc_rd = 30.0;       // phat thay doi goc lai giua 2 buoc (em). 13/10 du lieu that: 1 -> 8 -> 30: giat p90 6.7 -> 4.9 -> 4.6 do/frame, het lai 38 -> 23 -> 18 %; mo phong: giu lan khong kem di
  // Ti so lai that / khai bao va do lech lai ban dau (do 10/10: 0.42 / 0.6)
  double mpc_g0 = 0.70;
  double mpc_b0_deg = 0.0;
  bool mpc_learn = true;       // hoc g, b online tu do cong that (camera)
  // LQI: tich phan lech ngang e_y trong MPC -> khu lech du tai truc (sai so b/g,
  // deadband servo, camera lech). q_i = 0 = tat (kieu MPC cu). Xem lane_keeping.hpp.
  double mpc_qi = 60.0;        // trong so tich phan (m.s)
  double mpc_i_max = 0.06;     // chan tich phan, chong wind-up (m.s)
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
    int turn = 0;             // du doan cua: -1 trai, 0 thang, +1 phai
    double guard_deg = 0.0;   // phan goc do rao chan vach cong them
    double xte_m = 0.0;       // lech ngang chan camera -> duong tam (> 0: tam ben phai)
    double xte_deg = 0.0;     // phan goc do phan hoi lech ngang cong them (P + I)
    double xte_i_deg = 0.0;   // rieng phan tich phan
    bool mpc = false;         // goc lai nay do MPC tinh
    double mpc_ey_m = 0.0;    // lech ngang truc sau (> 0: xe ben PHAI duong tam)
    double mpc_epsi_deg = 0.0; // lech huong (> 0: mui xe lech PHAI)
    double mpc_g = 1.0;       // ti so lai that da hoc
    double mpc_b_deg = 0.0;   // do lech lai that da hoc (do banh)
    int mpc_learn_n = 0;      // so mau oracle da dung
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

  bool mpc_steer(const std::vector<cv::Point2f> &c, double v, double dt_c, Output &o,
                 double &steer_deg);
  void learn_from(const std::vector<cv::Point2f> &centre_cam, std::chrono::steady_clock::time_point stamp);

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
  std::chrono::steady_clock::time_point last_compute_{};

  // MPC + oracle
  LkOracle oracle_;
  double d_est_rad_ = 0.0; // goc banh lenh sau tre + loc (uoc luong goc dang co)
  // LQI: tich phan lech ngang e_y (m.s) dua vao MPC; cap nhat moi chu ky khi MPC
  // dang chay (xem mpc_steer). xa dan khi mat duong / MPC khong chay.
  double mpc_s_int_ = 0.0;
  std::deque<std::pair<std::chrono::steady_clock::time_point, double>> cmd_hist_;
  struct ObsRec {
    std::chrono::steady_clock::time_point t;
    std::vector<cv::Point2f> centre;
    double d = 0.0;
    double v = 0.0;
  };
  std::deque<ObsRec> obs_hist_;
};
