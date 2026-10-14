// ============================================================================
// lane_keeping.cpp - GIU LAN (cai dat cua lane_keeping.hpp)
// ----------------------------------------------------------------------------
// Muc luc:
//   1. Ham tien ich (polyline, cung duong, huong duong theo do sau)
//   2. Odometry: mo hinh xe dap tai truc sau, lich su vi tri (bu tre camera),
//      uoc luong goc banh dang co (tre + loc) cho MPC / oracle
//   3. Bo nho duong: ghep frame camera vao duong da nho, bo diem cu
//   4. Oracle: do do cong THAT cua xe tu 2 frame cach ~0.5 s -> hoc g, b
//   5. compute(): MPC (pure pursuit + lech ngang khi MPC khong chay duoc),
//      rao chan vach, do cong / khoang cach toi cua phia truoc
//   6. mpc_steer(): trang thai e_y, e_psi tai truc sau + do cong phia truoc
// ============================================================================

#include "lane_keeping.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

using Clock = std::chrono::steady_clock;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;

// Bo nho qua bay lau khong duoc camera xac nhan lai thi xoa
constexpr double MEMORY_TTL_S = 2.0;
// Diem nho nam sau truc sau qua muc nay thi bo
constexpr float BEHIND_KEEP_M = 0.30f;
// Frame moi lech duong da nho qua muc nay = khong khop
constexpr float MATCH_TOL_M = 0.25f;
// Khong khop lien tiep bay nhieu frame moi tin frame moi (bo bo nho cu)
constexpr int REJECT_RESET = 3;
// Doan moi chong len duong da nho ma lech NGANG (trung vi) qua muc nay = frame
// nhan sai (vet loa den de len bang keo lam vach lech / dut) -> bo qua frame
// do; lech nhu vay REJECT_RESET frame lien tiep moi tin. Truoc day chi so dau
// doan voi MATCH_TOL_M 0.25 m -> vach lech 10-20 cm vi loa van duoc nhan, xe
// dang di thang bong danh lai ra de len vach.
constexpr float JUMP_TOL_M = 0.07f;
// Vach khong duoc thay lai qua lau thi coi la vach ao
constexpr double VIRTUAL_AFTER_S = 0.15;
// Rao chan: goc cong them toi da (do)
constexpr double GUARD_MAX_DEG = 15.0;
// Do cua phia truoc: day cung CORNER_CHORD_M doc duong tam lech huong so voi
// day cung tai banh truoc >= CORNER_TURN_DEG = co cua. Thu vong kin (nhieu
// vach 1 cm): 15 do bao cua nham ~3% thoi gian tren duong thang (chi giam toc
// nhe), phat hien cua R 0.8 khi con cach ~0.5-0.8 m truoc banh truoc.
constexpr double CORNER_CHORD_M = 0.25;
constexpr double CORNER_TURN_DEG = 15.0;
constexpr double CORNER_STEP_M = 0.05;

// Do dai vector 2D
inline float norm2(const cv::Point2f &v) { return std::sqrt(v.x * v.x + v.y * v.y); }

// Vector don vi (vector 0 -> huong thang phia truoc)
inline cv::Point2f unit(const cv::Point2f &v) {
  const float n = norm2(v);
  return n > 1e-6f ? v * (1.0f / n) : cv::Point2f(0.0f, 1.0f);
}

// Diem gan p nhat tren duong gap khuc (seg = chi so doan chua diem do)
cv::Point2f closest_on(const cv::Point2f &p, const std::vector<cv::Point2f> &poly,
                       float &dist, size_t *seg = nullptr) {
  dist = std::numeric_limits<float>::max();
  if (seg) {
    *seg = 0;
  }
  cv::Point2f best = poly.empty() ? p : poly.front();
  if (poly.size() == 1) {
    dist = norm2(p - best);
  }
  for (size_t i = 0; i + 1 < poly.size(); ++i) {
    const cv::Point2f a = poly[i];
    const cv::Point2f ab = poly[i + 1] - a;
    const float L2 = ab.dot(ab);
    const float t = L2 > 1e-9f ? std::clamp((p - a).dot(ab) / L2, 0.0f, 1.0f) : 0.0f;
    const cv::Point2f q = a + ab * t;
    const float d = norm2(p - q);
    if (d < dist) {
      dist = d;
      best = q;
      if (seg) {
        *seg = i;
      }
    }
  }
  return best;
}

// Huong duong tai doan i, lay tren cung ~+/-0.10 m
cv::Point2f dir_at(const std::vector<cv::Point2f> &g, size_t i) {
  size_t a = i;
  size_t b = std::min(i + 1, g.size() - 1);
  while (a > 0 && norm2(g[i] - g[a]) < 0.10f) {
    --a;
  }
  while (b + 1 < g.size() && norm2(g[b] - g[i]) < 0.10f) {
    ++b;
  }
  return unit(g[b] - g[a]);
}

// Doi khoang thoi gian sang giay
double seconds(Clock::duration d) {
  return std::chrono::duration<double>(d).count();
}

// ---- Duong theo chieu dai cung (cho MPC / oracle) ----
struct Arc {
  std::vector<cv::Point2f> p;
  std::vector<double> s;
  double len() const { return s.empty() ? 0.0 : s.back(); }
  cv::Point2f at(double q) const {
    size_t j = 1;
    while (j + 1 < s.size() && s[j] < q) {
      ++j;
    }
    const double span = std::max(1e-6, s[j] - s[j - 1]);
    const float r = static_cast<float>(std::clamp((q - s[j - 1]) / span, 0.0, 1.0));
    return p[j - 1] + (p[j] - p[j - 1]) * r;
  }
  // Huong (rad, > 0 = sang phai) cua day cung [q, q + 5 cm]
  double heading(double q) const {
    const cv::Point2f d = at(q + 0.05) - at(q);
    return std::atan2(static_cast<double>(d.x), static_cast<double>(d.y));
  }
};

// Duong tu diem start noi tiep poly[from..] (bo diem trung), kem chieu dai cung
Arc make_arc(const cv::Point2f &start, const std::vector<cv::Point2f> &poly, size_t from) {
  Arc a;
  a.p.push_back(start);
  a.s.push_back(0.0);
  for (size_t i = from; i < poly.size(); ++i) {
    const double d = norm2(poly[i] - a.p.back());
    if (d < 1e-4) {
      continue;
    }
    a.s.push_back(a.s.back() + d);
    a.p.push_back(poly[i]);
  }
  return a;
}

// Huong duong (rad) tai do sau z (toa do camera, z tang dan): day cung
// [z, z + 6 cm]. false neu duong khong phu / di ngang.
bool heading_at_z(const std::vector<cv::Point2f> &g, double z, double &h) {
  auto x_at = [&](double zz, float &x) {
    for (size_t i = 0; i + 1 < g.size(); ++i) {
      const cv::Point2f a = g[i], b = g[i + 1];
      if ((a.y - zz) * (b.y - zz) <= 0.0 && std::fabs(b.y - a.y) > 1e-5f) {
        x = a.x + static_cast<float>((zz - a.y) / (b.y - a.y)) * (b.x - a.x);
        return true;
      }
    }
    return false;
  };
  float x0 = 0.0f, x1 = 0.0f;
  if (!x_at(z, x0) || !x_at(z + 0.06, x1)) {
    return false;
  }
  h = std::atan2(static_cast<double>(x1 - x0), 0.06);
  return std::fabs(h) < 50.0 * kDeg;
}

// Oracle: cua so do cong that (s) va do sau tham chieu tren anh (m)
constexpr double LEARN_WIN_MIN_S = 0.40;
constexpr double LEARN_WIN_MAX_S = 0.70;
constexpr double LEARN_Z_REF_M = 0.30;
// MPC chi chay khi duong tam bat dau khong qua xa truc sau (ngoai suy)
constexpr float MPC_MAX_EXTRAP_M = 0.60f;

} // namespace

// Khoi tao: oracle bat dau tu g0, b0 khai bao
PathTracker::PathTracker(const VehicleParams &p) : p_(p) {
  oracle_.reset(p_.mpc_g0, p_.mpc_b0_deg * kDeg);
}

// Xoa bo nho duong + odometry (giu g, b da hoc)
void PathTracker::reset() {
  pose_ = Pose{};
  hist_.clear();
  centre_ = Memory{};
  left_ = Memory{};
  right_ = Memory{};
  started_ = false;
  turn_ = 0;
  curv_prev_ = 0.0;
  xte_i_deg_ = 0.0;
  last_compute_ = {};
  // Giu g, b da hoc (dac tinh xe, khong doi khi mat duong)
  d_est_rad_ = 0.0;
  mpc_s_int_ = 0.0;
  cmd_hist_.clear();
  obs_hist_.clear();
}

// ============================================================================
// ODOMETRY - mo hinh xe dap tai truc sau
//   dth = v / L * tan(delta) * dt;  x += v sin(th) dt;  z += v cos(th) dt
// ============================================================================

PathTracker::Pose PathTracker::step(const Pose &p, double v, double steer_rad,
                                    double L, double dt) {
  Pose o = p;
  const double dth = v / std::max(0.05, L) * std::tan(steer_rad) * dt;
  const double thm = p.th + 0.5 * dth;
  o.x += v * std::sin(thm) * dt;
  o.z += v * std::cos(thm) * dt;
  o.th += dth;
  return o;
}

// Odometry 100 Hz: tich phan vi tri xe theo toc do that va goc banh dang lenh
void PathTracker::predict(double v, double steer_deg, Clock::time_point now) {
  const double lim = p_.max_steer_deg * kDeg;
  const double steer = std::clamp(steer_deg * kDeg, -lim, lim);
  if (!started_) {
    started_ = true;
  } else {
    const double dt = std::clamp(seconds(now - now_), 0.0, 0.1);
    pose_ = step(pose_, v, steer, p_.wheelbase_m, dt);
  }
  // Goc banh dang co (uoc luong): lenh tre actuator_latency_s roi loc bac 1
  // mpc_tau_s. Dung lam trang thai MPC va de oracle hoc.
  cmd_hist_.emplace_back(now, steer);
  while (cmd_hist_.size() > 2 && seconds(now - cmd_hist_[1].first) > p_.actuator_latency_s) {
    cmd_hist_.pop_front();
  }
  if (started_ && now_ != Clock::time_point{}) {
    const double dt = std::clamp(seconds(now - now_), 0.0, 0.1);
    d_est_rad_ += (cmd_hist_.front().second - d_est_rad_) *
                  std::min(1.0, dt / std::max(1e-3, p_.mpc_tau_s));
  }
  now_ = now;
  last_v_ = v;
  last_steer_rad_ = steer;

  hist_.emplace_back(now, pose_);
  while (!hist_.empty() && seconds(now - hist_.front().first) > 2.0) {
    hist_.pop_front();
  }
}

// Vi tri xe tai thoi diem t (noi suy lich su 2 s) - dat frame camera dung luc chup
PathTracker::Pose PathTracker::pose_at(Clock::time_point t) const {
  if (hist_.empty() || t >= hist_.back().first) {
    return pose_;
  }
  if (t <= hist_.front().first) {
    return hist_.front().second;
  }
  for (size_t i = 1; i < hist_.size(); ++i) {
    if (hist_[i].first >= t) {
      const auto &a = hist_[i - 1];
      const auto &b = hist_[i];
      const double span = seconds(b.first - a.first);
      const double r = span > 1e-6 ? seconds(t - a.first) / span : 1.0;
      Pose o;
      o.x = a.second.x + r * (b.second.x - a.second.x);
      o.z = a.second.z + r * (b.second.z - a.second.z);
      o.th = a.second.th + r * (b.second.th - a.second.th);
      return o;
    }
  }
  return pose_;
}

// Toa do xe (tai vi tri p) -> toa do the gioi
cv::Point2f PathTracker::to_world(const Pose &p, const cv::Point2f &v) {
  const double c = std::cos(p.th);
  const double s = std::sin(p.th);
  return cv::Point2f(static_cast<float>(p.x + v.x * c + v.y * s),
                     static_cast<float>(p.z - v.x * s + v.y * c));
}

// Toa do the gioi -> toa do xe (tai vi tri p)
cv::Point2f PathTracker::to_vehicle(const Pose &p, const cv::Point2f &w) {
  const double c = std::cos(p.th);
  const double s = std::sin(p.th);
  const double dx = w.x - p.x;
  const double dz = w.y - p.z;
  return cv::Point2f(static_cast<float>(dx * c - dz * s),
                     static_cast<float>(dx * s + dz * c));
}

// ============================================================================
// BO NHO DUONG
// ----------------------------------------------------------------------------
// Doan moi (camera) thay the phan bo nho tu diem dau cua no tro ra truoc;
// phan bo nho nam SAU diem dau doan moi (vung mu giua xe va ROI) duoc giu va
// CAN CUNG (xoay + tinh tien) cho khop vi tri + huong doan moi tai moi noi:
// sai so odometry (toc do, ty so lai sai) duoc camera sua lai moi frame thay
// vi tich luy. Diem ngam pure pursuit thuong nam trong vung mu nen buoc nay
// quyet dinh do chinh xac khi vao cua.
// Frame lech han duong da nho (nhan nham 1 frame) bi bo qua; lech REJECT_RESET
// frame lien tiep thi tin frame moi.
// ============================================================================

void PathTracker::merge(Memory &mem, const std::vector<cv::Point2f> &cam,
                        const Pose &pc, Clock::time_point stamp) {
  if (cam.size() < 2) {
    return;
  }
  std::vector<cv::Point2f> neu;
  neu.reserve(cam.size());
  for (const auto &q : cam) {
    neu.push_back(to_world(
        pc, cv::Point2f(q.x, q.y + static_cast<float>(p_.cam_to_rear_m))));
  }

  auto accept_new = [&]() {
    mem.pts = std::move(neu);
    mem.seen = stamp;
    mem.reject = 0;
  };
  if (mem.pts.size() < 2) {
    accept_new();
    return;
  }

  const cv::Point2f p0 = neu.front();
  const cv::Point2f t0 = dir_at(neu, 0);

  // Khop: dau doan moi nam tren duong cu, hoac dau xa duong cu nam tren
  // doan moi (doan moi bat dau xa hon duong cu)
  float d_start = 0.0f;
  size_t seg = 0;
  const cv::Point2f q_near = closest_on(p0, mem.pts, d_start, &seg);
  float d_end = 0.0f;
  closest_on(mem.pts.back(), neu, d_end);
  if (std::min(d_start, d_end) > MATCH_TOL_M) {
    if (++mem.reject < REJECT_RESET) {
      return;
    }
    accept_new();
    return;
  }
  {
    // Do lech ngang tren phan chong len nhau (diem moi chieu vao duong cu,
    // khong tinh diem vuot qua 2 dau duong cu)
    std::vector<float> dev;
    for (const auto &q : neu) {
      float d = 0.0f;
      size_t sg = 0;
      const cv::Point2f c = closest_on(q, mem.pts, d, &sg);
      const bool end = (sg == 0 && norm2(c - mem.pts.front()) < 1e-4f) ||
                       (sg + 2 == mem.pts.size() && norm2(c - mem.pts.back()) < 1e-4f);
      if (!end) {
        dev.push_back(d);
      }
    }
    if (dev.size() >= 4) {
      std::nth_element(dev.begin(), dev.begin() + dev.size() / 2, dev.end());
      if (dev[dev.size() / 2] > JUMP_TOL_M) {
        if (++mem.reject < REJECT_RESET) {
          return;
        }
        accept_new();
        return;
      }
    }
  }

  // Can cung tai moi noi: q -> p0 + R(rot) * (q - q_near). rot = lech huong
  // giua duong cu va doan moi, chi sua 1 nua moi frame (loc nhieu detector).
  const bool joined = d_start <= MATCH_TOL_M;
  const cv::Point2f anchor = joined ? q_near : p0;
  double rot = 0.0;
  if (joined) {
    const cv::Point2f t_old = dir_at(mem.pts, seg);
    rot = 0.5 * std::clamp(std::atan2(static_cast<double>(t_old.x * t0.y - t_old.y * t0.x),
                                      static_cast<double>(t_old.dot(t0))),
                           -0.3, 0.3);
  }
  // Goc trong he (X phai, Z truoc) quay nguoc chieu kim dong ho theo cross
  const float cr = static_cast<float>(std::cos(rot));
  const float sr = static_cast<float>(std::sin(rot));

  std::vector<cv::Point2f> merged;
  merged.reserve(mem.pts.size() + neu.size());
  // Giu phan duong cu nam TRUOC moi noi theo thu tu doc duong (khong xet
  // theo huong t0: trong cua gat doan vung mu da quay 60-90 do so voi t0)
  const size_t keep_n = joined ? seg + 1 : 0;
  for (size_t i = 0; i < keep_n; ++i) {
    const cv::Point2f d = mem.pts[i] - anchor;
    if (norm2(d) < 0.03f) {
      continue; // trung voi diem dau doan moi
    }
    merged.push_back(p0 + cv::Point2f(cr * d.x - sr * d.y, sr * d.x + cr * d.y));
  }
  merged.insert(merged.end(), neu.begin(), neu.end());
  mem.pts = std::move(merged);
  mem.seen = stamp;
  mem.reject = 0;
}

// Bo duong da nho qua cu (MEMORY_TTL_S) hoac da o xa phia sau xe
void PathTracker::prune(Memory &mem) const {
  if (mem.pts.empty()) {
    return;
  }
  if (seconds(now_ - mem.seen) > MEMORY_TTL_S) {
    mem.pts.clear();
    return;
  }
  mem.pts.erase(std::remove_if(mem.pts.begin(), mem.pts.end(),
                               [&](const cv::Point2f &w) {
                                 return to_vehicle(pose_, w).y < -BEHIND_KEEP_M;
                               }),
                mem.pts.end());
  if (mem.pts.size() > 400) {
    mem.pts.erase(mem.pts.begin(), mem.pts.end() - 400);
  }
}

// Duong da nho doi sang toa do xe tai vi tri p
std::vector<cv::Point2f> PathTracker::in_vehicle(const Memory &mem,
                                                 const Pose &p) const {
  std::vector<cv::Point2f> out;
  out.reserve(mem.pts.size());
  for (const auto &w : mem.pts) {
    out.push_back(to_vehicle(p, w));
  }
  return out;
}

// Them 1 frame camera: dat dung vi tri xe luc chup, ghep vao bo nho, cho oracle hoc
void PathTracker::add_observation(const std::vector<cv::Point2f> &centre_cam,
                                  const std::vector<cv::Point2f> &left_cam,
                                  const std::vector<cv::Point2f> &right_cam,
                                  Clock::time_point stamp) {
  // Dat frame vao dung vi tri xe LUC CHUP
  const auto t_cap = stamp - std::chrono::microseconds(
                                 static_cast<long>(p_.camera_latency_s * 1e6));
  const Pose pc = pose_at(t_cap);
  merge(centre_, centre_cam, pc, stamp);
  merge(left_, left_cam, pc, stamp);
  merge(right_, right_cam, pc, stamp);
  if (p_.mpc_enable && p_.mpc_learn) {
    learn_from(centre_cam, stamp);
  }
}

// Oracle: do cong THAT cua xe tu 2 frame cach nhau ~0.5 s. Diem duong o
// z_ref + ds trong frame cu la diem o z_ref trong frame moi (chan camera di
// ds) -> xe quay duoc  h_cu(z_ref + ds) - h_moi(z_ref). L * kappa = g d + b.
void PathTracker::learn_from(const std::vector<cv::Point2f> &centre_cam, Clock::time_point stamp) {
  if (centre_cam.size() < 3) {
    obs_hist_.clear();
    return;
  }
  obs_hist_.push_back({stamp, centre_cam, d_est_rad_, last_v_});
  while (!obs_hist_.empty() && seconds(stamp - obs_hist_.front().t) > LEARN_WIN_MAX_S) {
    obs_hist_.pop_front();
  }
  const ObsRec &old = obs_hist_.front();
  const double win = seconds(stamp - old.t);
  if (win < LEARN_WIN_MIN_S) {
    return;
  }
  double d_mean = 0.0, v_mean = 0.0;
  for (const auto &r : obs_hist_) {
    d_mean += r.d;
    v_mean += r.v;
  }
  d_mean /= static_cast<double>(obs_hist_.size());
  v_mean /= static_cast<double>(obs_hist_.size());
  const double ds = v_mean * win;
  if (v_mean < 0.15 || ds < 0.08) {
    return;
  }
  double h_old = 0.0, h_new = 0.0;
  if (!heading_at_z(old.centre, LEARN_Z_REF_M + ds, h_old) ||
      !heading_at_z(centre_cam, LEARN_Z_REF_M, h_new)) {
    return;
  }
  const double dpsi = std::remainder(h_old - h_new, 2.0 * kPi);
  oracle_.update(d_mean, p_.wheelbase_m * dpsi / ds);
  // Mau tiep theo dung cua so moi (khong dem 1 doan 2 lan qua nhieu)
  obs_hist_.pop_front();
}

// ============================================================================
// LAI
// ============================================================================

// Khoang trong tu mep xe toi MEP bang keo tai hang z (toa do xe). side -1:
// vach trai, +1: vach phai. Am = da de len vach.
double PathTracker::line_clearance(const std::vector<cv::Point2f> &line_v,
                                   double z, double x_wheel, int side,
                                   bool &found) const {
  found = false;
  double best = std::numeric_limits<double>::max();
  double best_dx = std::numeric_limits<double>::max();
  for (size_t i = 0; i + 1 < line_v.size(); ++i) {
    const cv::Point2f a = line_v[i];
    const cv::Point2f b = line_v[i + 1];
    if ((a.y - z) * (b.y - z) > 0.0 || std::fabs(b.y - a.y) < 1e-6f) {
      continue;
    }
    const double r = (z - a.y) / (b.y - a.y);
    const double x = a.x + r * (b.x - a.x);
    const double c = side < 0 ? (x_wheel - x) : (x - x_wheel);
    // Nhieu giao diem (vach cong): lay giao diem gan banh nhat
    if (std::fabs(x - x_wheel) < best_dx) {
      best_dx = std::fabs(x - x_wheel);
      best = c - p_.car_half_width_m - p_.tape_half_m;
      found = true;
    }
  }
  return best;
}

// Tinh goc lai cho chu ky nay (MPC; du phong pure pursuit + lech ngang) + du doan cua
PathTracker::Output PathTracker::compute(double v, double bias_m) {
  Output o;
  prune(centre_);
  prune(left_);
  prune(right_);
  const double dt_c = last_compute_ == Clock::time_point{}
                          ? 0.0
                          : std::clamp(seconds(now_ - last_compute_), 0.0, 0.1);
  last_compute_ = now_;

  // Du doan vi tri xe khi lenh lai nay toi duoc servo
  Pose pp = pose_;
  for (double t = 0.0; t < p_.actuator_latency_s - 1e-9; t += 0.01) {
    pp = step(pp, v, last_steer_rad_, p_.wheelbase_m, 0.01);
  }

  o.centre_v = in_vehicle(centre_, pp);
  // Doi duong tam sang PHAI bias_m (theo phap tuyen): chi thay 1 vach thi
  // lai lech ve phia vach bi mat de tim lai no (SINGLE_LINE_SEARCH Python)
  if (std::fabs(bias_m) > 1e-4 && o.centre_v.size() >= 2) {
    std::vector<cv::Point2f> sh(o.centre_v.size());
    for (size_t i = 0; i < o.centre_v.size(); ++i) {
      const cv::Point2f t = dir_at(o.centre_v, std::min(i, o.centre_v.size() - 2));
      sh[i] = o.centre_v[i] + cv::Point2f(t.y, -t.x) * static_cast<float>(bias_m);
    }
    o.centre_v.swap(sh);
  }
  o.left_v = in_vehicle(left_, pp);
  o.right_v = in_vehicle(right_, pp);
  o.virt_left = !left_.pts.empty() && seconds(now_ - left_.seen) > VIRTUAL_AFTER_S;
  o.virt_right = !right_.pts.empty() && seconds(now_ - right_.seen) > VIRTUAL_AFTER_S;

  o.mpc_g = oracle_.g;
  o.mpc_b_deg = oracle_.b / kDeg;
  o.mpc_learn_n = oracle_.used;
  const std::vector<cv::Point2f> &c = o.centre_v;
  if (c.size() < 2) {
    // Khong con duong: xa dan khau I (khong giu sai lech cu qua lau)
    xte_i_deg_ *= std::exp(-dt_c / 2.0);
    mpc_s_int_ *= std::exp(-dt_c / 2.0);
    return o;
  }
  const double L = p_.wheelbase_m;

  // Chieu dai duong con o truoc banh truoc
  for (size_t i = 1; i < c.size(); ++i) {
    if (c[i - 1].y > L && c[i].y > L) {
      o.ahead_m += norm2(c[i] - c[i - 1]);
    }
  }

  // ---- Pure pursuit tu truc sau ----
  const double Ld = std::clamp(p_.lookahead_min_m + p_.lookahead_gain_s * v,
                               p_.lookahead_min_m, p_.lookahead_max_m);
  o.lookahead_m = Ld;
  bool have = false;
  cv::Point2f prev(0.0f, 0.0f);
  bool have_prev = false;
  for (const auto &q : c) {
    if (q.y <= 0.05f) {
      have_prev = false;
      continue;
    }
    const float d = norm2(q);
    if (d >= Ld) {
      if (have_prev && norm2(prev) < Ld) {
        const float dp = norm2(prev);
        const float t = static_cast<float>((Ld - dp) / std::max(1e-6f, d - dp));
        o.target_v = prev + (q - prev) * t;
      } else {
        o.target_v = q; // duong bat dau xa hon Ld
      }
      have = true;
      break;
    }
    prev = q;
    have_prev = true;
  }
  if (!have) {
    // Duong ngan hon Ld: ngam diem cuoi neu no con o phia truoc
    if (c.back().y <= 0.10f) {
      return o;
    }
    o.target_v = c.back();
  }
  const double Le = std::max(0.20, static_cast<double>(norm2(o.target_v)));
  const double kappa = 2.0 * o.target_v.x / (Le * Le);
  double steer = std::atan(L * kappa) / kDeg;
  // Trong cua (do cong phia truoc lan tinh truoc): danh lai manh hon
  if (p_.corner_gain != 1.0) {
    const double r = std::clamp((std::fabs(curv_prev_) - 0.4) / 0.6, 0.0, 1.0);
    steer *= 1.0 + (p_.corner_gain - 1.0) * r;
  }

  // ---- Phan hoi lech ngang tai CHAN CAMERA ----
  // Do o chan camera chu khong o banh truoc: lech ngang + huong duong tam
  // tai day la so do TRUC TIEP cua camera (chi ngoai suy ~0.13 m), khong phu
  // thuoc cam_to_rear_m / steer_ratio khai bao. Thu vong kin: khai bao 2 so
  // nay lech thuc te thi do o banh truoc (qua bo nho odometry) kem hon.
  if (p_.xte_gain > 0.0 || p_.xte_ki > 0.0) {
    float d = 0.0f;
    size_t seg = 0;
    const cv::Point2f fa(0.0f, static_cast<float>(p_.cam_to_rear_m));
    const cv::Point2f q = closest_on(fa, c, d, &seg);
    // Chi khi duong tam phu toi banh truoc (khong ngoai suy qua dau duong)
    const bool covered = c.front().y <= fa.y + 0.05;
    if (covered && d < 0.30f) {
      const cv::Point2f t = dir_at(c, seg);
      // > 0: duong tam nam ben PHAI banh truoc
      o.xte_m = (fa.x - q.x) * t.y - (fa.y - q.y) * t.x < 0.0f ? d : -d;
      o.xte_deg = std::atan(p_.xte_gain * o.xte_m / (std::max(0.0, v) + p_.xte_soft_mps)) / kDeg;
      // Tich phan chi khi xe dang chay (dung yen thi lech khong doi duoc)
      if (p_.xte_ki > 0.0 && v > 0.2) {
        xte_i_deg_ = std::clamp(xte_i_deg_ + p_.xte_ki * o.xte_m * dt_c,
                                -p_.xte_i_max_deg, p_.xte_i_max_deg);
      }
      o.xte_i_deg = xte_i_deg_;
      o.xte_deg += xte_i_deg_;
      steer += o.xte_deg;
    }
  }

  // ---- MPC giu lan (thay pure pursuit + lech ngang) ----
  if (p_.mpc_enable) {
    double ms = 0.0;
    if (mpc_steer(c, v, dt_c, o, ms)) {
      steer = ms;
      xte_i_deg_ = 0.0;
      o.xte_i_deg = 0.0;
    } else {
      // MPC khong chay duoc (duong chua phu toi truc sau...): xa dan tich phan
      mpc_s_int_ *= std::exp(-dt_c / 1.0);
    }
  }
  o.mpc_g = oracle_.g;
  o.mpc_b_deg = oracle_.b / kDeg;
  o.mpc_learn_n = oracle_.used;

  // ---- Rao chan vach (ke ca vach ao) tai banh truoc va 0.25 m truoc do ----
  double guard = 0.0;
  const struct {
    double dz;
    double w;
  } rows[] = {{0.0, 1.0}, {0.25, 0.6}};
  for (const auto &r : rows) {
    const double z = L + r.dz;
    bool f = false;
    const double cl = line_clearance(o.left_v, z, 0.0, -1, f);
    if (f && cl < p_.line_margin_m) {
      guard += r.w * p_.guard_gain_deg_per_m * (p_.line_margin_m - cl); // day sang phai
    }
    const double cr = line_clearance(o.right_v, z, 0.0, 1, f);
    if (f && cr < p_.line_margin_m) {
      guard -= r.w * p_.guard_gain_deg_per_m * (p_.line_margin_m - cr); // day sang trai
    }
  }
  o.guard_deg = std::clamp(guard, -GUARD_MAX_DEG, GUARD_MAX_DEG);
  steer += o.guard_deg;
  o.steer_deg = std::clamp(steer, -p_.max_steer_deg, p_.max_steer_deg);

  // ---- Du doan cua: do cong duong tam doan [banh truoc, +1.2 m] ----
  std::vector<cv::Point2f> fw;
  for (const auto &q : c) {
    if (q.y >= L && q.y <= L + 1.2) {
      fw.push_back(q);
    }
  }
  if (fw.size() >= 4) {
    const size_t n = fw.size();
    const size_t k = std::max<size_t>(1, n / 3);
    const cv::Point2f t_near = unit(fw[k] - fw[0]);
    const cv::Point2f t_far = unit(fw[n - 1] - fw[n - 1 - k]);
    double dth = std::atan2(t_far.x, t_far.y) - std::atan2(t_near.x, t_near.y);
    double arc = 0.0;
    for (size_t i = 1; i < n; ++i) {
      arc += norm2(fw[i] - fw[i - 1]);
    }
    o.curv_ahead = dth / std::max(0.15, arc * (1.0 - 1.0 / 3.0));
  }
  curv_prev_ = o.curv_ahead;

  // ---- Khoang cach toi CUA phia truoc (giam toc truoc khi vao cua) ----
  // curv_ahead la do cong TRUNG BINH ca doan 1.2 m: cua vua lo ra o cuoi tam
  // nhin bi pha loang voi doan thang truoc no -> truoc day xe chi giam toc khi
  // da sat cua, xe con dang troi (ESC chi nha ga, khong phanh) nen vot ra
  // ngoai cua. O day tim diem dau tien doc duong ma huong duong da quay
  // CORNER_TURN_DEG so voi huong duong tai banh truoc. Cung R 0.8 m: phat hien
  // tre ~0.1 m sau diem bat dau cong (bu bang corner_margin o node).
  {
    size_t i0 = 0;
    while (i0 < c.size() && c[i0].y < L) {
      ++i0;
    }
    std::vector<double> s_acc;
    for (size_t i = i0; i < c.size(); ++i) {
      s_acc.push_back(i == i0 ? 0.0 : s_acc.back() + norm2(c[i] - c[i - 1]));
    }
    auto at = [&](double s) {
      size_t j = 1;
      while (j + 1 < s_acc.size() && s_acc[j] < s) {
        ++j;
      }
      const double span = std::max(1e-6, s_acc[j] - s_acc[j - 1]);
      const float r = static_cast<float>(std::clamp((s - s_acc[j - 1]) / span, 0.0, 1.0));
      return c[i0 + j - 1] + (c[i0 + j] - c[i0 + j - 1]) * r;
    };
    // Diem trung binh +-5 cm quanh s (detector cho diem cach 5 cm): nhieu
    // vach o xa (~1 m) khong bao cua nham tren duong thang
    auto avg_at = [&](double s) {
      cv::Point2f m(0.0f, 0.0f);
      for (int k = -2; k <= 2; ++k) {
        m += at(std::clamp(s + 0.025 * k, 0.0, s_acc.back()));
      }
      return m * 0.2f;
    };
    auto heading = [&](double s) {
      const cv::Point2f d = avg_at(s + CORNER_CHORD_M) - avg_at(s);
      return std::atan2(static_cast<double>(d.x), static_cast<double>(d.y));
    };
    if (s_acc.size() >= 3 && s_acc.back() >= CORNER_CHORD_M + CORNER_STEP_M) {
      const double h0 = heading(0.0);
      for (double s = CORNER_STEP_M; s + CORNER_CHORD_M <= s_acc.back() + 1e-9;
           s += CORNER_STEP_M) {
        const double dh = std::remainder(heading(s) - h0, 2.0 * kPi);
        if (std::fabs(dh) >= CORNER_TURN_DEG * kDeg) {
          o.corner_dist_m = s;
          break;
        }
      }
    }
  }
  // Tre: vao trang thai cua khi |k| > 0.6, thoat khi < 0.3
  if (turn_ == 0) {
    if (o.curv_ahead > 0.6) {
      turn_ = 1;
    } else if (o.curv_ahead < -0.6) {
      turn_ = -1;
    }
  } else if (o.curv_ahead * turn_ < 0.3) {
    turn_ = 0;
  }
  o.turn = turn_;
  o.valid = true;
  return o;
}

// ============================================================================
// MPC (lk_mpc_solve, lane_keeping.hpp): trang thai tuong doi voi duong tam tai TRUC SAU, do cong
// duong theo quang duong xe se di trong tam du doan.
// ============================================================================
bool PathTracker::mpc_steer(const std::vector<cv::Point2f> &c, double v, double dt_c,
                            Output &o, double &steer_deg) {
  if (c.size() < 2) {
    return false;
  }
  float dq = 0.0f;
  size_t seg = 0;
  const cv::Point2f origin(0.0f, 0.0f);
  cv::Point2f q = closest_on(origin, c, dq, &seg);
  cv::Point2f t = dir_at(c, seg);
  if (seg == 0 && c.front().y > 0.0f) {
    // Duong bat dau truoc truc sau: keo dai doan dau
    if (c.front().y > MPC_MAX_EXTRAP_M) {
      return false;
    }
    q = c.front();
    t = unit(c[1] - c[0]);
  }
  if (dq > 0.40f && !(seg == 0 && c.front().y > 0.0f)) {
    return false;
  }
  // e_y > 0: xe ben PHAI duong tam; e_psi > 0: mui xe lech PHAI so voi duong
  const double ey = -q.x * t.y + q.y * t.x;
  const double epsi = -std::atan2(static_cast<double>(t.x), static_cast<double>(t.y));
  const Arc arc = make_arc(q, c, seg + 1);
  if (arc.len() < 0.15) {
    return false;
  }
  LkMpcCfg cfg;
  cfg.wheelbase = p_.wheelbase_m;
  cfg.tau = p_.mpc_tau_s;
  cfg.n = std::max(4, static_cast<int>(std::lround(p_.mpc_horizon_s / cfg.dt)));
  cfg.max_steer = p_.max_steer_deg * kDeg;
  cfg.zone_m = p_.mpc_zone_m;
  cfg.rd = p_.mpc_rd;
  cfg.q_i = p_.mpc_qi;
  cfg.i_max = p_.mpc_i_max;
  // LQI: tich phan lech ngang (m.s). Chi tich luy khi duong hop le, xe dang chay
  // va lech con nho (chong wind-up khi vua mat lan / lech lon).
  if (p_.mpc_qi > 0.0 && v > 0.15 && std::fabs(ey) < 0.15) {
    mpc_s_int_ = std::clamp(mpc_s_int_ + ey * dt_c, -p_.mpc_i_max, p_.mpc_i_max);
  } else {
    mpc_s_int_ *= std::exp(-dt_c / 1.0);
  }
  const double vm = std::max(0.15, v);
  std::vector<double> rho(static_cast<size_t>(cfg.n), 0.0);
  for (int k = 0; k < cfg.n; ++k) {
    const double sk = vm * cfg.dt * k;
    if (sk + 0.20 > arc.len()) {
      break; // het duong da biet: coi nhu thang
    }
    rho[static_cast<size_t>(k)] = std::remainder(arc.heading(sk + 0.15) - arc.heading(sk), 2.0 * kPi) / 0.15;
  }
  // Goc banh luc lenh nay toi servo (sau actuator_latency_s)
  const double dact = d_est_rad_ + (last_steer_rad_ - d_est_rad_) *
                                       (1.0 - std::exp(-p_.actuator_latency_s / std::max(1e-3, p_.mpc_tau_s)));
  const double u = lk_mpc_solve(ey, epsi, dact, last_steer_rad_, vm, rho, oracle_.g, oracle_.b,
                                cfg, mpc_s_int_);
  steer_deg = u / kDeg;
  o.mpc = true;
  o.mpc_ey_m = ey;
  o.mpc_epsi_deg = epsi / kDeg;
  return true;
}
