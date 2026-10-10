#include "path_tracker.hpp"

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
// Do lech ngang: duong tam bat dau truoc chan camera toi bay nhieu thi keo
// dai thang doan dau ve sau (camera thay mat dat tu ~0.47 m truoc chan cam)
constexpr float XTE_EXTEND_M = 0.50f;
// Lech ngang lon hon muc nay = bo nho sai, bo phan hoi xte. Truoc day 0.30:
// xe lech that 30-35 cm (de len vach, log 10/10 18:05) lai MAT khau keo ve.
constexpr float XTE_MAX_M = 0.45f;

inline float norm2(const cv::Point2f &v) { return std::sqrt(v.x * v.x + v.y * v.y); }

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

// Khop duong thang x = a + b * y (binh phuong toi thieu) qua cac diem duong
// tam trong [y0 - 0.1, y0 + len] (duong bat dau xa hon: lay doan len dau
// tien). e = lech ngang tai y0 (m, > 0 = ben phai), psi = huong (do, > 0 =
// duong re phai so voi xe). Can >= 4 diem trai >= 0.25 m.
bool fit_line(const std::vector<cv::Point2f> &c, float y0, float len, double &e, double &psi) {
  float lo = y0 - 0.1f;
  if (!c.empty() && c.front().y > lo) {
    lo = c.front().y;
  }
  const float hi = std::max(y0, lo) + len;
  double n = 0, sy = 0, sx = 0, syy = 0, sxy = 0;
  float ymin = 1e9f, ymax = -1e9f;
  for (const auto &q : c) {
    if (q.y < lo || q.y > hi) {
      continue;
    }
    n += 1;
    sy += q.y;
    sx += q.x;
    syy += static_cast<double>(q.y) * q.y;
    sxy += static_cast<double>(q.x) * q.y;
    ymin = std::min(ymin, q.y);
    ymax = std::max(ymax, q.y);
  }
  if (n < 4 || ymax - ymin < 0.25f || lo > y0 + 0.5f) {
    return false;
  }
  const double den = n * syy - sy * sy;
  if (std::fabs(den) < 1e-9) {
    return false;
  }
  const double b = (n * sxy - sy * sx) / den;
  const double a = (sx - b * sy) / n;
  e = a + b * y0;
  psi = std::atan(b) * 180.0 / CV_PI;
  return true;
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

double seconds(Clock::duration d) {
  return std::chrono::duration<double>(d).count();
}

// GOC GAP: huong duong truoc / sau diem s (day cung KINK_CHORD_M) lech >=
// KINK_MIN_DEG. Duong dua that la cac doan thang noi goc gap 30-90 do.
constexpr double KINK_CHORD_M = 0.12;
constexpr double KINK_MIN_DEG = 20.0;
constexpr double KINK_STEP_M = 0.02;
// Chi giu thang khi THAN XE dang song song doan truoc goc (lech < bay nhieu).
// Cua da giac = nhieu goc gap noi tiep: xe dang quay qua goc truoc thi KHONG
// tra lai thang vi goc sau.
constexpr double KINK_ALIGN_DEG = 12.0;

// Goc gap dau tien tren duong c (toa do xe, sau -> truoc) trong khoang
// [0, s_max] m tinh tu ngang truc sau. s_k: quang duong toi dinh, th: goc
// quay (rad, > 0 = re phai), vtx: dinh, h_in: huong doan truoc dinh.
bool find_kink(const std::vector<cv::Point2f> &c, double s_max, double &s_k,
               double &th, cv::Point2f &vtx, cv::Point2f &h_in) {
  std::vector<cv::Point2f> p;
  for (size_t i = 0; i < c.size(); ++i) {
    if (c[i].y < 0.0f) {
      continue;
    }
    if (p.empty() && i > 0 && c[i - 1].y < 0.0f) {
      const float t = -c[i - 1].y / std::max(1e-6f, c[i].y - c[i - 1].y);
      p.push_back(c[i - 1] + (c[i] - c[i - 1]) * t); // diem ngang truc sau
    }
    p.push_back(c[i]);
  }
  if (p.size() < 3) {
    return false;
  }
  std::vector<double> acc(p.size(), 0.0);
  for (size_t i = 1; i < p.size(); ++i) {
    acc[i] = acc[i - 1] + norm2(p[i] - p[i - 1]);
  }
  auto at = [&](double s) {
    size_t j = 1;
    while (j + 1 < p.size() && acc[j] < s) {
      ++j;
    }
    const double span = std::max(1e-6, acc[j] - acc[j - 1]);
    const float r = static_cast<float>(std::clamp((s - acc[j - 1]) / span, 0.0, 1.0));
    return p[j - 1] + (p[j] - p[j - 1]) * r;
  };
  auto turn = [&](double s) {
    const cv::Point2f a = unit(at(s) - at(s - KINK_CHORD_M));
    const cv::Point2f b = unit(at(s + KINK_CHORD_M) - at(s));
    return std::atan2(static_cast<double>(a.y * b.x - a.x * b.y),
                      static_cast<double>(a.dot(b)));
  };
  const double end = std::min(s_max, acc.back() - KINK_CHORD_M);
  for (double s = KINK_CHORD_M; s <= end + 1e-9; s += KINK_STEP_M) {
    double t = turn(s);
    if (std::fabs(t) < KINK_MIN_DEG * kDeg) {
      continue;
    }
    // Dinh = noi goc quay lon nhat trong ~1 day cung tiep theo
    s_k = s;
    for (double u = s + KINK_STEP_M; u <= std::min(s + KINK_CHORD_M, end) + 1e-9;
         u += KINK_STEP_M) {
      const double tu = turn(u);
      if (std::fabs(tu) > std::fabs(t)) {
        t = tu;
        s_k = u;
      }
    }
    th = t;
    vtx = at(s_k);
    h_in = unit(vtx - at(std::max(0.0, s_k - 2.0 * KINK_CHORD_M)));
    return true;
  }
  return false;
}

} // namespace

PathTracker::PathTracker(const VehicleParams &p) : p_(p) {}

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
  pid_primed_ = false;
  pid_i_deg_ = 0.0;
  last_compute_ = {};
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

void PathTracker::predict(double v, double steer_deg, Clock::time_point now) {
  const double lim = p_.max_steer_deg * kDeg;
  const double steer = std::clamp(steer_deg * kDeg, -lim, lim);
  if (!started_) {
    started_ = true;
  } else {
    const double dt = std::clamp(seconds(now - now_), 0.0, 0.1);
    pose_ = step(pose_, v, steer, p_.wheelbase_m, dt);
  }
  now_ = now;
  last_v_ = v;
  last_steer_rad_ = steer;

  hist_.emplace_back(now, pose_);
  while (!hist_.empty() && seconds(now - hist_.front().first) > 2.0) {
    hist_.pop_front();
  }
}

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

cv::Point2f PathTracker::to_world(const Pose &p, const cv::Point2f &v) {
  const double c = std::cos(p.th);
  const double s = std::sin(p.th);
  return cv::Point2f(static_cast<float>(p.x + v.x * c + v.y * s),
                     static_cast<float>(p.z - v.x * s + v.y * c));
}

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

std::vector<cv::Point2f> PathTracker::in_vehicle(const Memory &mem,
                                                 const Pose &p) const {
  std::vector<cv::Point2f> out;
  out.reserve(mem.pts.size());
  for (const auto &w : mem.pts) {
    out.push_back(to_vehicle(p, w));
  }
  return out;
}

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

PathTracker::Output PathTracker::compute(double v, double bias_m) {
  Output o;
  prune(centre_);
  prune(left_);
  prune(right_);
  const double dt_c = last_compute_ == Clock::time_point{}
                          ? 0.0
                          : std::clamp(seconds(now_ - last_compute_), 0.0, 0.1);
  last_compute_ = now_;

  // Du doan vi tri xe khi lenh lai nay toi duoc servo (toc do odometry =
  // toc do THAT uoc luong, khong phai v lenh dung cho Ld / xte)
  Pose pp = pose_;
  for (double t = 0.0; t < p_.actuator_latency_s - 1e-9; t += 0.01) {
    pp = step(pp, last_v_, last_steer_rad_, p_.wheelbase_m, 0.01);
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

  const std::vector<cv::Point2f> &c = o.centre_v;
  if (c.size() < 2) {
    // Khong con duong: xa dan khau I (khong giu sai lech cu qua lau)
    xte_i_deg_ *= std::exp(-dt_c / 2.0);
    pid_i_deg_ *= std::exp(-dt_c / 2.0);
    pid_primed_ = false;
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
  double Ld = std::clamp(p_.lookahead_min_m + p_.lookahead_gain_s * v,
                         p_.lookahead_min_m, p_.lookahead_max_m);
  // TRONG CUA nhin GAN hon: diem ngam xa ~0.7 m giua cua da nam tren doan
  // thang sau cua -> xe lai thang toi do, cat goc, de vach trong. Duong
  // thang giu Ld (khong lac). Ramp theo do cong phia truoc 0.4 -> 1.0 1/m.
  if (p_.lookahead_corner_scale < 1.0) {
    const double r = std::clamp((std::fabs(curv_prev_) - 0.4) / 0.6, 0.0, 1.0);
    const double lc = std::max(Ld * p_.lookahead_corner_scale, p_.lookahead_min_m);
    Ld += (lc - Ld) * r;
  }
  o.lookahead_m = Ld;
  // GOC GAP phia truoc: xe re theo cung ban kinh kink_radius_m chi can bat
  // dau be lai khi truc sau con cach dinh d_in = R * tan(goc / 2) (30 do:
  // 0.27 m, 45: 0.41, 60: 0.58). Pure pursuit thuan ngam diem cach Ld ~0.65
  // m nen thay dinh trong Ld la be lai NGAY -> vao cua som, cat goc, de vach
  // trong (chay that 10/10). Chua toi diem vao cua: diem ngam di THANG theo
  // doan truoc dinh (khong vuot qua dinh).
  bool kink_hold = false;
  cv::Point2f kink_vtx, kink_hin;
  if (p_.kink_radius_m > 0.0) {
    double s_k = 0.0, th = 0.0;
    if (find_kink(c, Ld + 0.3, s_k, th, kink_vtx, kink_hin)) {
      o.kink_dist_m = s_k;
      o.kink_deg = th / kDeg;
      const double d_in = p_.kink_radius_m * std::tan(0.5 * std::min(std::fabs(th), 100.0 * kDeg));
      const double ang_in = std::atan2(static_cast<double>(kink_hin.x),
                                       static_cast<double>(kink_hin.y));
      kink_hold = s_k > d_in && s_k < Ld && std::fabs(ang_in) < KINK_ALIGN_DEG * kDeg;
      o.kink_hold = kink_hold;
    }
  }
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
  if (kink_hold) {
    o.target_v = kink_vtx + kink_hin * static_cast<float>(Ld - o.kink_dist_m);
  }
  const double Le = std::max(0.20, static_cast<double>(norm2(o.target_v)));
  const double kappa = 2.0 * o.target_v.x / (Le * Le);
  double steer = std::atan(L * kappa) / kDeg;
  // Trong so DUONG THANG cho PID: 1 = thang, 0 = cua (do cong phia truoc
  // 0.4 -> 1.0 1/m nhu corner_gain, hoac thay goc gap phia truoc)
  double straight_w = 1.0 - std::clamp((std::fabs(curv_prev_) - 0.4) / 0.6, 0.0, 1.0);
  if (o.kink_dist_m >= 0.0) {
    straight_w = 0.0;
  }
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
    cv::Point2f q = closest_on(fa, c, d, &seg);
    // Duong tam chua phu toi chan camera: KEO DAI THANG doan dau duong ve
    // sau toi da XTE_EXTEND_M. Bo nho bi thay bang frame moi (dau duong o
    // ~0.3 m truoc chan camera) ~14% frame (log 10/10 18:05) -> truoc day
    // mat ca P + I cua frame do (I bao hoa 6 do roi ve 0) -> lai giat
    // +-8 do moi vai frame, xe lech 15-20 cm ve tam rat cham.
    bool covered = c.front().y <= fa.y + 0.05;
    if (!covered && c.front().y <= fa.y + XTE_EXTEND_M) {
      const cv::Point2f t0 = dir_at(c, 0);
      if (t0.y > 0.5f) {
        const float s = (c.front().y - fa.y) / t0.y;
        const cv::Point2f a = c.front() - t0 * s;
        q = a + t0 * (fa - a).dot(t0);
        d = norm2(fa - q);
        seg = 0;
        covered = true;
      }
    }
    if (covered && d < XTE_MAX_M) {
      const cv::Point2f t = dir_at(c, seg);
      // > 0: duong tam nam ben PHAI banh truoc
      o.xte_m = (fa.x - q.x) * t.y - (fa.y - q.y) * t.x < 0.0f ? d : -d;
      // Lech xa hon xte_far_m: phan vuot nhan them xte_far_gain -> ve tam
      // nhanh hon; lech nho giu nguyen (khong lac tren duong thang)
      const double far = std::max(0.0, std::fabs(o.xte_m) - p_.xte_far_m);
      const double e = o.xte_m + std::copysign(p_.xte_far_gain / std::max(1e-6, p_.xte_gain) * far, o.xte_m);
      o.xte_deg = std::atan(p_.xte_gain * e / (std::max(0.0, v) + p_.xte_soft_mps)) / kDeg;
      // Tich phan chi khi xe dang chay (dung yen thi lech khong doi duoc).
      // Duong thang do PID lo -> I nay chi tich trong cua, ra thang xa dan
      // (log 19:44: bao hoa -6 do suot 40 s, day xe lech 1 ben)
      if (straight_w > 0.5 && p_.pid_kp_deg_per_m > 0.0) {
        xte_i_deg_ *= std::exp(-dt_c / 1.0);
      } else if (p_.xte_ki > 0.0 && v > 0.2) {
        xte_i_deg_ = std::clamp(xte_i_deg_ + p_.xte_ki * o.xte_m * dt_c,
                                -p_.xte_i_max_deg, p_.xte_i_max_deg);
      }
      o.xte_i_deg = xte_i_deg_;
      o.xte_deg += xte_i_deg_;
      steer += o.xte_deg;
    }
  }

  // ---- PID duong thang (xem VehicleParams::pid_*) ----
  if (p_.pid_kp_deg_per_m > 0.0) {
    const float y0 = static_cast<float>(p_.cam_to_rear_m);
    double e = 0.0, psi = 0.0;
    const bool ok = fit_line(c, y0, static_cast<float>(p_.pid_fit_m), e, psi);
    if (ok) {
      if (!pid_primed_) {
        pid_e_f_ = e;
        pid_psi_f_ = psi;
        pid_primed_ = true;
      } else {
        const double a = p_.pid_filter_s > 0.0 ? std::min(1.0, dt_c / p_.pid_filter_s) : 1.0;
        pid_e_f_ += a * (e - pid_e_f_);
        pid_psi_f_ += a * (psi - pid_psi_f_);
      }
      // Xe lech huong nhieu so voi duong = dang trong cua gap (giua cua do
      // cong phia truoc ~0 vi doan ke tiep nam cheo, xem sharp mode) -> tra ve
      // pure pursuit: <= 10 do PID het, >= 20 do pure pursuit het. Replay
      // 16:43: 28% frame "thang" theo do cong co |psi| > 30 do, 59% het lai.
      const double w = straight_w * std::clamp((20.0 - std::fabs(pid_psi_f_)) / 10.0, 0.0, 1.0);
      // Tich phan chi tren duong thang, xe gan tam va gan song song (khong
      // tich khi dang ve tam sau cua -> khong bao hoa nhu xte_ki)
      if (p_.pid_ki > 0.0 && v > 0.2 && w > 0.8 &&
          std::fabs(pid_e_f_) < 0.06 && std::fabs(pid_psi_f_) < 8.0) {
        pid_i_deg_ = std::clamp(pid_i_deg_ + p_.pid_ki * pid_e_f_ * dt_c,
                                -p_.pid_i_max_deg, p_.pid_i_max_deg);
      } else if (w < 0.5) {
        pid_i_deg_ *= std::exp(-dt_c / 2.0);
      }
      const double u = std::clamp(p_.pid_kp_deg_per_m * pid_e_f_ + p_.pid_kd * pid_psi_f_ + pid_i_deg_,
                                  -p_.pid_max_deg, p_.pid_max_deg);
      o.pid_w = w;
      o.pid_e_m = pid_e_f_;
      o.pid_psi_deg = pid_psi_f_;
      o.pid_i_deg = pid_i_deg_;
      o.pid_deg = u;
      steer = w * u + (1.0 - w) * steer;
    } else {
      pid_primed_ = false;
    }
  }

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
