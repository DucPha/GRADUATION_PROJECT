// ============================================================================
// NHAN LAN TREN MAT SAN (BEV) - port tu lane_bev.py (ban Python da chay tren xe)
// ----------------------------------------------------------------------------
// Xem tong quan o dau camera_node.hpp. File nay chua toan bo detector:
// luoi BEV, mask vach, loc do vat, do vach theo goc gap, ghep cap, hieu chinh
// goc cui, duong tam (mitre), pure pursuit va anh quan sat.
// ============================================================================

#include "camera_node.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <limits>
#include <numeric>
#include <string>

namespace {

using Poly = std::vector<cv::Point2f>;

constexpr float kDeg = static_cast<float>(CV_PI) / 180.0f;

inline float norm2(const cv::Point2f &v) { return std::sqrt(v.x * v.x + v.y * v.y); }

// > 0: b nam ben TRAI a (nhin theo huong a); < 0: ben PHAI
inline float cross2(const cv::Point2f &a, const cv::Point2f &b) {
  return a.x * b.y - a.y * b.x;
}

inline cv::Point2f unit(const cv::Point2f &v) {
  const float n = norm2(v);
  return n > 1e-6f ? v * (1.0f / n) : cv::Point2f(0.0f, 1.0f);
}

// 0 khi v <= lo, 1 khi v >= hi
inline float ramp(float v, float lo, float hi) {
  return std::clamp((v - lo) / (hi - lo), 0.0f, 1.0f);
}

// Goc huong (rad) so voi phia truoc xe: > 0 = huong sang phai
inline float heading_of(const cv::Point2f &t) { return std::atan2(t.x, t.y); }

// Quay vector d mot goc deg NGUOC chieu kim dong ho (sang trai) trong (X, Z)
inline cv::Point2f rotate(const cv::Point2f &d, float deg) {
  const float c = std::cos(deg * kDeg);
  const float s = std::sin(deg * kDeg);
  return cv::Point2f(d.x * c - d.y * s, d.x * s + d.y * c);
}

float poly_len(const Poly &g) {
  float s = 0.0f;
  for (size_t i = 1; i < g.size(); ++i) {
    s += norm2(g[i] - g[i - 1]);
  }
  return s;
}

// Chia lai polyline thanh diem cach deu step (m), khong lam tron
Poly resample_lin(const Poly &p, float step) {
  if (p.size() < 2) {
    return p;
  }
  Poly r;
  r.push_back(p.front());
  float carry = 0.0f;
  for (size_t i = 1; i < p.size(); ++i) {
    const cv::Point2f a = p[i - 1];
    const cv::Point2f d = p[i] - a;
    const float L = norm2(d);
    float pos = step - carry;
    while (pos <= L) {
      r.push_back(a + d * (pos / std::max(L, 1e-6f)));
      pos += step;
    }
    carry = L - (pos - step);
  }
  if (norm2(r.back() - p.back()) > 0.3f * step) {
    r.push_back(p.back());
  }
  return r;
}

// Douglas-Peucker: giu lai cac dinh (goc gap) cua vach
Poly simplify(const Poly &p, float eps) {
  if (p.size() < 3) {
    return p;
  }
  Poly out;
  cv::approxPolyDP(p, out, eps, false);
  return out;
}

// Huong dau vach (diem 0 -> diem n)
cv::Point2f init_dir(const Poly &p, size_t n = 6) {
  if (p.size() < 2) {
    return cv::Point2f(0.0f, 1.0f);
  }
  return unit(p[std::min(p.size() - 1, n)] - p[0]);
}

// Doi polyline sang PHAI (side = +1) / TRAI (-1) mot doan dist, noi goc kieu
// MITRE -> o goc gap 90 do, duong doi la dung tam lan cua goc (khong cat goc)
Poly offset_mitre(const Poly &p, float dist, int side, float eps) {
  if (p.size() < 2) {
    return {};
  }
  const Poly v = simplify(p, eps);
  if (v.size() < 2) {
    return {};
  }
  const size_t n = v.size();
  std::vector<cv::Point2f> nrm(n - 1);
  for (size_t i = 0; i + 1 < n; ++i) {
    const cv::Point2f s = unit(v[i + 1] - v[i]);
    nrm[i] = cv::Point2f(s.y, -s.x); // phap tuyen ben phai
  }
  const float k = static_cast<float>(side) * dist;
  Poly out(n);
  out[0] = v[0] + k * nrm[0];
  out[n - 1] = v[n - 1] + k * nrm[n - 2];
  for (size_t i = 1; i + 1 < n; ++i) {
    cv::Point2f m = nrm[i - 1] + nrm[i];
    const float mn = norm2(m);
    if (mn < 1e-6f) { // gap nguoc 180 do
      out[i] = v[i] + k * nrm[i];
      continue;
    }
    m *= 1.0f / mn;
    out[i] = v[i] + k * m * (1.0f / std::max(0.35f, m.dot(nrm[i - 1])));
  }
  // Doan vach ngan + gap gat (vach bi mep tam nhin cat) lam duong doi tu cat
  // chinh no thanh vong "tam giac": diem nao gan vach hon 0.7 * dist la diem
  // cua vong do -> bo
  Poly clean;
  for (const auto &q : out) {
    float best = std::numeric_limits<float>::max();
    for (size_t i = 0; i + 1 < n; ++i) {
      const cv::Point2f ab = v[i + 1] - v[i];
      const float L2 = std::max(ab.dot(ab), 1e-12f);
      const float t = std::clamp((q - v[i]).dot(ab) / L2, 0.0f, 1.0f);
      best = std::min(best, norm2(v[i] + ab * t - q));
    }
    if (best >= 0.7f * dist) {
      clean.push_back(q);
    }
  }
  return clean.size() >= 2 ? clean : Poly{};
}

// Vi tri (m tinh doc polyline) cua hinh chieu diem p len poly
float arc_pos(const cv::Point2f &p, const Poly &poly) {
  float best = std::numeric_limits<float>::max();
  float pos = 0.0f;
  float acc = 0.0f;
  for (size_t i = 0; i + 1 < poly.size(); ++i) {
    const cv::Point2f ab = poly[i + 1] - poly[i];
    const float L2 = std::max(ab.dot(ab), 1e-12f);
    const float t = std::clamp((p - poly[i]).dot(ab) / L2, 0.0f, 1.0f);
    const float d = norm2(poly[i] + ab * t - p);
    const float L = std::sqrt(L2);
    if (d < best) {
      best = d;
      pos = acc + t * L;
    }
    acc += L;
  }
  return pos;
}

float median_of(std::vector<float> v) {
  if (v.empty()) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
  return v[v.size() / 2];
}

// Phan vi q (0..1), noi suy tuyen tinh nhu numpy
float percentile(std::vector<float> v, float q) {
  std::sort(v.begin(), v.end());
  const float pos = q * static_cast<float>(v.size() - 1);
  const size_t lo = static_cast<size_t>(pos);
  const size_t hi = std::min(lo + 1, v.size() - 1);
  return v[lo] + (v[hi] - v[lo]) * (pos - static_cast<float>(lo));
}

} // namespace

// ============================================================================
// HINH HOC CAMERA PINHOLE (cao h, cui pitch, khong nghieng ngang)
// ============================================================================

bool CameraLane::to_image(double pitch, double f, double h, double cx,
                          double cy, double X, double Z, double &u, double &v) {
  const double c = std::cos(pitch);
  const double s = std::sin(pitch);
  const double yc = h * c - Z * s;
  const double zc = h * s + Z * c;
  if (zc <= 1e-3) {
    return false;
  }
  u = cx + f * X / zc;
  v = cy + f * yc / zc;
  return true;
}

cv::Point2f CameraLane::to_ground(double pitch, double f, double h, double cx,
                                  double cy, double u, double v) {
  const double rx = (u - cx) / f;
  const double ry = (v - cy) / f;
  const double c = std::cos(pitch);
  const double s = std::sin(pitch);
  const double du = -ry * c - s;
  const double dz = -ry * s + c;
  const double t = h / std::max(-du, 1e-6);
  return cv::Point2f(static_cast<float>(rx * t), static_cast<float>(dz * t));
}

double CameraLane::ground_dist_m(double y) const {
  // goc nhin xuong so voi phuong ngang cua tia qua hang y
  const double psi = pitch_ + std::atan((y - cy_) / f_px_);
  return psi > 1e-3 ? h_ / std::tan(psi)
                    : std::numeric_limits<double>::infinity();
}

bool CameraLane::ground_to_img(const cv::Point2f &g, cv::Point2d &p) const {
  double u = 0.0;
  double v = 0.0;
  if (!to_image(pitch_, f_px_, h_, WORK_W / 2.0, cy_, g.x, g.y, u, v)) {
    return false;
  }
  p = cv::Point2d(u, v);
  return true;
}

// ============================================================================
// LUOI BEV: bang tra o luoi mat san -> pixel khung lam viec
// ============================================================================

void CameraLane::build_bev() {
  if (f_px_ <= 0.0) {
    return;
  }
  const float roi_top = roi_top_frac_.load();
  roi_top_used_ = roi_top;
  const double v_top = std::clamp(static_cast<double>(roi_top) * work_h_, 0.0,
                                  work_h_ - 10.0);
  const double v_bot =
      std::clamp(static_cast<double>(roi_bottom_frac_) * work_h_, v_top + 8.0,
                 static_cast<double>(work_h_ - 1));

  const double z_far = ground_dist_m(v_top);
  const double z_near = ground_dist_m(v_bot);
  z_min_ = static_cast<float>(std::max(0.02, z_near - 0.03));
  z_max_ = static_cast<float>(std::min<double>(BEV_Z_MAX_M, z_far + 0.02));
  if (!(z_max_ > z_min_ + 0.2f)) {
    z_max_ = z_min_ + 0.2f;
  }
  x_min_ = -BEV_X_HALF_M;
  nx_ = static_cast<int>(std::lround(2.0f * BEV_X_HALF_M / BEV_RES_M));
  nz_ = static_cast<int>(std::lround((z_max_ - z_min_) / BEV_RES_M));
  z_max_ = z_min_ + nz_ * BEV_RES_M;

  mapx_.create(nz_, nx_, CV_32F);
  mapy_.create(nz_, nx_, CV_32F);
  valid_ = cv::Mat::zeros(nz_, nx_, CV_8U);
  const double cx = WORK_W / 2.0;
  for (int r = 0; r < nz_; ++r) {
    float *mx = mapx_.ptr<float>(r);
    float *my = mapy_.ptr<float>(r);
    uchar *ok = valid_.ptr<uchar>(r);
    const double Z = z_max_ - (r + 0.5) * BEV_RES_M;
    for (int c = 0; c < nx_; ++c) {
      const double X = x_min_ + (c + 0.5) * BEV_RES_M;
      double u = -1.0;
      double v = -1.0;
      const bool in = to_image(pitch_, f_px_, h_, cx, cy_, X, Z, u, v) &&
                      u >= 1.0 && u <= WORK_W - 2.0 && v >= v_top && v <= v_bot;
      ok[c] = in ? 255 : 0;
      mx[c] = static_cast<float>(std::clamp(u, 0.0, WORK_W - 1.0));
      my[c] = static_cast<float>(std::clamp(v, 0.0, work_h_ - 1.0));
    }
  }
  // Bo vien 2 o: mep vung nhin co gia tri noi suy tu ngoai anh
  cv::erode(valid_, valid_, cv::Mat::ones(5, 5, CV_8U), cv::Point(-1, -1), 1,
            cv::BORDER_CONSTANT, cv::Scalar(0));
  n_valid_ = std::max(1, cv::countNonZero(valid_));

  // Z gan nhat nhin thay theo tung cot (mep duoi tam nhin)
  z_near_col_.assign(static_cast<size_t>(nx_),
                     std::numeric_limits<float>::infinity());
  for (int c = 0; c < nx_; ++c) {
    for (int r = nz_ - 1; r >= 0; --r) {
      if (valid_.at<uchar>(r, c)) {
        z_near_col_[static_cast<size_t>(c)] = z_max_ - (r + 0.5f) * BEV_RES_M;
        break;
      }
    }
  }
}

void CameraLane::px_of(const cv::Point2f &g, float &c, float &r) const {
  c = (g.x - x_min_) / BEV_RES_M - 0.5f;
  r = (z_max_ - g.y) / BEV_RES_M - 0.5f;
}

cv::Point2f CameraLane::ground_of(float c, float r) const {
  return cv::Point2f(x_min_ + (c + 0.5f) * BEV_RES_M,
                     z_max_ - (r + 0.5f) * BEV_RES_M);
}

bool CameraLane::inside(const cv::Point2f &g) const {
  float c = 0.0f;
  float r = 0.0f;
  px_of(g, c, r);
  const int ci = static_cast<int>(std::lround(c));
  const int ri = static_cast<int>(std::lround(r));
  return ci >= 0 && ci < nx_ && ri >= 0 && ri < nz_ && valid_.at<uchar>(ri, ci);
}

// ============================================================================
// MASK VACH TOI (giong _line_mask cua Python)
// ----------------------------------------------------------------------------
//   nen  = closing (gian roi co) 13 cm -> lap vach, giu vet sang lon
//   rel  = (nen - anh) / nen: vach toi hon nen theo TI LE (khong theo hieu
//          tuyet doi: duoi den manh camera giam phoi sang)
//   san  = phan vi 40% tung khoi 40x20 cm: vach phai toi hon MUC SAN xung
//          quanh 10% -> khe san giua 2 vet chieu sang chi toi hon nen cuc bo
//          bi loai (nguyen nhan chinh sinh vach gia khi den manh)
// ============================================================================

// Chi lay pixel TRONG vung nhin thay (valid_). Ngoai vung nhin anh BEV la
// pixel mep anh bi keo dai (BORDER_REPLICATE) - toi, soc; truoc day bi tinh
// chung -> o gan xe (luoi BEV hep) muc san tut xuong ~1/2 san that -> vach
// khong "toi hon san" -> mat ca doan vach gan xe du anh goc thay ro.
cv::Mat CameraLane::floor_level(const cv::Mat &gray) const {
  const int h = gray.rows;
  const int w = gray.cols;
  const int bh = std::max(4, static_cast<int>(std::lround(FLOOR_BLOCK_H_M / BEV_RES_M)));
  const int bw = std::max(8, static_cast<int>(std::lround(FLOOR_BLOCK_W_M / BEV_RES_M)));
  const int nby = (h + bh - 1) / bh;
  const int nbx = std::max(1, w / bw);
  const bool use_valid = valid_.size() == gray.size();
  // Muc san chung (moi pixel nhin thay) cho khoi gan nhu nam ngoai vung nhin
  std::vector<uchar> all;
  all.reserve(static_cast<size_t>(n_valid_));
  for (int y = 0; y < h; ++y) {
    const uchar *g = gray.ptr<uchar>(y);
    const uchar *ok = use_valid ? valid_.ptr<uchar>(y) : nullptr;
    for (int x = 0; x < w; ++x) {
      if (!ok || ok[x]) {
        all.push_back(g[x]);
      }
    }
  }
  float global = 128.0f;
  if (!all.empty()) {
    const size_t k = static_cast<size_t>(FLOOR_PCT * (all.size() - 1));
    std::nth_element(all.begin(), all.begin() + k, all.end());
    global = all[k];
  }
  cv::Mat level(nby, nbx, CV_32F);
  std::vector<uchar> buf;
  for (int by = 0; by < nby; ++by) {
    for (int bx = 0; bx < nbx; ++bx) {
      buf.clear();
      const int y1 = std::min(h, (by + 1) * bh);
      const int x1 = bx == nbx - 1 ? w : (bx + 1) * bw;
      for (int y = by * bh; y < y1; ++y) {
        const uchar *g = gray.ptr<uchar>(y);
        const uchar *ok = use_valid ? valid_.ptr<uchar>(y) : nullptr;
        for (int x = bx * bw; x < x1; ++x) {
          if (!ok || ok[x]) {
            buf.push_back(g[x]);
          }
        }
      }
      // It pixel nhin thay (mep vung nhin): phan vi khong dang tin -> muc chung
      const size_t need = static_cast<size_t>((y1 - by * bh) * (x1 - bx * bw)) / 5;
      if (buf.size() < std::max<size_t>(20, need)) {
        level.at<float>(by, bx) = global;
        continue;
      }
      const size_t k = static_cast<size_t>(FLOOR_PCT * (buf.size() - 1));
      std::nth_element(buf.begin(), buf.begin() + k, buf.end());
      level.at<float>(by, bx) = buf[k];
    }
  }
  cv::Mat out;
  cv::resize(level, out, gray.size(), 0, 0, cv::INTER_LINEAR);
  return out;
}

void CameraLane::line_mask(const cv::Mat &bev_bgr, cv::Mat &mask,
                           cv::Mat &rel, cv::Mat &glare) const {
  cv::Mat gray;
  cv::cvtColor(bev_bgr, gray, cv::COLOR_BGR2GRAY);
  cv::GaussianBlur(gray, gray, cv::Size(3, 3), 0);

  const int k = std::max(5, static_cast<int>(std::lround(BG_KERNEL_M / BEV_RES_M))) | 1;
  cv::Mat bg;
  cv::morphologyEx(gray, bg, cv::MORPH_CLOSE,
                   cv::getStructuringElement(cv::MORPH_RECT, cv::Size(k, k)),
                   cv::Point(-1, -1), 1, cv::BORDER_REPLICATE);
  cv::Mat diff;
  cv::subtract(bg, gray, diff);
  cv::divide(diff, bg, rel, 255.0);

  cv::Mat m_rel, m_abs;
  cv::threshold(rel, m_rel, dark_ratio_ * 255.0, 255, cv::THRESH_BINARY);
  cv::threshold(diff, m_abs, DARK_MIN_ABS - 1, 255, cv::THRESH_BINARY);
  cv::bitwise_and(m_rel, m_abs, mask);

  const cv::Mat level = floor_level(gray);
  cv::Mat floor_thr;
  level.convertTo(floor_thr, CV_8U, 1.0 - FLOOR_DARK_RATIO);
  cv::Mat floor_mask;
  cv::compare(gray, floor_thr, floor_mask, cv::CMP_LT);
  cv::bitwise_and(mask, floor_mask, mask);

  // Loa den tran: gan bao hoa, hoac sang han hon muc san xung quanh
  cv::Mat glare_thr, g_rel, g_abs;
  level.convertTo(glare_thr, CV_8U, GLARE_FLOOR_GAIN);
  cv::compare(gray, glare_thr, g_rel, cv::CMP_GT);
  cv::compare(gray, GLARE_MIN_V, g_abs, cv::CMP_GE);
  cv::bitwise_or(g_rel, g_abs, glare);
  cv::bitwise_and(glare, valid_, glare);

  // Pixel co MAU ro (ghe cam, vat mau...) khong phai bang keo den
  cv::Mat colored = cv::Mat::zeros(bev_bgr.size(), CV_8U);
  for (int y = 0; y < bev_bgr.rows; ++y) {
    const cv::Vec3b *p = bev_bgr.ptr<cv::Vec3b>(y);
    uchar *c = colored.ptr<uchar>(y);
    for (int x = 0; x < bev_bgr.cols; ++x) {
      const int mx = std::max({p[x][0], p[x][1], p[x][2]});
      const int mn = std::min({p[x][0], p[x][1], p[x][2]});
      c[x] = (mx >= COLOR_MIN_V && (mx - mn) * 255 > COLOR_MAX_SAT * mx) ? 255 : 0;
    }
  }
  cv::dilate(colored, colored, cv::Mat::ones(3, 3, CV_8U));
  mask.setTo(0, colored);

  cv::bitwise_and(mask, valid_, mask);
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, cv::Mat::ones(3, 3, CV_8U));
  cv::bitwise_and(mask, valid_, mask);
}

// ============================================================================
// LOC DO VAT (giong _filter_clutter cua Python)
// ----------------------------------------------------------------------------
//  (1) Qua day: tui, chan tuong, nguoi, gam tu... Bang keo ban rong / dan 2
//      lop tren mask day toi ~12 cm nhung luon THUON DAI.
//  (2) Vat dung dung (chan ban/ghe, khung cua): canh thang dung trong anh
//      chieu xuong san thanh vet chia thang ve camera va bat dau tu cho no
//      cham san (khong keo toi mep duoi tam nhin nhu vach xe dang de len).
//  (0) Truoc ca 2 buoc tren: xoa phan DAY cua mask (mo hinh thai hoc) de
//      vach dinh vao vat (vach cham chan ghe, to giay, o cam) tach ra khoi
//      vat. Truoc day ca khoi bi giu/loai chung -> mat vach hoac bam vat.
//  (3) Net qua manh va nhat (khe gach, vet nut) -> loai.
// ============================================================================

void CameraLane::filter_clutter(const cv::Mat &mask_in, const cv::Mat &rel,
                                cv::Mat &clean, cv::Mat &rejected) const {
  clean = cv::Mat::zeros(mask_in.size(), CV_8U);
  rejected = cv::Mat::zeros(mask_in.size(), CV_8U);

  // (0) Phan mask chua vua hinh tron BLOB_OPEN_M = than vat -> xoa (kem vien).
  // TRU 2 truong hop la bang keo: khoi nho co 1-2 hinh tron (GOC GAP chu L
  // cua bang keo chua vua hinh tron ~1.2 lan be rong) va khoi dai manh (bang
  // keo nhoe / dan 2 lop). Truoc day xoa ca 2 -> mat goc cua, vach dut doi.
  const int ko = std::max(5, static_cast<int>(std::lround(BLOB_OPEN_M / BEV_RES_M))) | 1;
  cv::Mat body;
  cv::morphologyEx(mask_in, body, cv::MORPH_OPEN,
                   cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ko, ko)));
  if (cv::countNonZero(body) > 0) {
    cv::Mat blab, bst, bcen;
    const int nb = cv::connectedComponentsWithStats(body, blab, bst, bcen, 8, CV_32S);
    const double disc = CV_PI * 0.25 * ko * ko;
    std::vector<uchar> keep_body(static_cast<size_t>(nb), 0);
    std::vector<std::vector<cv::Point>> pts(static_cast<size_t>(nb));
    for (int y = 0; y < blab.rows; ++y) {
      const int *l = blab.ptr<int>(y);
      for (int x = 0; x < blab.cols; ++x) {
        if (l[x] > 0) {
          pts[static_cast<size_t>(l[x])].emplace_back(x, y);
        }
      }
    }
    for (int i = 1; i < nb; ++i) {
      const double a = bst.at<int>(i, cv::CC_STAT_AREA);
      const cv::RotatedRect rr = cv::minAreaRect(pts[static_cast<size_t>(i)]);
      const double lo = std::min(rr.size.width, rr.size.height);
      const double hi = std::max(rr.size.width, rr.size.height);
      const bool corner = a < BODY_CORNER_DISCS * disc;
      const bool strip = hi > 3.0 * std::max(lo, 1.0) && lo * BEV_RES_M < BLOB_THICK_M;
      keep_body[static_cast<size_t>(i)] = (corner || strip) ? 1 : 0;
    }
    for (int y = 0; y < blab.rows; ++y) {
      const int *l = blab.ptr<int>(y);
      uchar *b = body.ptr<uchar>(y);
      for (int x = 0; x < blab.cols; ++x) {
        if (l[x] > 0 && keep_body[static_cast<size_t>(l[x])]) {
          b[x] = 0;
        }
      }
    }
  }
  cv::Mat mask = mask_in.clone();
  if (cv::countNonZero(body) > 0) {
    cv::dilate(body, body,
               cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                         cv::Size(2 * BLOB_EAT_PX + 1, 2 * BLOB_EAT_PX + 1)));
    cv::bitwise_and(body, mask_in, body);
    mask.setTo(0, body);
    rejected.setTo(255, body);
  }

  cv::Mat lab, st, cen;
  const int n = cv::connectedComponentsWithStats(mask, lab, st, cen, 8, CV_32S);
  if (n <= 1) {
    return;
  }
  cv::Mat dt;
  cv::distanceTransform(mask, dt, cv::DIST_L2, 3);
  // Duong tam net (dt cuc dai dia phuong): dt o day = nua be rong net
  cv::Mat dt_max;
  cv::dilate(dt, dt_max, cv::Mat::ones(3, 3, CV_8U));
  const float thick_half = 0.5f * TAPE_MAX_THICK_M / BEV_RES_M;
  const float blob_half = 0.5f * BLOB_THICK_M / BEV_RES_M;

  const size_t N = static_cast<size_t>(n);
  std::vector<double> sx(N, 0.0), sy(N, 0.0), sxx(N, 0.0), syy(N, 0.0), sxy(N, 0.0);
  std::vector<int> thick(N, 0), very(N, 0);
  std::vector<std::vector<float>> ridge(N), relv(N);
  for (int y = 0; y < lab.rows; ++y) {
    const int *l = lab.ptr<int>(y);
    const float *d = dt.ptr<float>(y);
    const float *dm = dt_max.ptr<float>(y);
    const uchar *rv = rel.ptr<uchar>(y);
    for (int x = 0; x < lab.cols; ++x) {
      if (l[x] <= 0) {
        continue;
      }
      const size_t i = static_cast<size_t>(l[x]);
      if (d[x] >= dm[x]) {
        ridge[i].push_back(d[x]);
      }
      relv[i].push_back(rv[x]);
      sx[i] += x;
      sy[i] += y;
      sxx[i] += static_cast<double>(x) * x;
      syy[i] += static_cast<double>(y) * y;
      sxy[i] += static_cast<double>(x) * y;
      thick[i] += d[x] > thick_half;
      very[i] += d[x] > blob_half;
    }
  }

  const double cos_radial = std::cos(RADIAL_DEG * CV_PI / 180.0);
  std::vector<uchar> keep(N, 0);
  for (int l = 1; l < n; ++l) {
    const size_t i = static_cast<size_t>(l);
    const double a = st.at<int>(l, cv::CC_STAT_AREA);
    if (a < MIN_AREA_PX) {
      continue;
    }
    const double mx = sx[i] / a;
    const double my = sy[i] / a;
    const double cxx = sxx[i] / a - mx * mx;
    const double cyy = syy[i] / a - my * my;
    const double cxy = sxy[i] / a - mx * my;
    const double tr = cxx + cyy;
    const double det = cxx * cyy - cxy * cxy;
    const double disc = std::sqrt(std::max(tr * tr / 4.0 - det, 0.0));
    const double l1 = tr / 2.0 + disc;
    const double l2 = std::max(tr / 2.0 - disc, 1e-6);
    const double th = 0.5 * std::atan2(2.0 * cxy, cxx - cyy); // truc chinh (pixel BEV)
    const double ax_x = std::cos(th);
    const double ax_z = -std::sin(th);                         // doi sang (X, Z)
    const double gx = x_min_ + (mx + 0.5) * BEV_RES_M;
    const double gz = z_max_ - (my + 0.5) * BEV_RES_M;
    const double rn = std::max(std::hypot(gx, gz), 1e-6);
    const bool radial = std::fabs(ax_x * gx + ax_z * gz) / rn > cos_radial;
    const bool elong = l1 / l2 > 9.0;
    const double length = 4.0 * std::sqrt(l1) * BEV_RES_M;
    const int bottom = st.at<int>(l, cv::CC_STAT_TOP) + st.at<int>(l, cv::CC_STAT_HEIGHT) - 1;
    const double z_near = z_max_ - (bottom + 0.5) * BEV_RES_M;
    const int col = std::clamp(static_cast<int>(std::lround(mx)), 0, nx_ - 1);
    const double gap = z_near - z_near_col_[static_cast<size_t>(col)];
    const bool upright = radial && elong && length > 0.12 && gap > RADIAL_NEAR_GAP_M;
    const double width = std::max<double>(
        BEV_RES_M, a * BEV_RES_M * BEV_RES_M / std::max<double>(length, BEV_RES_M));
    const bool blob = very[i] > std::max(4.0, 0.10 * a) ||
                      (thick[i] > std::max(4.0, 0.15 * a) && length < 6.0 * width);
    const bool faint_thin = median_of(ridge[i]) < STROKE_MIN_HALF_PX &&
                            median_of(relv[i]) < STROKE_THIN_REL * 255.0f;
    keep[i] = !upright && !blob && !faint_thin;
  }

  for (int y = 0; y < lab.rows; ++y) {
    const int *l = lab.ptr<int>(y);
    uchar *c = clean.ptr<uchar>(y);
    uchar *r = rejected.ptr<uchar>(y);
    for (int x = 0; x < lab.cols; ++x) {
      if (l[x] > 0) {
        (keep[static_cast<size_t>(l[x])] ? c : r)[x] = 255;
      }
    }
  }
}

// ============================================================================
// DO VACH: di doc vach bang cua so truot, theo duoc goc gap
// ============================================================================

bool CameraLane::window(const cv::Mat &work, const cv::Point2f &p, int r,
                        cv::Point2f &mean, int &count) const {
  float c = 0.0f;
  float rr = 0.0f;
  px_of(p, c, rr);
  const int ci = static_cast<int>(std::lround(c));
  const int ri = static_cast<int>(std::lround(rr));
  const int c0 = std::max(0, ci - r), c1 = std::min(nx_, ci + r + 1);
  const int r0 = std::max(0, ri - r), r1 = std::min(nz_, ri + r + 1);
  if (c0 >= c1 || r0 >= r1) {
    return false;
  }
  long sx = 0;
  long sy = 0;
  int n = 0;
  for (int y = r0; y < r1; ++y) {
    const uchar *w = work.ptr<uchar>(y);
    for (int x = c0; x < c1; ++x) {
      if (w[x]) {
        sx += x;
        sy += y;
        ++n;
      }
    }
  }
  if (n < MIN_WIN_PX) {
    return false;
  }
  mean = ground_of(static_cast<float>(sx) / n, static_cast<float>(sy) / n);
  count = n;
  return true;
}

// Co pixel loa den trong o 5x5 quanh g?
bool CameraLane::in_glare(const cv::Point2f &g) const {
  if (glare_.empty()) {
    return false;
  }
  float c = 0.0f;
  float rr = 0.0f;
  px_of(g, c, rr);
  const int ci = static_cast<int>(std::lround(c));
  const int ri = static_cast<int>(std::lround(rr));
  const int c0 = std::max(0, ci - 2), c1 = std::min(nx_, ci + 3);
  const int r0 = std::max(0, ri - 2), r1 = std::min(nz_, ri + 3);
  return c0 < c1 && r0 < r1 &&
         cv::countNonZero(glare_(cv::Range(r0, r1), cv::Range(c0, c1))) > 0;
}

void CameraLane::consume(cv::Mat &work, const cv::Point2f &p, int r) const {
  float c = 0.0f;
  float rr = 0.0f;
  px_of(p, c, rr);
  const int ci = static_cast<int>(std::lround(c));
  const int ri = static_cast<int>(std::lround(rr));
  const int c0 = std::max(0, ci - r), c1 = std::min(nx_, ci + r + 1);
  const int r0 = std::max(0, ri - r), r1 = std::min(nz_, ri + r + 1);
  if (c0 < c1 && r0 < r1) {
    work(cv::Range(r0, r1), cv::Range(c0, c1)).setTo(0);
  }
}

// Di doc vach tu p theo huong d, an dan pixel da di qua (khong quay lai, khong
// bat vach do lan 2). Het vach phia truoc thi thu cac huong GAP (goc cua gap)
// roi di tiep.
CameraLane::Poly CameraLane::walk(cv::Mat &work, cv::Point2f p, cv::Point2f d,
                                  float max_len, bool allow_kink) const {
  static const std::vector<float> kink_angles = [] {
    std::vector<float> a;
    for (int k = 30; k <= 120; k += 10) {
      a.push_back(static_cast<float>(-k));
      a.push_back(static_cast<float>(k));
    }
    return a;
  }();

  Poly pts{p};
  consume(work, p, WIN_PX + 1);
  float length = 0.0f;
  size_t last_kink = 0;
  float len_at_kink = 0.0f;
  const int r = WIN_PX;
  const int max_iter = static_cast<int>(max_len / WALK_STEP_M) + 20;
  // Do "day" cua net dang di (so pixel trong cua so, TB truot). Bang keo day
  // ~7 cm -> 20-30 px / cua so; khe gach / vet nut 1-2 cm -> 5-10 px. Net
  // dot ngot manh han (bang keo het, khe gach noi tiep) -> dung lai, khong
  // di lac theo khe gach (sinh "goc cua" gia hoac vach lech huong).
  float thick_ema = 0.0f;
  int weak = 0;
  // Sau 1 lan noi xa qua vung loa den: vai buoc tiep theo phai di THANG nhu
  // bang keo. Vung "loa" co the la to giay trang canh vach -> noi sang bui day
  // cap / do vat ben kia; di tiep thi cong queo -> cat bo tu cho noi.
  size_t bridge_at = 0;
  int probation = 0;
  for (int it = 0; it < max_iter && length < max_len; ++it) {
    cv::Point2f found;
    int cnt = 0;
    bool ok = false;
    bool glare_gap = false;
    int k_used = 0;
    // Noi qua cho dut ngan (<= GAP_STEPS buoc); cho dut nam trong vung loa den
    // (bang keo bong phan chieu den tran) thi noi xa hon
    int k_max = GAP_STEPS;
    for (int k = 1; k <= k_max && !ok; ++k) {
      const cv::Point2f q = p + d * (WALK_STEP_M * k);
      if (!inside(q)) {
        break;
      }
      ok = window(work, q, r, found, cnt);
      k_used = k;
      if (!ok && in_glare(q)) {
        glare_gap = true;
        k_max = GLARE_BRIDGE_STEPS;
      }
    }
    if (ok && !glare_gap && thick_ema >= THICK_REF_PX) {
      if (static_cast<float>(cnt) < THIN_DROP_FRAC * thick_ema) {
        if (++weak >= THIN_DROP_STEPS) {
          // Bo cac buoc manh vua di (khong phai bang keo)
          const size_t drop = std::min(pts.size() - 1, static_cast<size_t>(weak - 1));
          pts.resize(pts.size() - drop);
          length = std::max(0.0f, length - static_cast<float>(drop) * WALK_STEP_M);
          break;
        }
      } else {
        weak = 0;
      }
    }
    bool kink = false;
    if (!ok) {
      if (!allow_kink) {
        break;
      }
      // Goc gap that: doan sau goc cung la BANG KEO (du day), khong phai khe
      // gach manh noi vao dau vach
      const int need = std::max(KINK_MIN_PX, static_cast<int>(0.5f * thick_ema));
      int best_n = 0;
      for (float a : kink_angles) {
        const cv::Point2f dd = rotate(d, a);
        const cv::Point2f q = p + dd * (WALK_STEP_M * 1.7f);
        if (!inside(q)) {
          continue;
        }
        cv::Point2f m;
        int c = 0;
        if (window(work, q, r, m, c) && c >= need && c > best_n) {
          best_n = c;
          found = m;
        }
      }
      if (best_n == 0 || probation > 0) {
        if (probation > 0) {
          pts.resize(bridge_at); // het vach ngay sau cho noi qua loa: noi sai
        }
        break;
      }
      kink = true;
      cnt = best_n;
      weak = 0;
    }
    if (!glare_gap && !kink) {
      thick_ema = thick_ema <= 0.0f ? static_cast<float>(cnt)
                                    : 0.7f * thick_ema + 0.3f * static_cast<float>(cnt);
    }
    const cv::Point2f seg = found - p;
    const float n = norm2(seg);
    if (n < 0.4f * WALK_STEP_M) {
      break;
    }
    const cv::Point2f dn = seg * (1.0f / n);
    if (probation > 0) {
      if (dn.dot(d) < std::cos(BRIDGE_MAX_TURN_DEG * kDeg)) {
        pts.resize(bridge_at);
        break;
      }
      --probation;
    }
    if (ok && k_used > GAP_STEPS) {
      bridge_at = pts.size();
      probation = 3;
      // Diem noi phai nam tren duong keo dai cua vach (lech ngang it)
      if (std::fabs(cross2(d, seg)) > BRIDGE_MAX_OFF_M) {
        break;
      }
    }
    d = kink ? dn : unit(0.5f * d + 0.5f * dn);
    consume(work, found, r + 1);
    if (kink) {
      last_kink = pts.size();
      len_at_kink = length;
    }
    pts.push_back(found);
    length += n;
    p = found;
  }
  // Doan sau goc gap cuoi qua ngan = re vao vet ban / do vat dinh dau vach
  // (khong phai goc cua that) -> bo, khong de sinh "goc cua" gia
  if (last_kink > 0 && last_kink < pts.size() && length - len_at_kink < MIN_KINK_TAIL_M) {
    pts.resize(last_kink);
  }
  return pts;
}

CameraLane::Poly CameraLane::trace_from(cv::Mat &work, const cv::Point2f &seed,
                                        const cv::Point2f &d) const {
  // Di lui ve phia xe: KHONG re o goc gap (re sang doan vach ngang -> thu tu
  // vach bi dao -> tam lan sang nham phia)
  const Poly back = walk(work, seed, -d, 0.6f, false);
  const Poly fwd = walk(work, seed, d, MAX_LINE_M, true);
  if (back.size() <= 1) {
    return fwd;
  }
  Poly out(back.rbegin(), back.rend() - 1);
  out.insert(out.end(), fwd.begin(), fwd.end());
  return out;
}

// Bam lai 1 vach quanh vi tri du doan
bool CameraLane::follow(cv::Mat &work, const Poly &pred, float gate, Poly &pts,
                        float &err) const {
  const Poly pr = resample_lin(pred, WALK_STEP_M);
  if (pr.size() < 2) {
    return false;
  }
  const int seed_px = std::max(4, static_cast<int>(std::lround(SEED_RADIUS_M / BEV_RES_M)));
  float acc = 0.0f;
  for (size_t i = 0; i < pr.size(); ++i) {
    if (i > 0) {
      acc += WALK_STEP_M;
    }
    if (acc > SEED_SEARCH_M) {
      break;
    }
    if (!inside(pr[i])) {
      continue;
    }
    cv::Point2f w;
    int cnt = 0;
    if (!window(work, pr[i], seed_px, w, cnt)) {
      continue;
    }
    const size_t a = i > 0 ? i - 1 : 0;
    const size_t b = std::min(i + 1, pr.size() - 1);
    // Di tren BAN SAO: vach bi loai (lech du doan) thi pixel cua no van con
    // cho buoc tim vach moi. Truoc day pixel bi an mat -> vach that dang
    // thay ro cung khong nhan lai duoc trong frame do -> LOST giua cua.
    cv::Mat trial = work.clone();
    Poly t = trace_from(trial, w, unit(pr[b] - pr[a]));
    if (poly_len(t) < 0.08f) {
      trial.copyTo(work); // vet ngan (nhieu): bo luon
      continue;
    }
    // Giu dung chieu di cua vach cu
    if (arc_pos(t.back(), pr) < arc_pos(t.front(), pr) - 0.02f) {
      std::reverse(t.begin(), t.end());
    }
    // Dung vach cu? (khong nhay sang vat khac). Chi so phan CHONG LEN vach
    // du doan: vach moi dai hon (noi qua vet loa den, thay xa hon) thi phan
    // vuot ra ngoai 2 dau vach cu khong tinh la lech.
    const size_t nn = std::min(t.size(), std::max<size_t>(2, static_cast<size_t>(0.4f / WALK_STEP_M)));
    std::vector<float> d, d_all;
    for (size_t k = 0; k < nn; ++k) {
      bool in = false;
      const float dk = dist_to_polyline(t[k], pred, &in);
      d_all.push_back(dk);
      if (in) {
        d.push_back(dk);
      }
    }
    err = median_of(d.size() >= 2 ? d : d_all);
    if (err > gate) {
      return false;
    }
    trial.copyTo(work);
    pts = std::move(t);
    return true;
  }
  return false;
}

// Vach ung vien moi: moi thanh phan lon, bat dau tu diem GAN TRUC SAU nhat
std::vector<CameraLane::Poly> CameraLane::candidates(cv::Mat &work, int max_n) const {
  std::vector<Poly> out;
  cv::Mat lab, st, cen;
  const int n = cv::connectedComponentsWithStats(work, lab, st, cen, 8, CV_32S);
  std::vector<int> order;
  for (int i = 1; i < n; ++i) {
    if (st.at<int>(i, cv::CC_STAT_AREA) >= 2 * MIN_AREA_PX) {
      order.push_back(i);
    }
  }
  std::sort(order.begin(), order.end(), [&](int a, int b) {
    return st.at<int>(a, cv::CC_STAT_AREA) > st.at<int>(b, cv::CC_STAT_AREA);
  });
  const cv::Point2f axle(0.0f, -profile_.cam_to_rear_m);
  for (size_t k = 0; k < order.size() && static_cast<int>(k) < max_n; ++k) {
    const int i = order[k];
    const int x0 = st.at<int>(i, cv::CC_STAT_LEFT), y0 = st.at<int>(i, cv::CC_STAT_TOP);
    const int w = st.at<int>(i, cv::CC_STAT_WIDTH), h = st.at<int>(i, cv::CC_STAT_HEIGHT);
    cv::Point2f seed;
    cv::Point2f sum(0.0f, 0.0f);
    float best = std::numeric_limits<float>::max();
    int cnt = 0;
    for (int y = y0; y < y0 + h; ++y) {
      const int *l = lab.ptr<int>(y);
      for (int x = x0; x < x0 + w; ++x) {
        if (l[x] != i) {
          continue;
        }
        const cv::Point2f g = ground_of(static_cast<float>(x), static_cast<float>(y));
        sum += g;
        ++cnt;
        const float dd = norm2(g - axle);
        if (dd < best) {
          best = dd;
          seed = g;
        }
      }
    }
    if (cnt == 0) {
      continue;
    }
    cv::Point2f d = sum * (1.0f / static_cast<float>(cnt)) - seed;
    if (norm2(d) < 0.03f) {
      continue;
    }
    d = unit(d);
    cv::Point2f w0;
    int c0 = 0;
    if (!window(work, seed, WIN_PX, w0, c0)) {
      continue;
    }
    // Di ca 2 chieu tu diem mam roi xep GAN -> XA: mam co the nam giua vach
    // (vach dinh vao vat / khe gach) -> vach nguoc chieu thi x_at_car, huong
    // dau vach deu sai -> ghep nham phia
    Poly pts = trace_from(work, w0, d);
    if (pts.size() >= 2 && norm2(pts.back() - axle) < norm2(pts.front() - axle)) {
      std::reverse(pts.begin(), pts.end());
    }
    if (poly_len(pts) >= 0.12f) {
      out.push_back(std::move(pts));
    }
  }
  return out;
}

// ============================================================================
// CHUYEN DONG XE GIUA 2 FRAME (mo hinh xe dap, quanh truc sau)
// ============================================================================

void CameraLane::predict_tracks(float dt) {
  const float v = v_mps_.load();
  if (v < 0.05f || dt <= 0.0f) {
    return;
  }
  const float axle = profile_.cam_to_rear_m;
  const float ds = v * dt;
  const float delta = std::clamp(wheel_deg_.load(), -40.0f, 40.0f) * kDeg;
  const float dth = ds * std::tan(delta) / std::max(0.05f, profile_.wheelbase_m); // + = phai
  const cv::Point2f t(ds * std::sin(0.5f * dth), ds * std::cos(0.5f * dth));
  const float c = std::cos(dth);
  const float s = std::sin(dth);
  for (Track *tr : {&left_, &right_}) {
    if (!tr->valid) {
      continue;
    }
    for (auto &p : tr->pts) {
      const cv::Point2f q = cv::Point2f(p.x, p.y + axle) - t;
      p = cv::Point2f(q.x * c - q.y * s, q.x * s + q.y * c - axle);
    }
  }
}

// ============================================================================
// GHEP CAP / NHAN VACH MOI
// ============================================================================

// (khoang cach, goc lech huong (do), b nam ben phai a?) o doan dau
void CameraLane::pair_geom(const Poly &a, const Poly &b, float &dist,
                           float &ang, bool &right) const {
  const size_t nb = std::min(b.size(), std::max<size_t>(2, static_cast<size_t>(0.3f / WALK_STEP_M)));
  std::vector<float> d, sd;
  for (size_t k = 0; k < nb; ++k) {
    float side = 0.0f;
    d.push_back(dist_to_polyline(b[k], a, nullptr, &side));
    sd.push_back(side);
  }
  dist = median_of(d);
  right = median_of(sd) > 0.0f;
  const float c = std::clamp(init_dir(a).dot(init_dir(b)), -1.0f, 1.0f);
  ang = std::acos(c) / kDeg;
}

float CameraLane::x_at_car(const Poly &p) {
  if (p.empty()) {
    return 0.0f;
  }
  // cross2(d, car - p0): > 0 khi chan camera nam ben TRAI vach -> vach ben phai
  return cross2(init_dir(p), cv::Point2f(0.0f, 0.0f) - p[0]);
}

void CameraLane::acquire(const std::vector<Poly> &cands, bool have_left,
                         bool have_right, bool allow_single) {
  // Be rong lan KHONG co dinh (bang keo dan tay, moi doan rong / hep khac
  // nhau): ghep cap chi doi khoang tuyet doi LANE_W_MIN_M..LANE_W_MAX_M; W
  // (be rong vua do gan day) chi dung de uu tien cap gan dung nhat
  const float max_ang = pitch_confirmed_ ? 25.0f : 40.0f;
  const float W = lane_w_local_;
  auto make = [](const Poly &p, bool verified) {
    Track t;
    t.valid = true;
    t.pts = p;
    t.observed = true;
    t.verified = verified;
    t.seen = 1;
    return t;
  };

  if (have_left || have_right) {
    const Track &ref = have_left ? left_ : right_;
    const bool want_right = have_left;
    // Vach con thieu phai nam phia ben kia xe, cach vach dang co ~1 lan
    const float x_ref = x_at_car(ref.pts);
    const float x_want = x_ref + (want_right ? W : -W);
    int best = -1;
    float best_score = -std::numeric_limits<float>::max();
    for (size_t i = 0; i < cands.size(); ++i) {
      float dist = 0.0f, ang = 0.0f;
      bool right = false;
      pair_geom(ref.pts, cands[i], dist, ang, right);
      if (right != want_right || dist < LANE_W_MIN_M || dist > LANE_W_MAX_M || ang > max_ang + 5.0f) {
        continue;
      }
      const float xc = x_at_car(cands[i]);
      if (want_right ? xc < -CAR_SIDE_SLACK_M : xc > CAR_SIDE_SLACK_M) {
        continue;
      }
      const float score = poly_len(cands[i]) - 2.0f * std::fabs(dist - W) -
                          1.5f * std::fabs(xc - x_want) -
                          std::max(0.0f, cands[i][0].y - 0.4f);
      if (score > best_score) {
        best_score = score;
        best = static_cast<int>(i);
      }
    }
    if (best >= 0) {
      (want_right ? right_ : left_) = make(cands[static_cast<size_t>(best)], ref.verified);
    }
    return;
  }

  auto fwd_ok = [](const Poly &p, float lim) {
    return init_dir(p).y > std::cos(lim * kDeg) && p[0].y < 1.0f;
  };
  int bl = -1, br = -1;
  float best_score = -std::numeric_limits<float>::max();
  for (size_t i = 0; i < cands.size(); ++i) {
    for (size_t j = 0; j < cands.size(); ++j) {
      if (i == j || !fwd_ok(cands[i], 60.0f) || !fwd_ok(cands[j], 60.0f)) {
        continue;
      }
      float dist = 0.0f, ang = 0.0f;
      bool right = false;
      pair_geom(cands[i], cands[j], dist, ang, right);
      if (!right || dist < LANE_W_MIN_M || dist > LANE_W_MAX_M || ang > max_ang) {
        continue;
      }
      // Xe phai nam GIUA 2 vach (vach trai ben trai, vach phai ben phai chan
      // camera). Thieu dieu kien nay xe tung ghep vach phai that voi mep o
      // cam / to giay ben ngoai lan -> danh lai het co sang phai.
      const float xl = x_at_car(cands[i]);
      const float xr = x_at_car(cands[j]);
      if (xl > CAR_SIDE_SLACK_M || xr < -CAR_SIDE_SLACK_M || xl < -LANE_W_MAX_M || xr > LANE_W_MAX_M) {
        continue;
      }
      const float score = poly_len(cands[i]) + poly_len(cands[j]) - 2.0f * std::fabs(dist - W) -
                          1.5f * std::fabs(0.5f * (xl + xr));
      if (score > best_score) {
        best_score = score;
        bl = static_cast<int>(i);
        br = static_cast<int>(j);
      }
    }
  }
  if (bl >= 0) {
    left_ = make(cands[static_cast<size_t>(bl)], true);
    right_ = make(cands[static_cast<size_t>(br)], true);
    return;
  }
  if (!allow_single) {
    return;
  }
  // Chi 1 vach: nhan tam (chua xac minh, khong dung de lai), doan trai/phai
  // theo vi tri. Vach phai cach chan camera tam nua lan (xe dang trong lan).
  int bs = -1;
  float bs_len = 0.0f;
  for (size_t i = 0; i < cands.size(); ++i) {
    const Poly &c = cands[i];
    const float L = poly_len(c);
    const float xc = std::fabs(x_at_car(c));
    if (fwd_ok(c, 45.0f) && L >= 0.3f && c[0].y < 0.7f && std::fabs(c[0].x) < 0.45f &&
        xc > 0.15f * W && xc < 0.9f * W && L > bs_len) {
      bs_len = L;
      bs = static_cast<int>(i);
    }
  }
  if (bs >= 0) {
    const Poly &c = cands[static_cast<size_t>(bs)];
    (x_at_car(c) < 0.0f ? left_ : right_) = make(c, false);
  }
}

// ============================================================================
// HIEU CHINH: be rong lan, goc cui camera, nguong toi
// ============================================================================

bool CameraLane::straight_head(const Poly &pts, float length, cv::Point2f &a,
                               cv::Point2f &b) const {
  Poly p = resample_lin(pts, 0.03f);
  const size_t need = static_cast<size_t>(length / 0.03f);
  if (p.size() < need) {
    return false;
  }
  p.resize(std::min(p.size(), need + 1));
  a = p.front();
  b = p.back();
  const cv::Point2f ab = b - a;
  const float n = norm2(ab);
  if (n < 0.8f * length) {
    return false;
  }
  for (const auto &q : p) {
    if (std::fabs(cross2(ab, q - a)) / n >= 0.012f) {
      return false;
    }
  }
  return true;
}

// Be rong lan (m): khoang cach vuong goc giua 2 doan thang gan xe
void CameraLane::learn_width(const Poly &left, const Poly &right) {
  cv::Point2f la, lb, ra, rb;
  if (!straight_head(left, 0.45f, la, lb) || !straight_head(right, 0.45f, ra, rb)) {
    return;
  }
  const cv::Point2f ab = lb - la;
  const float n = std::max(1e-9f, norm2(ab));
  const float w = 0.5f * (std::fabs(cross2(ab, ra - la)) + std::fabs(cross2(ab, rb - la))) / n;
  if (w >= LANE_W_MIN_M && w <= LANE_W_MAX_M) {
    width_samples_.push_back(w);
    if (width_samples_.size() > 100) {
      width_samples_.pop_front();
    }
    if (width_samples_.size() >= 15) {
      lane_w_m_ = median_of(std::vector<float>(width_samples_.begin(), width_samples_.end()));
    }
  }
}

// Goc cui camera suy tu 2 doan vach thang (gia thiet song song tren san).
// Diem tu cua 2 duong song song tren san nam tren duong chan troi -> ra goc
// cui. Kiem tra cheo: o goc do 2 vach phai cach nhau dung ~1 lan va nam 2 ben
// xe -> loai cap nhieu / 2 mau cua cung 1 vach.
bool CameraLane::pitch_from_pair(const cv::Point2f ha[2], const cv::Point2f hb[2],
                                 double &pitch) const {
  const double cx = WORK_W / 2.0;
  cv::Vec3d img[4];
  const cv::Point2f g4[4] = {ha[0], ha[1], hb[0], hb[1]};
  double v_min = std::numeric_limits<double>::max();
  for (int i = 0; i < 4; ++i) {
    double u = 0.0, v = 0.0;
    if (!to_image(pitch_, f_px_, h_, cx, cy_, g4[i].x, g4[i].y, u, v)) {
      return false;
    }
    img[i] = cv::Vec3d(u, v, 1.0);
    v_min = std::min(v_min, v);
  }
  const cv::Vec3d la = img[0].cross(img[1]);
  const cv::Vec3d lb = img[2].cross(img[3]);
  const cv::Vec3d vp = la.cross(lb);
  if (std::fabs(vp[2]) < 1e-9) {
    return false;
  }
  const double u_h = vp[0] / vp[2];
  const double v_h = vp[1] / vp[2];
  if (!(u_h > -2.0 * WORK_W && u_h < 3.0 * WORK_W) || v_h > v_min - 5.0) {
    return false;
  }
  const double p_deg = std::atan2(cy_ - v_h, f_px_) * 180.0 / CV_PI;
  if (std::fabs(p_deg - profile_.pitch_deg) > profile_.pitch_max_dev_deg) {
    return false;
  }
  const double p_rad = p_deg * CV_PI / 180.0;
  cv::Point2f g[4];
  for (int i = 0; i < 4; ++i) {
    g[i] = to_ground(p_rad, f_px_, h_, cx, cy_, img[i][0], img[i][1]);
  }
  const cv::Point2f ab = g[1] - g[0];
  const float n = std::max(1e-9f, norm2(ab));
  const float s2 = cross2(ab, g[2] - g[0]) / n;
  const float s3 = cross2(ab, g[3] - g[0]) / n;
  const float w = 0.5f * std::fabs(s2 + s3);
  if (w < LANE_W_MIN_M || w > LANE_W_MAX_M) {
    return false;
  }
  // Xe (diem duoi camera) phai nam GIUA 2 vach
  const float car = cross2(ab, cv::Point2f(0.0f, 0.0f) - g[0]) / n;
  if (car * s2 < 0.0f || std::fabs(car) > std::fabs(s2) + 0.05f) {
    return false;
  }
  pitch = p_deg;
  return true;
}

void CameraLane::calibrate_pitch(const std::vector<const Poly *> &lines) {
  std::vector<std::array<cv::Point2f, 2>> heads;
  for (size_t i = 0; i < lines.size() && i < 8; ++i) {
    cv::Point2f a, b;
    if (straight_head(*lines[i], 0.35f, a, b)) {
      heads.push_back({a, b});
    }
  }
  std::vector<float> found;
  for (size_t i = 0; i < heads.size(); ++i) {
    for (size_t j = i + 1; j < heads.size(); ++j) {
      if (norm2(heads[i][0] - heads[j][0]) < 0.1f) {
        continue;
      }
      double p = 0.0;
      if (pitch_from_pair(heads[i].data(), heads[j].data(), p)) {
        found.push_back(static_cast<float>(p));
      }
    }
  }
  if (found.empty()) {
    return;
  }
  pitch_samples_.push_back(median_of(found)); // 1 mau / frame
  if (pitch_samples_.size() > 40) {
    pitch_samples_.pop_front();
  }
  if (pitch_samples_.size() < 10) {
    return;
  }
  const std::vector<float> recent(
      pitch_samples_.end() - std::min<long>(15, static_cast<long>(pitch_samples_.size())),
      pitch_samples_.end());
  const float q1 = percentile(recent, 0.25f);
  const float med = percentile(recent, 0.50f);
  const float q3 = percentile(recent, 0.75f);
  if (q3 - q1 > 2.0f) {
    return;
  }
  pitch_confirmed_ = true;
  const double old = pitch_ * 180.0 / CV_PI;
  const double diff = std::fabs(med - old);
  if (diff > 0.7) {
    // Ap dung o dau frame sau: luoi BEV doi kich thuoc, phan con lai cua frame
    // nay van dung anh BEV cu
    pending_pitch_ = med;
    pending_reset_ = diff > 2.0; // vach dang bam tinh theo goc cu sai nhieu -> do lai
    pitch_samples_.clear();
    width_samples_.clear();
    std::cout << "[CameraLane] Tu hieu chinh goc cui camera: " << old << " -> "
              << med << " do (dat camera_pitch_deg:=" << med
              << " de lan sau khoi dong dung ngay)\n";
  }
}

// Nguong toi = mot phan do tuong phan cua CHINH vach dang bam. Ban cu ha
// nguong moi khi thieu vach -> trong cua / ngoai duong nguong tut xuong va
// nhan ca vet gach, do dac.
void CameraLane::adapt_threshold(const cv::Mat &rel,
                                 const std::vector<const Poly *> &obs) {
  std::vector<float> vals;
  for (const Poly *pts : obs) {
    for (size_t k = 0; k < pts->size(); k += 2) {
      float c = 0.0f, r = 0.0f;
      px_of((*pts)[k], c, r);
      const int ci = static_cast<int>(std::lround(c));
      const int ri = static_cast<int>(std::lround(r));
      if (ci >= 1 && ci < nx_ - 1 && ri >= 1 && ri < nz_ - 1) {
        double mx = 0.0;
        cv::minMaxLoc(rel(cv::Range(ri - 1, ri + 2), cv::Range(ci - 1, ci + 2)),
                      nullptr, &mx);
        vals.push_back(static_cast<float>(mx));
      }
    }
  }
  float ratio = dark_ratio_;
  if (vals.size() >= 8) {
    contrast_ = median_of(vals) / 255.0f;
    const float target = std::min(CONTRAST_THR_MAX, CONTRAST_FRAC * contrast_);
    ratio += 0.1f * (target - ratio);
  } else if (obs.empty()) {
    ratio += 0.02f * (DARK_RATIO_INIT - ratio);
  }
  if (last_fill_ > NOISE_FILL) {
    ratio *= 1.05f;
  }
  dark_ratio_ = std::clamp(ratio, DARK_RATIO_MIN, DARK_RATIO_MAX);
}

// ============================================================================
// DUONG DI + PURE PURSUIT
// ============================================================================

// Diem bam tren path kieu pure pursuit tu TRUC SAU. Path chi bat dau tu cho
// camera thay -> keo dai nguoc ve phia xe theo huong dau path, lay diem gan
// truc sau nhat roi di TOI doc path sqrt(L^2 - d^2) (>= PP_MIN_FWD_M). Khong
// lay "diem dau path" khi path o xa: o goc cua, vach ngoai nam ngang truoc
// mat co dau path o phia ben kia -> xe lai nguoc.
bool CameraLane::target_on(const Poly &path, float L, cv::Point2f &target,
                           float &ext) const {
  ext = 0.0f;
  Poly p = resample_lin(path, 0.02f);
  if (p.size() < 2) {
    return false;
  }
  const cv::Point2f a(0.0f, -profile_.cam_to_rear_m);
  const cv::Point2f d0 = unit(p[1] - p[0]);
  Poly full;
  for (float s = 0.80f; s > 0.001f; s -= 0.02f) {
    full.push_back(p[0] - d0 * s);
  }
  full.insert(full.end(), p.begin(), p.end());
  size_t ic = 0;
  float dmin = std::numeric_limits<float>::max();
  for (size_t i = 0; i < full.size(); ++i) {
    const float d = norm2(full[i] - a);
    if (d < dmin) {
      dmin = d;
      ic = i;
    }
  }
  const float off = std::max(std::sqrt(std::max(0.0f, L * L - dmin * dmin)), PP_MIN_FWD_M);
  float acc = 0.0f;
  for (size_t i = ic; i + 1 < full.size(); ++i) {
    const float l = norm2(full[i + 1] - full[i]);
    if (acc + l >= off) {
      target = full[i] + (full[i + 1] - full[i]) * ((off - acc) / std::max(l, 1e-6f));
      return true;
    }
    acc += l;
  }
  ext = off - acc;
  target = full.back() + unit(full.back() - full[full.size() - 2]) * ext;
  return true;
}

bool CameraLane::kink_ahead(const Poly &path, float &dist, int &dir, float &deg,
                            cv::Point2f &pt) const {
  const Poly v = simplify(path, RDP_EPS_M * 1.5f);
  if (v.size() < 3) {
    return false;
  }
  const cv::Point2f a(0.0f, -profile_.cam_to_rear_m);
  for (size_t i = 1; i + 1 < v.size(); ++i) {
    const cv::Point2f d_in = v[i] - v[i - 1];
    const cv::Point2f d_out = v[i + 1] - v[i];
    if (norm2(d_out) < 0.06f || norm2(d_in) < 0.06f) {
      continue; // canh qua ngan = rang cua, khong phai goc cua
    }
    const float ang = std::atan2(cross2(d_in, d_out), d_in.dot(d_out)) / kDeg;
    const float d = norm2(v[i] - a);
    if (d > 1.4f + profile_.cam_to_rear_m) {
      break;
    }
    if (std::fabs(ang) >= CORNER_KINK_DEG) {
      dist = d;
      dir = ang > 0.0f ? -1 : 1; // nguoc chieu kim dong ho = re trai
      deg = std::fabs(ang);
      pt = v[i];
      return true;
    }
  }
  return false;
}

// ============================================================================
// TIEN ICH HINH HOC
// ============================================================================

// Khoang cach tu p toi duong gap khuc. interior = diem gan nhat KHONG phai 2
// dau mut. side: > 0 khi p nam ben PHAI huong di cua poly.
float CameraLane::dist_to_polyline(const cv::Point2f &p, const Poly &poly,
                                   bool *interior, float *side) {
  if (interior) {
    *interior = false;
  }
  if (side) {
    *side = 0.0f;
  }
  if (poly.empty()) {
    return std::numeric_limits<float>::max();
  }
  if (poly.size() == 1) {
    return norm2(p - poly[0]);
  }
  float best = std::numeric_limits<float>::max();
  bool best_in = false;
  float best_side = 0.0f;
  const size_t last = poly.size() - 2;
  for (size_t i = 0; i + 1 < poly.size(); ++i) {
    const cv::Point2f a = poly[i];
    const cv::Point2f ab = poly[i + 1] - a;
    const float L2 = ab.dot(ab);
    float t = L2 > 1e-9f ? (p - a).dot(ab) / L2 : 0.0f;
    bool in = true;
    if (t <= 0.0f) {
      t = 0.0f;
      in = i > 0;
    } else if (t >= 1.0f) {
      t = 1.0f;
      in = i < last;
    }
    const float d = norm2(p - (a + ab * t));
    if (d < best) {
      best = d;
      best_in = in;
      const float c = cross2(ab, p - a);
      best_side = c > 0.0f ? -1.0f : (c < 0.0f ? 1.0f : 0.0f);
    }
  }
  if (interior) {
    *interior = best_in;
  }
  if (side) {
    *side = best_side;
  }
  return best;
}

// Lay mau lai duong gap khuc deu `step` met, lam tron nhe (TB 3 diem)
CameraLane::Poly CameraLane::resample(const Poly &p, float step) {
  Poly r = resample_lin(p, step);
  if (r.size() >= 3) {
    Poly s = r;
    for (size_t i = 1; i + 1 < r.size(); ++i) {
      s[i] = (r[i - 1] + r[i] + r[i + 1]) * (1.0f / 3.0f);
    }
    r.swap(s);
  }
  return r;
}

// ============================================================================
// DETECTOR - 1 frame
// ============================================================================

bool CameraLane::detect(const cv::Mat &frame, LaneOutput &out, bool draw,
                        std::chrono::steady_clock::time_point stamp) {
  const auto t_wall = std::chrono::steady_clock::now();
  const auto t_begin = stamp == std::chrono::steady_clock::time_point{} ? t_wall : stamp;
  if (pending_pitch_ > 0.0) {
    set_pitch(pending_pitch_);
    pending_pitch_ = -1.0;
    if (pending_reset_) {
      left_ = Track{};
      right_ = Track{};
    }
  } else if (roi_top_frac_.load() != roi_top_used_) {
    build_bev();
  }

  const int WH = work_h_;
  out.frame_w = cam_w_ > 0 ? cam_w_ : frame.cols;
  out.frame_h = cam_h_ > 0 ? cam_h_ : frame.rows;

  if (!logged_geometry_) {
    logged_geometry_ = true;
    std::cout << "[CameraLane] geometry: camera " << cam_w_ << "x" << cam_h_
              << ", decoded " << frame.cols << "x" << frame.rows << " -> work "
              << WORK_W << "x" << WH << ", h=" << h_ << " m, pitch="
              << pitch_ * 180.0 / CV_PI << " deg, f=" << f_px_
              << " px (hfov " << 2.0 * std::atan(WORK_W / 2.0 / f_px_) * 180.0 / CV_PI
              << " deg), horizon_y=" << horizon_y_ << "; BEV " << nx_ << "x" << nz_
              << " (1 cm), Z " << z_min_ << ".." << z_max_ << " m, lane "
              << lane_w_m_ << " m\n";
  }

  const float dt = last_detect_time_ == std::chrono::steady_clock::time_point{}
                       ? 0.04f
                       : std::clamp(std::chrono::duration<float>(t_begin - last_detect_time_).count(),
                                    0.0f, 0.5f);
  last_detect_time_ = t_begin;
  dt_ = dt;
  const float v_now = v_mps_.load();
  predict_tracks(dt);

  // ---- 1. Chieu anh xuong mat san ----------------------------------------
  cv::Mat work_bgr;
  cv::resize(frame, work_bgr, cv::Size(WORK_W, WH), 0, 0, cv::INTER_AREA);
  cv::GaussianBlur(work_bgr, work_bgr, cv::Size(3, 3), 0);
  cv::Mat bev;
  cv::remap(work_bgr, bev, mapx_, mapy_, cv::INTER_LINEAR, cv::BORDER_REPLICATE);

  // ---- 2-3. Mask vach + loc do vat ---------------------------------------
  cv::Mat mask, rel, clean, rejected;
  line_mask(bev, mask, rel, glare_);
  filter_clutter(mask, rel, clean, rejected);
  last_fill_ = static_cast<float>(cv::countNonZero(mask)) / static_cast<float>(n_valid_);
  cv::Mat work = clean.clone();

  // ---- 4. Bam lai vach dang theo doi (vach tin cay hon truoc) ------------
  const float gate = TRACK_GATE_M + 0.3f * std::max(v_now, 0.25f) * dt;
  std::vector<Track *> order;
  for (Track *t : {&left_, &right_}) {
    if (t->valid) {
      order.push_back(t);
    }
  }
  std::sort(order.begin(), order.end(), [](const Track *a, const Track *b) {
    if (a->verified != b->verified) {
      return a->verified;
    }
    return a->missed < b->missed;
  });
  for (Track *t : order) {
    Poly pts;
    float e = 0.0f;
    if (follow(work, t->pts, gate, pts, e)) {
      t->pts = std::move(pts);
      t->gate_err = e;
      t->observed = true;
      t->missed = 0.0f;
      ++t->seen;
    } else {
      t->observed = false;
      t->seen = 0;
      t->missed += dt;
      if (t->missed > COAST_SEC) {
        *t = Track{};
      }
    }
  }

  // 2 vach trung / bat cheo nhau -> bo vach khop kem hon
  if (left_.valid && right_.valid && left_.observed && right_.observed) {
    float dist = 0.0f, ang = 0.0f;
    bool right = false;
    pair_geom(left_.pts, right_.pts, dist, ang, right);
    if (dist < 0.5f * LANE_W_MIN_M || !right) {
      (left_.gate_err > right_.gate_err ? left_ : right_) = Track{};
    }
  }

  // ---- 5. Tim vach con thieu ---------------------------------------------
  bool have_l = left_.valid && left_.observed;
  bool have_r = right_.valid && right_.observed;
  std::vector<Poly> cands;
  if (!(have_l && have_r)) {
    cands = candidates(work);
    // Vach dang "troi" (vua bi loa den / vat che, chua qua COAST_SEC) hien lai
    // gan cho du doan -> gan lai NGAY vao vach do (giu danh tinh trai/phai,
    // da xac minh). Truoc day phai cho het COAST_SEC + vai frame xac nhan ->
    // ~0.7 s LOST, xe giu goc lai cu va de len vach sau moi vet loa.
    for (Track *t : {&left_, &right_}) {
      if (!t->valid || t->observed || cands.empty()) {
        continue;
      }
      int best = -1;
      float best_d = std::numeric_limits<float>::max();
      for (size_t i = 0; i < cands.size(); ++i) {
        const Poly &c = cands[i];
        // Phai nam dung phia xe (vach trai ben trai, vach phai ben phai)
        const float xc = x_at_car(c);
        const bool side_ok = t == &left_ ? xc < CAR_SIDE_SLACK_M : xc > -CAR_SIDE_SLACK_M;
        if (!side_ok) {
          continue;
        }
        const float cosang = init_dir(c).dot(init_dir(t->pts));
        // (a) Trung vach du doan: lech ngang (trung vi doan dau) nho, cung huong
        const size_t nn = std::min(c.size(), std::max<size_t>(2, static_cast<size_t>(0.4f / WALK_STEP_M)));
        std::vector<float> d;
        for (size_t k = 0; k < nn; ++k) {
          d.push_back(dist_to_polyline(c[k], t->pts));
        }
        float score = std::numeric_limits<float>::max();
        if (cosang > std::cos(REACQ_MAX_DEG * kDeg)) {
          const float m = median_of(d);
          if (m < REACQ_GATE_M) {
            score = m;
          }
        }
        // (b) Goc GAP nam duoi vet loa: keo dai nguoc doan dau vach moi ve
        // phia xe toi da 0.5 m, cham vach du doan -> la doan sau goc gap
        if (cosang > std::cos(REACQ_KINK_MAX_DEG * kDeg)) {
          const cv::Point2f dir = init_dir(c);
          float meet = std::numeric_limits<float>::max();
          for (float sb = 0.0f; sb <= 0.5f; sb += 0.02f) {
            meet = std::min(meet, dist_to_polyline(c[0] - dir * sb, t->pts));
          }
          if (meet < REACQ_KINK_MEET_M) {
            score = std::min(score, REACQ_GATE_M + meet);
          }
        }
        if (score < best_d) {
          best_d = score;
          best = static_cast<int>(i);
        }
      }
      if (best >= 0) {
        t->pts = cands[static_cast<size_t>(best)];
        t->observed = true;
        t->missed = 0.0f;
        t->seen = 1;
        t->gate_err = best_d;
        cands.erase(cands.begin() + best);
      }
    }
    const bool have_l2 = left_.valid && left_.observed;
    const bool have_r2 = right_.valid && right_.observed;
    have_l = have_l2;
    have_r = have_r2;
    if (!cands.empty() && !(have_l && have_r)) {
      if (have_l || have_r) {
        acquire(cands, have_l, have_r, true);
      } else {
        // Vach cu dang 'troi' (du doan) -> chi nhan 1 cap lan hop le
        acquire(cands, false, false, !left_.valid && !right_.valid);
      }
    }
  }
  const bool obs_l = left_.valid && left_.observed;
  const bool obs_r = right_.valid && right_.observed;

  // ---- 6. Kiem tra cap lan + hieu chinh ----------------------------------
  bool both_ok = false;
  float width = 0.0f;
  if (obs_l && obs_r) {
    float ang = 0.0f;
    bool right = false;
    pair_geom(left_.pts, right_.pts, width, ang, right);
    both_ok = right && width >= LANE_W_MIN_M && width <= LANE_W_MAX_M &&
              ang < 35.0f && left_.pts[0].y < 0.8f && right_.pts[0].y < 0.8f;
    if (both_ok) {
      left_.verified = right_.verified = true;
      learn_width(left_.pts, right_.pts);
      // Be rong TAI CHO: lan dau thay du 2 vach lay ngay, sau do EMA nhanh
      if (!have_width_) {
        lane_w_m_ = width;
      }
      lane_w_local_ = have_width_ ? lane_w_local_ + LANE_W_LOCAL_ALPHA * (width - lane_w_local_)
                                  : width;
      have_width_ = true;
      lane_w_local_age_ = 0.0f;
    }
  }
  if (!both_ok) {
    // Lau khong thay du 2 vach: tro dan ve be rong hoc dai han
    lane_w_local_age_ += dt;
    if (lane_w_local_age_ > LANE_W_LOCAL_HOLD_SEC) {
      lane_w_local_ += 0.05f * (lane_w_m_ - lane_w_local_);
    }
  }
  std::vector<const Poly *> seen;
  if (obs_l) {
    seen.push_back(&left_.pts);
  }
  if (obs_r) {
    seen.push_back(&right_.pts);
  }
  if (profile_.auto_pitch) {
    std::vector<const Poly *> all = seen;
    for (const auto &c : cands) {
      all.push_back(&c);
    }
    calibrate_pitch(all);
  }
  adapt_threshold(rel, seen);

  // Vach dung de lai: dang thay va da xac minh, hoac 1 vach don bam on dinh
  // SINGLE_CONFIRM_FRAMES frame (1 frame le co the la gach san / do dac)
  const bool use_l = obs_l && (left_.verified || both_ok || left_.seen >= SINGLE_CONFIRM_FRAMES);
  const bool use_r = obs_r && (right_.verified || both_ok || right_.seen >= SINGLE_CONFIRM_FRAMES);
  LaneState state = LaneState::LOST;
  if (use_l && use_r) {
    state = LaneState::TWO_LINES;
  } else if (use_l || use_r) {
    state = LaneState::ONE_LINE;
  }
  bool two_lanes = state == LaneState::TWO_LINES;
  if (both_ok) {
    out.lane_width_cm = width * 100.0f;
  }

  // ---- 7. Tam lan tu tung vach (doi nua lan theo phap tuyen, mitre) ------
  // Du 2 vach: doi nua be rong DO O FRAME NAY -> 2 duong tam trung nhau =
  // dung GIUA 2 vach du lan rong / hep bao nhieu. (Truoc day doi nua be rong
  // da hoc, khoi dau 0.42 m: lan 0.52 m thi tam lech 5 cm ve 1 vach.) Chi 1
  // vach: doi nua be rong vua do gan nhat (lane_w_local_).
  const float half = 0.5f * (both_ok ? width : lane_w_local_);
  const Poly c_left = use_l ? offset_mitre(left_.pts, half, +1, RDP_EPS_M) : Poly{};
  const Poly c_right = use_r ? offset_mitre(right_.pts, half, -1, RDP_EPS_M) : Poly{};

  // Diem bam tu 2 tam lan: trung nhau thi TB co trong so, lech nhau thi tin
  // duong THAY GOC GAP (vach kia chi bi mep tam nhin cat nen trong nhu di
  // thang), roi toi duong dai hon va it phai ngoai suy hon
  bool kinked[2] = {false, false};
  {
    const Poly *cp[2] = {&c_left, &c_right};
    for (int s = 0; s < 2; ++s) {
      float kd = 0.0f, kg = 0.0f;
      int kr = 0;
      cv::Point2f kp;
      kinked[s] = cp[s]->size() >= 3 && kink_ahead(*cp[s], kd, kr, kg, kp);
    }
  }
  auto fuse = [&](float L, cv::Point2f &tgt, const Poly *&path) {
    struct Cand {
      float w;
      cv::Point2f t;
      const Poly *p;
      bool kink;
    };
    std::vector<Cand> cs;
    const std::pair<const Poly *, const Track *> src[2] = {{&c_left, &left_}, {&c_right, &right_}};
    for (const auto &s : src) {
      if (s.first->size() < 2) {
        continue;
      }
      cv::Point2f t;
      float ext = 0.0f;
      if (!target_on(*s.first, L, t, ext)) {
        continue;
      }
      const float w = std::min(1.0f, poly_len(s.second->pts) / 0.8f) / (1.0f + ext / 0.1f);
      cs.push_back({w, t, s.first, kinked[s.first == &c_left ? 0 : 1]});
    }
    if (cs.empty()) {
      return false;
    }
    std::sort(cs.begin(), cs.end(), [](const Cand &a, const Cand &b) {
      if (a.kink != b.kink) {
        return a.kink;
      }
      return a.w > b.w;
    });
    tgt = cs[0].t;
    path = cs[0].p;
    if (cs.size() > 1 && norm2(cs[0].t - cs[1].t) < 0.12f) {
      tgt = (cs[0].t * cs[0].w + cs[1].t * cs[1].w) * (1.0f / (cs[0].w + cs[1].w));
    }
    return true;
  };

  cv::Point2f aim(0.0f, PP_LOOKAHEAD_M);
  const Poly *path = nullptr;
  bool have_aim = fuse(PP_LOOKAHEAD_M, aim, path);
  bool have_kink = false;
  float kink_dist = 0.0f, kink_deg = 0.0f;
  int kink_dir = 0;
  cv::Point2f kink_pt;
  if (have_aim) {
    have_kink = kink_ahead(*path, kink_dist, kink_dir, kink_deg, kink_pt);
    // Goc gap phia truoc: nhin xa hon de bat dau be lai som (xe can ban kinh
    // quay ~0.6 m). Duong thang giu lookahead ngan -> ve tam nhanh sau cua.
    if (have_kink && kink_dist < PP_LOOKAHEAD_CORNER_M + 0.15f) {
      fuse(PP_LOOKAHEAD_CORNER_M, aim, path);
    }
  }
  // Doi nguon tam lan (2 vach <-> 1 vach): giu diem ngam lien mach roi
  // chuyen dan sang tam moi (STATE_BLEND_SEC), khong nhay vai cm 1 frame
  const int key = (use_l ? 1 : 0) | (use_r ? 2 : 0);
  if (have_aim) {
    if (blend_have_ && key != blend_key_) {
      blend_off_m_ = std::clamp(blend_prev_x_ - aim.x, -STATE_BLEND_MAX_M, STATE_BLEND_MAX_M);
    } else {
      blend_off_m_ *= std::exp(-dt / STATE_BLEND_SEC);
    }
    aim.x += blend_off_m_;
    blend_prev_x_ = aim.x;
    blend_have_ = true;
  } else {
    blend_off_m_ = 0.0f;
    blend_have_ = false;
  }
  blend_key_ = key;

  if (have_kink) {
    out.kink_dist_m = kink_dist;
    out.kink_dir = kink_dir;
    out.kink_deg = kink_deg;
  }

  // ---- 8. Pure pursuit -> dev_px -----------------------------------------
  if (have_aim) {
    const cv::Point2f v = aim - cv::Point2f(0.0f, -profile_.cam_to_rear_m);
    const float d = std::max(0.05f, norm2(v));
    const float kappa = 2.0f * std::sin(std::atan2(v.x, v.y)) / d; // > 0 = phai
    // Quy ve do lech tuong duong tai DEV_REF_DIST_M (giu thang do firmware)
    const float x_cmd = 0.5f * kappa * DEV_REF_DIST_M * DEV_REF_DIST_M;
    out.dev_cm = x_cmd * 100.0f;
    const float raw = std::clamp(x_cmd / DEV_M_PER_PX, -static_cast<float>(DEV_MAX_REF_PX),
                                 static_cast<float>(DEV_MAX_REF_PX));
    const float alpha = two_lanes ? EMA_ALPHA_TWO : EMA_ALPHA_ONE;
    if (!ema_primed_) {
      dev_ema_ = raw;
      ema_primed_ = true;
      jump_count_ = 0;
    } else if (std::fabs(raw - dev_ema_) > JUMP_GATE_REF_PX &&
               ++jump_count_ < JUMP_CONFIRM_FRAMES) {
      // Nhay dot ngot 1 frame: nhieu kha nang nhan nham -> giu gia tri cu
      out.gated = true;
    } else {
      jump_count_ = 0;
      dev_ema_ += std::clamp(alpha * (raw - dev_ema_), -static_cast<float>(DEV_MAX_STEP_REF_PX),
                             static_cast<float>(DEV_MAX_STEP_REF_PX));
    }
  } else if (state != LaneState::LOST) {
    state = LaneState::LOST; // co vach nhung khong dung duoc duong tam
    two_lanes = false;
  }

  // ---- 8b. Duong tam + vach toa do mat dat cho PathTracker ---------------
  Poly centre;
  if (have_aim) {
    centre = *path;
    const Poly &other = path == &c_left ? c_right : c_left;
    // Chi trung binh khi 2 duong cung "thay" hoac cung khong thay goc gap:
    // duong thang (vach bi mep tam nhin cat) se keo goc cua ve phia di thang
    if (other.size() >= 2 && kinked[0] == kinked[1]) {
      // 2 vach: trung binh 2 duong tam o phan chong len nhau
      for (auto &p : centre) {
        bool in = false;
        const float d = dist_to_polyline(p, other, &in);
        if (!in || d >= 0.15f) {
          continue;
        }
        float best = std::numeric_limits<float>::max();
        cv::Point2f q = p;
        for (size_t k = 0; k + 1 < other.size(); ++k) {
          const cv::Point2f ab = other[k + 1] - other[k];
          const float L2 = ab.dot(ab);
          const float t = L2 > 1e-9f ? std::clamp((p - other[k]).dot(ab) / L2, 0.0f, 1.0f) : 0.0f;
          const cv::Point2f c = other[k] + ab * t;
          if (norm2(p - c) < best) {
            best = norm2(p - c);
            q = c;
          }
        }
        p = 0.5f * (p + q);
      }
    }
    if (blend_off_m_ != 0.0f) {
      for (auto &p : centre) {
        p.x += blend_off_m_;
      }
    }
    out.centre_g = resample(centre, 0.05f);
    if (use_l) {
      out.left_g = left_.pts;
    }
    if (use_r) {
      out.right_g = right_.pts;
    }
  }
  out.fit_ok = centre.size() >= 2;
  out.stamp = t_begin;
  auto to_work_px = [&](const Poly &g) {
    std::vector<cv::Point> p;
    for (const auto &q : g) {
      cv::Point2d ip;
      if (ground_to_img(q, ip)) {
        p.emplace_back(cvRound(ip.x), cvRound(ip.y));
      }
    }
    return p;
  };
  if (use_l) {
    out.left_pts = to_work_px(left_.pts);
  }
  if (use_r) {
    out.right_pts = to_work_px(right_.pts);
  }

  // Do cong + lech ngang xa/gan cua duong tam (day cung CURVE_CHORD_M)
  float heading_far = 0.0f;
  const Poly cpath = resample_lin(centre, 0.02f);
  const float total = poly_len(cpath);
  if (cpath.size() >= 2 && total >= 0.05f) {
    const float chord = std::min(CURVE_CHORD_M, 0.5f * total);
    auto along = [&](float d, bool from_end) {
      float acc = 0.0f;
      const size_t n = cpath.size();
      for (size_t k = 1; k < n; ++k) {
        const cv::Point2f &a = from_end ? cpath[n - k] : cpath[k - 1];
        const cv::Point2f &b = from_end ? cpath[n - k - 1] : cpath[k];
        const float l = norm2(b - a);
        if (acc + l >= d) {
          return a + (b - a) * ((d - acc) / std::max(l, 1e-6f));
        }
        acc += l;
      }
      return from_end ? cpath.front() : cpath.back();
    };
    const cv::Point2f tn = unit(along(chord, false) - cpath.front());
    const cv::Point2f tf = unit(cpath.back() - along(chord, true));
    heading_far = heading_of(tf);
    float dth = heading_far - heading_of(tn);
    if (dth > static_cast<float>(CV_PI)) {
      dth -= 2.0f * static_cast<float>(CV_PI);
    } else if (dth < -static_cast<float>(CV_PI)) {
      dth += 2.0f * static_cast<float>(CV_PI);
    }
    out.curvature = dth / std::max(0.15f, total - chord);
    out.path_dx_cm = 100.0f * (cpath.back().x - cpath.front().x);
  }

  // ---- 9. Giu gia tri khi mat lan, he so toc do --------------------------
  if (state != LaneState::LOST) {
    last_valid_time_ = t_begin;
  }
  const auto lost_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                           t_begin - last_valid_time_)
                           .count();
  const bool holding = state == LaneState::LOST && lost_ms <= HOLD_MS;
  if (state == LaneState::LOST && !holding) {
    ema_primed_ = false;
    jump_count_ = 0;
  }

  out.two_lanes = two_lanes;
  out.state = state;
  out.dev_px = (state != LaneState::LOST || holding)
                   ? static_cast<int>(std::lround(dev_ema_))
                   : 0;
  if (state == LaneState::LOST) {
    out.speed_scale = 0.0f;
  } else {
    const float deg = std::fabs(heading_far) / kDeg;
    float curve = std::max(ramp(deg, CURVE_HEADING_START_DEG, CURVE_HEADING_FULL_DEG),
                           ramp(std::fabs(out.curvature), CURVE_K_START, CURVE_K_FULL));
    if (have_kink) {
      curve = std::max(curve, 1.0f - ramp(kink_dist, CORNER_SLOW_M, CORNER_SLOW_START_M));
    }
    out.speed_scale = 1.0f - curve;
  }

  out.horizon_frac = static_cast<float>(horizon_y_ / WH);
  out.pitch_deg = static_cast<float>(pitch_ * 180.0 / CV_PI);
  out.pitch_confirmed = pitch_confirmed_;
  out.near_z_m = z_min_ + 0.03f;
  out.dark_ratio = dark_ratio_;
  out.proc_ms = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - t_wall)
                    .count();

  if (!draw) {
    return two_lanes;
  }

  // ---- 10. Anh quan sat cho GUI (chi khi co nguoi xem) -------------------
  const int vis_h = std::max(
      1, static_cast<int>(std::lround(static_cast<double>(VIS_W) * frame.rows / frame.cols)));
  cv::Mat vis;
  cv::resize(frame, vis, cv::Size(VIS_W, vis_h), 0, 0, cv::INTER_AREA);
  out.raw = vis.clone();
  const double vx = static_cast<double>(VIS_W) / WORK_W;
  const double vy = static_cast<double>(vis_h) / WH;

  auto img_poly = [&](const Poly &g) {
    std::vector<cv::Point> p;
    for (const auto &q : g) {
      cv::Point2d ip;
      if (ground_to_img(q, ip) && ip.y > -WH && ip.y < 2 * WH) {
        p.emplace_back(cvRound(ip.x * vx), cvRound(ip.y * vy));
      }
    }
    return p;
  };

  // BEV: anh san toi, mask vach xam, phan bi loc tim, ngoai tam nhin xanh dam
  cv::Mat bv = bev * 0.6;
  bv.setTo(cv::Scalar(120, 0, 120), rejected);
  bv.setTo(cv::Scalar(200, 200, 200), clean);
  cv::Mat invalid;
  cv::bitwise_not(valid_, invalid);
  bv.setTo(cv::Scalar(40, 0, 0), invalid);
  const int bs = 3; // phong to BEV cho de nhin
  cv::resize(bv, bv, cv::Size(nx_ * bs, nz_ * bs), 0, 0, cv::INTER_NEAREST);
  auto bev_px = [&](const cv::Point2f &g) {
    float c = 0.0f, r = 0.0f;
    px_of(g, c, r);
    return cv::Point(cvRound((c + 0.5f) * bs), cvRound((r + 0.5f) * bs));
  };
  auto bev_poly = [&](const Poly &g) {
    std::vector<cv::Point> p;
    for (const auto &q : g) {
      p.push_back(bev_px(q));
    }
    return p;
  };
  auto draw_both = [&](const Poly &g, const cv::Scalar &c, int th) {
    if (g.size() < 2) {
      return;
    }
    cv::polylines(bv, bev_poly(g), false, c, th, cv::LINE_AA);
    const auto p = img_poly(g);
    if (p.size() > 1) {
      cv::polylines(vis, p, false, c, th + 1, cv::LINE_AA);
    }
  };

  // Vung lan (to mau trong suot giua 2 vach)
  if (two_lanes) {
    std::vector<cv::Point> area = img_poly(left_.pts);
    const auto rp = img_poly(right_.pts);
    area.insert(area.end(), rp.rbegin(), rp.rend());
    if (area.size() >= 3) {
      cv::Mat layer = vis.clone();
      cv::fillPoly(layer, std::vector<std::vector<cv::Point>>{area},
                   cv::Scalar(80, 200, 80), cv::LINE_AA);
      cv::addWeighted(layer, 0.28, vis, 0.72, 0.0, vis);
    }
  }
  // Ung vien khong duoc chon: xam mo
  for (const auto &c : cands) {
    draw_both(c, cv::Scalar(110, 110, 110), 1);
  }
  // Vach: trai xanh la, phai cam; du doan (khong thay) / chua xac minh: xam
  if (left_.valid) {
    draw_both(left_.pts, use_l ? cv::Scalar(0, 230, 0) : cv::Scalar(160, 160, 160), 2);
  }
  if (right_.valid) {
    draw_both(right_.pts, use_r ? cv::Scalar(255, 120, 0) : cv::Scalar(160, 160, 160), 2);
  }
  // Duong tam lan (vang) + diem ngam
  draw_both(centre, cv::Scalar(0, 230, 255), 1);
  if (have_aim) {
    cv::circle(bv, bev_px(aim), 5, cv::Scalar(0, 230, 255), -1, cv::LINE_AA);
    cv::Point2d ip;
    if (aim.y > 0.05f && ground_to_img(aim, ip)) {
      const cv::Point ap(cvRound(ip.x * vx), cvRound(ip.y * vy));
      cv::circle(vis, ap, 7, cv::Scalar(0, 230, 255), 2, cv::LINE_AA);
      cv::line(vis, cv::Point(VIS_W / 2, vis_h - 1), ap, cv::Scalar(0, 230, 255), 1, cv::LINE_AA);
    }
  }
  if (have_kink) {
    cv::drawMarker(bv, bev_px(kink_pt), cv::Scalar(255, 0, 255), cv::MARKER_TILTED_CROSS, 14, 2);
    cv::Point2d ip;
    if (ground_to_img(kink_pt, ip)) {
      cv::drawMarker(vis, cv::Point(cvRound(ip.x * vx), cvRound(ip.y * vy)),
                     cv::Scalar(255, 0, 255), cv::MARKER_TILTED_CROSS, 16, 2);
    }
  }
  // Mui ten huong xe o day BEV
  cv::arrowedLine(bv, cv::Point(nx_ * bs / 2, nz_ * bs - 2), cv::Point(nx_ * bs / 2, nz_ * bs - 22),
                  cv::Scalar(255, 255, 255), 2);
  out.roi = bv;
  // Anh BINARY cho GUI: chi pixel cua 2 vach dang dung de lai (trang), bo
  // het do vat / vach chua xac minh
  {
    cv::Mat lanes = cv::Mat::zeros(clean.size(), CV_8U);
    const int th = std::max(3, static_cast<int>(std::lround(0.10f / BEV_RES_M)));
    for (const Poly *g : {use_l ? &left_.pts : nullptr, use_r ? &right_.pts : nullptr}) {
      if (g && g->size() >= 2) {
        float c = 0.0f, r = 0.0f;
        std::vector<cv::Point> p;
        for (const auto &q : *g) {
          px_of(q, c, r);
          p.emplace_back(cvRound(c), cvRound(r));
        }
        cv::polylines(lanes, p, false, cv::Scalar(255), th);
      }
    }
    cv::bitwise_and(lanes, clean, lanes);
    out.bin = lanes / 255;
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
      flag_text = use_l ? "1 LANE (L)" : "1 LANE (R)";
    }
    if (out.gated) {
      flag_text += " *";
    }
    cv::putText(vis, flag_text, cv::Point(8, 18), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                flag_color, 1, cv::LINE_AA);
    char info[220];
    std::snprintf(info, sizeof(info),
                  "dev %+dpx  w %.0fcm  v %.0f%%  pitch %.1f%s  thr %.2f%s",
                  out.dev_px, lane_w_local_ * 100.0f, out.speed_scale * 100.0f,
                  out.pitch_deg, pitch_confirmed_ ? "" : "?", dark_ratio_,
                  have_kink ? (kink_dir > 0 ? "  CUA >>" : "  CUA <<") : "");
    cv::putText(vis, info, cv::Point(118, 18), cv::FONT_HERSHEY_SIMPLEX, 0.40,
                cv::Scalar(235, 235, 235), 1, cv::LINE_AA);
  }

  // Chen BEV thu nho goc tren ben TRAI (ben phai la ban do PathTracker)
  {
    const double s = std::min(0.50 * vis_h / bv.rows, 0.34 * VIS_W / bv.cols);
    cv::Mat small;
    cv::resize(bv, small, cv::Size(), s, s, cv::INTER_AREA);
    const int x0 = 4;
    const int y0 = 30;
    if (x0 >= 0 && y0 + small.rows <= vis_h) {
      small.copyTo(vis(cv::Rect(x0, y0, small.cols, small.rows)));
      cv::rectangle(vis, cv::Rect(x0, y0, small.cols, small.rows), cv::Scalar(200, 200, 200), 1);
    }
  }

  out.vis = vis;
  return two_lanes;
}
