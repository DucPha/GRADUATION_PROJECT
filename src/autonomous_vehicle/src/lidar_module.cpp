#include "lidar_module.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

// ============================================================================
// SƠ ĐỒ CUNG ĐO (góc tính trong KHUNG XE)
// ----------------------------------------------------------------------------
// angle_deg_cache_[] đã cộng sẵn góc hiệu chỉnh `lidar_mount_offset_deg`, nên
// nó nằm trong hệ toạ độ xe theo đúng chuẩn REP-103 của ROS:
//        0° = TRƯỚC      90° = TRÁI      180° = SAU      270° = PHẢI
//
//   cung 45°   : FRONT 337.5..22.5 | F.R 22.5..67.5 | RIGHT 247.5..292.5
//                R.R 292.5..337.5  | REAR 157.5..202.5 | R.L 112.5..157.5
//                LEFT  67.5..112.5
//   cung 90° HUD: FRONT 315..45 | LEFT 45..135 | REAR 135..225 | RIGHT 225..315
//
// Vì cung trước và hai cung bên là các khoảng rời nhau, vật cản đứng thẳng
// trước xe KHÔNG bao giờ lọt nhầm vào cung bên (lỗi của bản cũ).
// ============================================================================

namespace {

constexpr float S = 22.5f;   // nửa góc vuông 45°
constexpr float Q = 45.0f;   // nửa góc vuông 90°

} // namespace

// ============================================================================
// GEOMETRY CACHE
// ============================================================================

void LidarModule::update_geometry_cache(
    const sensor_msgs::msg::LaserScan& scan
) {
    const size_t scan_size = scan.ranges.size();

    if (geometry_cache_valid_ &&
        cached_scan_size_ == scan_size &&
        cached_angle_min_ == scan.angle_min &&
        cached_angle_increment_ == scan.angle_increment &&
        cached_mount_offset_ == mount_offset_deg_) {
        return;
    }

    cached_scan_size_ = scan_size;
    cached_angle_min_ = scan.angle_min;
    cached_angle_increment_ = scan.angle_increment;
    cached_mount_offset_ = mount_offset_deg_;

    angle_deg_cache_.resize(scan_size);
    cos_cache_.resize(scan_size);
    sin_cache_.resize(scan_size);

    if (scan_size == 0) {
        geometry_cache_valid_ = true;
        return;
    }

    // Bản đồ vẽ theo GÓC ĐÃ HIỆU CHỈNH để xe luôn nằm "quay mặt lên trên"
    // (khớp với nhãn Front/Rear/Left/Right trên HUD).
    const float off_rad = mount_offset_deg_ * static_cast<float>(CV_PI) / 180.0f;

    float angle_rad = scan.angle_min;
    float angle_deg = normalize_deg(
        angle_rad * 180.0f / static_cast<float>(CV_PI) + mount_offset_deg_);
    const float angle_step_deg =
        scan.angle_increment * 180.0f / static_cast<float>(CV_PI);

    for (size_t i = 0; i < scan_size; ++i) {
        angle_deg_cache_[i] = angle_deg;
        cos_cache_[i] = std::cos(angle_rad + off_rad);
        sin_cache_[i] = std::sin(angle_rad + off_rad);

        // Offset là hằng số hiệu chỉnh, chỉ cộng một lần ở đầu chuỗi.
        angle_rad += scan.angle_increment;
        angle_deg = normalize_deg(angle_deg + angle_step_deg);
    }

    geometry_cache_valid_ = true;
}

// ============================================================================
// ORIGINAL API
// ============================================================================

LidarStatus LidarModule::update(
    const sensor_msgs::msg::LaserScan& scan
) {
    LidarStatus st;
    update(scan, st);
    return st;
}

// ============================================================================
// OPTIMIZED UPDATE
// ============================================================================

void LidarModule::update(
    const sensor_msgs::msg::LaserScan& scan,
    LidarStatus& out
) {
    out.points_px.clear();
    out.front_min_cm.reset();
    out.right_min_cm.reset();
    out.left_min_cm.reset();
    out.rear_bypass_min_cm.reset();
    out.ob_front_cm.reset();
    out.ob_right_cm.reset();
    out.ob_left_cm.reset();
    out.ob_right_rear_cm.reset();
    out.ob_left_rear_cm.reset();
    out.ob_rear_cm.reset();
    out.alert = "CLEAR";
    out.detail.clear();
    out.has_data = false;

    if (scan.ranges.empty()) {
        out.detail = "No LiDAR data";
        return;
    }

    update_geometry_cache(scan);

    const cv::Point O = origin();
    const float map_radius_cm = static_cast<float>(MAP_RADIUS_CM);
    const float detect_radius_cm = static_cast<float>(DETECT_RADIUS_CM);

    if (out.points_px.capacity() < MAP_POINT_RESERVE) {
        out.points_px.reserve(MAP_POINT_RESERVE);
    }

    const auto& ranges = scan.ranges;
    const auto& angle_deg = angle_deg_cache_;
    const auto& cos_table = cos_cache_;
    const auto& sin_table = sin_cache_;

    // ---- Cung vuông 90° (HUD): 4 góc không chồng lấn ----
    const float hud_front_lo = 360.0f - Q, hud_front_hi = Q;     // 315..45   TRƯỚC
    const float hud_left_lo  = Q, hud_left_hi  = 180.0f - Q;     //  45..135  TRÁI
    const float hud_rear_lo  = 180.0f - Q, hud_rear_hi = 180.0f + Q; // 135..225 SAU
    const float hud_right_lo = 180.0f + Q, hud_right_hi = 360.0f - Q; // 225..315 PHẢI

    // ---- Cung 45° (logic né tránh) ----
    // TRƯỚC: 337.5..22.5 (vắt qua 0)
    const float ob_front_lo = 360.0f - S, ob_front_hi = S;       // 337.5..22.5
    // Làn trống bên trái = F.L + LEFT + R.L = 22.5..157.5
    const float ob_left_lo  = S, ob_left_hi  = 180.0f - S;      //  22.5..157.5
    // Làn trống bên phải = R.R + RIGHT + F.R = 202.5..337.5
    const float ob_right_lo = 180.0f + S, ob_right_hi = 360.0f - S; // 202.5..337.5
    // Sau trái / sau phải (kiểm tra đã lách qua vật cản chưa)
    const float ob_left_rear_lo  = 180.0f - 2.0f * S, ob_left_rear_hi  = 180.0f; // 135..180
    const float ob_right_rear_lo = 180.0f, ob_right_rear_hi = 180.0f + 2.0f * S; // 180..225
    // Chính phía sau
    const float ob_rear_lo = 180.0f - S, ob_rear_hi = 180.0f + S; // 157.5..202.5

    size_t valid_point_count = 0;
    const float px_per_cm = PX_PER_CM;

    for (size_t i = 0; i < ranges.size(); ++i) {
        const float range_m = ranges[i];

        if (!std::isfinite(range_m)) continue;
        if (scan.range_min > 0.0f && range_m < scan.range_min) continue;
        if (scan.range_max > 0.0f && range_m > scan.range_max) continue;
        if (range_m <= 0.0f || range_m > detect_radius_cm * 0.01f) continue;

        // Còn điểm đo trong bán kính phát hiện => cảm biến còn sống.
        ++valid_point_count;

        const float distance_cm = range_m * 100.0f;
        const float deg = angle_deg[i];

        // ---- Vùng hiển thị HUD: 4 góc vuông 90°, không chồng lấn ----
        if (in_sector(deg, hud_front_lo, hud_front_hi)) {
            update_min(out.front_min_cm, distance_cm);
        }
        if (in_sector(deg, hud_left_lo, hud_left_hi)) {
            update_min(out.left_min_cm, distance_cm);
        }
        if (in_sector(deg, hud_rear_lo, hud_rear_hi)) {
            update_min(out.rear_bypass_min_cm, distance_cm);
        }
        if (in_sector(deg, hud_right_lo, hud_right_hi)) {
            update_min(out.right_min_cm, distance_cm);
        }

        // ---- Vùng cho logic né tránh ----
        if (in_sector(deg, ob_front_lo, ob_front_hi)) {
            update_min(out.ob_front_cm, distance_cm);
        }
        if (in_sector(deg, ob_left_lo, ob_left_hi)) {
            update_min(out.ob_left_cm, distance_cm);
        }
        if (in_sector(deg, ob_right_lo, ob_right_hi)) {
            update_min(out.ob_right_cm, distance_cm);
        }
        if (in_sector(deg, ob_left_rear_lo, ob_left_rear_hi)) {
            update_min(out.ob_left_rear_cm, distance_cm);
        }
        if (in_sector(deg, ob_right_rear_lo, ob_right_rear_hi)) {
            update_min(out.ob_right_rear_cm, distance_cm);
        }
        if (in_sector(deg, ob_rear_lo, ob_rear_hi)) {
            update_min(out.ob_rear_cm, distance_cm);
        }

        // ---- Bản đồ cục bộ ----
        if (distance_cm <= map_radius_cm) {
            const float distance_px = distance_cm * px_per_cm;
            const int x = static_cast<int>(std::lround(
                static_cast<float>(O.x) + cos_table[i] * distance_px));
            const int y = static_cast<int>(std::lround(
                static_cast<float>(O.y) - sin_table[i] * distance_px));

            if (x >= 0 && x < MAP_W && y >= 0 && y < MAP_H) {
                out.points_px.emplace_back(x, y);
            }
        }
    }

    out.has_data = (valid_point_count > 0);

    if (!out.has_data) {
        out.alert = "CLEAR";
        out.detail = "No valid LiDAR points";
        return;
    }

    if (out.front_min_cm && *out.front_min_cm < DANGER_FRONT_DISTANCE) {
        out.alert = "DANGER";
        out.detail = "Front < 40cm";
        return;
    }

    if (out.front_min_cm && *out.front_min_cm < SAFE_FRONT_DISTANCE) {
        out.alert = "WARNING";
        out.detail = "Front < 60cm";
        return;
    }

    if ((out.left_min_cm && *out.left_min_cm < SAFE_SIDE_DISTANCE) ||
        (out.right_min_cm && *out.right_min_cm < SAFE_SIDE_DISTANCE)) {
        out.alert = "WARNING";
        out.detail = "Side < 30cm";
        return;
    }

    out.alert = "CLEAR";
    out.detail = "All clear";
}
