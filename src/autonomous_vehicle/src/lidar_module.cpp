#include "lidar_module.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

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
        cached_angle_increment_ == scan.angle_increment) {
        return;
    }

    cached_scan_size_ = scan_size;
    cached_angle_min_ = scan.angle_min;
    cached_angle_increment_ = scan.angle_increment;

    angle_deg_cache_.resize(scan_size);
    cos_cache_.resize(scan_size);
    sin_cache_.resize(scan_size);

    if (scan_size == 0) {
        geometry_cache_valid_ = true;
        return;
    }

    float angle_rad = scan.angle_min;
    float angle_deg = angle_rad * 180.0f / static_cast<float>(CV_PI);
    const float angle_step_deg = scan.angle_increment * 180.0f / static_cast<float>(CV_PI);

    for (size_t i = 0; i < scan_size; ++i) {
        float d = angle_deg;
        while (d < 0.0f) d += 360.0f;
        while (d >= 360.0f) d -= 360.0f;

        angle_deg_cache_[i] = d;
        cos_cache_[i] = std::cos(angle_rad);
        sin_cache_[i] = std::sin(angle_rad);

        angle_rad += scan.angle_increment;
        angle_deg += angle_step_deg;
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
    LidarStatus& st
) {
    st.points_px.clear();
    st.front_min_cm.reset();
    st.right_min_cm.reset();
    st.left_min_cm.reset();
    st.rear_bypass_min_cm.reset();
    st.ob_front_cm.reset();
    st.ob_right_cm.reset();
    st.ob_left_cm.reset();
    st.ob_right_rear_cm.reset();
    st.ob_left_rear_cm.reset();
    st.alert = "CLEAR";
    st.detail.clear();
    st.has_data = false;

    if (scan.ranges.empty()) {
        st.detail = "No LiDAR data";
        return;
    }

    update_geometry_cache(scan);

    const cv::Point O = origin();
    const float map_radius_cm = static_cast<float>(MAP_RADIUS_CM);
    const float detect_radius_cm = static_cast<float>(DETECT_RADIUS_CM);

    if (st.points_px.capacity() < MAP_POINT_RESERVE) {
        st.points_px.reserve(MAP_POINT_RESERVE);
    }

    const auto& ranges = scan.ranges;
    const auto& angle_deg = angle_deg_cache_;
    const auto& cos_table = cos_cache_;
    const auto& sin_table = sin_cache_;

    size_t valid_point_count = 0;
    const float px_per_cm = PX_PER_CM;

    for (size_t i = 0; i < ranges.size(); ++i) {
        const float range_m = ranges[i];

        if (!std::isfinite(range_m)) continue;

        if (scan.range_min > 0.0f && range_m < scan.range_min) continue;
        if (scan.range_max > 0.0f && range_m > scan.range_max) continue;

        if (range_m <= 0.0f || range_m > detect_radius_cm * 0.01f) continue;

        ++valid_point_count;

        const float distance_cm = range_m * 100.0f;
        const float deg = angle_deg[i];

        // Original fusion zones
        if (deg >= 60.0f && deg <= 120.0f) update_min(st.front_min_cm, distance_cm);
        if (deg >= 300.0f || deg <= 60.0f) update_min(st.right_min_cm, distance_cm);
        if (deg >= 120.0f && deg <= 240.0f) update_min(st.left_min_cm, distance_cm);
        if (deg >= 240.0f && deg <= 300.0f) update_min(st.rear_bypass_min_cm, distance_cm);

        // Obstacle avoidance zones
        if (deg >= 70.0f && deg <= 110.0f) update_min(st.ob_front_cm, distance_cm);
        if (deg >= 1.0f && deg <= 69.0f) update_min(st.ob_right_cm, distance_cm);
        if (deg >= 111.0f && deg <= 179.0f) update_min(st.ob_left_cm, distance_cm);
        if (deg >= 270.0f || deg <= 69.0f) update_min(st.ob_right_rear_cm, distance_cm);
        if (deg >= 111.0f && deg <= 270.0f) update_min(st.ob_left_rear_cm, distance_cm);

        // Local Grid Map
        if (distance_cm <= map_radius_cm) {
            const float distance_px = distance_cm * px_per_cm;
            const int x = static_cast<int>(std::lround(static_cast<float>(O.x) + cos_table[i] * distance_px));
            const int y = static_cast<int>(std::lround(static_cast<float>(O.y) - sin_table[i] * distance_px));

            if (x >= 0 && x < MAP_W && y >= 0 && y < MAP_H) {
                st.points_px.emplace_back(x, y);
            }
        }
    }

    st.has_data = (valid_point_count > 0);

    if (!st.has_data) {
        st.alert = "CLEAR";
        st.detail = "No valid LiDAR points";
        return;
    }

    if (st.front_min_cm && *st.front_min_cm < DANGER_FRONT_DISTANCE) {
        st.alert = "DANGER";
        st.detail = "Front < 40cm";
        return;
    }

    if (st.front_min_cm && *st.front_min_cm < SAFE_FRONT_DISTANCE) {
        st.alert = "WARNING";
        st.detail = "Front < 60cm";
        return;
    }

    if ((st.left_min_cm && *st.left_min_cm < SAFE_SIDE_DISTANCE) ||
        (st.right_min_cm && *st.right_min_cm < SAFE_SIDE_DISTANCE)) {
        st.alert = "WARNING";
        st.detail = "Side < 30cm";
        return;
    }

    st.alert = "CLEAR";
    st.detail = "All clear";
}