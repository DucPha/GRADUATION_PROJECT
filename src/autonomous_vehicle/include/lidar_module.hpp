#pragma once

#include <opencv2/opencv.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <sensor_msgs/msg/laser_scan.hpp>

// ============================================================================
// LIDAR STATUS
// ============================================================================

struct LidarStatus {

    // Local Grid Map points
    std::vector<cv::Point> points_px;

    // Original zones (for fusion / visualization)
    std::optional<float> front_min_cm;
    std::optional<float> right_min_cm;
    std::optional<float> left_min_cm;
    std::optional<float> rear_bypass_min_cm;

    // Obstacle avoidance zones
    std::optional<float> ob_front_cm;
    std::optional<float> ob_right_cm;
    std::optional<float> ob_left_cm;
    std::optional<float> ob_right_rear_cm;
    std::optional<float> ob_left_rear_cm;

    // Safety status
    std::string alert = "CLEAR";
    std::string detail;
    bool has_data = false;
};

// ============================================================================
// LIDAR MODULE
// ============================================================================

class LidarModule {

public:

    static constexpr int MAP_W = 600;
    static constexpr int MAP_H = 600;
    static constexpr int MAP_RADIUS_CM = 80;
    static constexpr float PX_PER_CM = 3.75f;

    static constexpr int DETECT_RADIUS_CM = 200;

    static constexpr float SAFE_FRONT_DISTANCE = 60.0f;
    static constexpr float DANGER_FRONT_DISTANCE = 40.0f;
    static constexpr float SAFE_SIDE_DISTANCE = 30.0f;

    static constexpr float BYPASS_MIN_SPACE = 60.0f;
    static constexpr float BYPASS_COMPLETE_DISTANCE = 60.0f;

    static constexpr size_t MAP_POINT_RESERVE = 1024;

    static inline cv::Point origin() {
        return { MAP_W / 2, MAP_H / 2 };
    }

    static constexpr float px_per_cm() {
        return PX_PER_CM;
    }

    // Original API (Standalone)
    LidarStatus update(
        const sensor_msgs::msg::LaserScan& scan
    );

    // Optimized API (Zero-Allocation for ROS callbacks)
    void update(
        const sensor_msgs::msg::LaserScan& scan,
        LidarStatus& out
    );

private:

    static inline float normalize_deg(float degree) {
        if (degree < 0.0f) {
            degree += 360.0f;
            if (degree < 0.0f) {
                degree = std::fmod(degree, 360.0f) + 360.0f;
            }
        } else if (degree >= 360.0f) {
            degree -= 360.0f;
            if (degree >= 360.0f) {
                degree = std::fmod(degree, 360.0f);
            }
        }
        return degree;
    }

    static inline float norm_deg(float rad) {
        return normalize_deg(rad * 180.0f / static_cast<float>(CV_PI));
    }

    static inline bool in_range(float angle_deg, float angle_start, float angle_end) {
        if (angle_start <= angle_end) {
            return angle_deg >= angle_start && angle_deg <= angle_end;
        }
        return angle_deg >= angle_start || angle_deg <= angle_end;
    }

    static inline void update_min(std::optional<float>& current, float value) {
        if (!current || value < *current) {
            current = value;
        }
    }

    void update_geometry_cache(
        const sensor_msgs::msg::LaserScan& scan
    );

    std::vector<float> angle_deg_cache_;
    std::vector<float> cos_cache_;
    std::vector<float> sin_cache_;

    size_t cached_scan_size_ = 0;
    float cached_angle_min_ = 0.0f;
    float cached_angle_increment_ = 0.0f;
    bool geometry_cache_valid_ = false;
};