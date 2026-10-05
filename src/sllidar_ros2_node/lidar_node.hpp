#pragma once

#include <opencv2/opencv.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <sensor_msgs/msg/laser_scan.hpp>

// ============================================================================
// QUY ƯỚC GÓC
// ----------------------------------------------------------------------------
// Theo chuẩn REP-103 của ROS: LaserScan.angle_min + i*angle_increment là góc
// đo NGƯỢC chiều kim đồng hồ tính từ trục +X của xe (trục +X = PHÍA TRƯỚC).
// Bản vẽ bản đồ trong fusion_viz_node.cpp dùng đúng quy ước đó:
//     x = origin.x + cos(a)*d      (0° -> bên phải ảnh)
//     y = origin.y - sin(a)*d      (90° -> phía trên ảnh)
// nên 0° = TRƯỚC, 90° = TRÁI, 180° = SAU, 270° = PHẢI.
//
// Vòng tròn vòng trước đây đảo 90°: "front" lấy 60..120 (thực chất là BÊN TRÁI)
// và "right" lấy 300..360 u 0..60 (thực chất là PHÍA TRƯỚC). Hậu quả:
// ob_front_cm không bao giờ thấy vật cản phía trước -> xe không giảm tốc.
//
// Nếu LiDAR được lắp xoay vật lý, dùng tham số ROS `lidar_mount_offset_deg`
// (độ) để bù. Ví dụ LiDAR đặt đầu dòng chĩa sang bên trái -> offset = -90.
// ============================================================================

// ============================================================================
// LIDAR STATUS
// ============================================================================

struct LidarStatus {

    // Local Grid Map points
    std::vector<cv::Point> points_px;

    // Vùng hiển thị HUD: 4 góc vuông 90°, không chồng lấn.
    //   front  = trước, left = trái, rear_bypass = sau, right = phải
    std::optional<float> front_min_cm;
    std::optional<float> right_min_cm;
    std::optional<float> left_min_cm;
    std::optional<float> rear_bypass_min_cm;

    // Vùng cho logic né tránh. 8 góc vuông 45° quanh xe:
    //   FRONT_LEFT  FRONT  FRONT_RIGHT
    //   LEFT            REAR            RIGHT        <- góc nhìn từ trên xuống
    //   REAR_LEFT  REAR  REAR_RIGHT
    // Làn trống để lách = hợp của FRONT_LEFT + LEFT (và đối xứng bên phải),
    // phía sau dùng REAR_LEFT / REAR_RIGHT để biết đã lách qua chưa.
    std::optional<float> ob_front_cm;
    std::optional<float> ob_right_cm;
    std::optional<float> ob_left_cm;
    std::optional<float> ob_right_rear_cm;
    std::optional<float> ob_left_rear_cm;
    std::optional<float> ob_rear_cm;

    // Safety status
    std::string alert = "CLEAR";
    std::string detail;
    bool has_data = false;      // còn điểm đo hợp lệ => cảm biến còn sống
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

    static constexpr int DETECT_RADIUS_CM = 500;

    static constexpr float SAFE_FRONT_DISTANCE = 60.0f;
    static constexpr float DANGER_FRONT_DISTANCE = 40.0f;
    static constexpr float SAFE_SIDE_DISTANCE = 30.0f;

    // LƯU Ý: các ngưỡng lách (khoảng trống bên tối thiểu, điều kiện "đã lách
    // qua xong") thuộc về logic state machine, nên nằm ở ObstacleAvoidance
    // (MIN_BYPASS_SPACE_CM, BYPASS_REAR_CLEAR_CM, BYPASS_SIDE_OPEN_DELTA_CM).
    // Trước đây khaihai bản sao ở đây rồi không ai dùng -> hai nơi có thể lệch
    // nhau mà không ai phát hiện.

    static constexpr size_t MAP_POINT_RESERVE = 1024;

    // ---------------------------------------------------------------------
    // Biên vùng. Toàn bộ tính toán DIỄN RA TRONG HỆ TOẠ ĐỘ XE theo REP-103:
    //     0° = TRƯỚC   90° = TRÁI   180° = SAU   270° = PHẢI
    // Không có mốc "phía trước" nào khác; chỉ có góc hiệu chỉnh lắp đặt bên
    // dưới. Như vậy sửa/ghi lại logic không bao giờ phải nhân thêm hệ số nào.
    // ---------------------------------------------------------------------
    static constexpr float SECTOR_HALF = 22.5f;   // nửa góc vuông 45°
    static constexpr float QUAD_HALF = 45.0f;      // nửa góc vuông 90°

    // Góc lệch lắp đặt mặc định: -90° nghĩa là đầu dòng LiDAR chĩa sang bên
    // trái của xe, tức góc thô 90° ứng với phía trước. Đây đúng bằng giả định
    // của bản cũ ("front" lấy 60..120°) nên hành vi không đổi, nhưng giờ đã
    // nằm đúng khung xe. Nếu LiDAR lắp khác, đổi tham số ROS
    // `lidar_mount_offset_deg`.
    static constexpr float DEFAULT_MOUNT_OFFSET_DEG = -90.0f;

    static inline cv::Point origin() {
        return { MAP_W / 2, MAP_H / 2 };
    }

    static constexpr float px_per_cm() {
        return PX_PER_CM;
    }

    void set_mount_offset_deg(float offset_deg) { mount_offset_deg_ = offset_deg; }
    float get_mount_offset_deg() const { return mount_offset_deg_; }

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

    // Chuẩn hoá góc về [0, 360)
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

    // Kiểm tra góc có nằm trong cung [start, end) không. Tự xử lý cung vắt qua 0.
    // Dùng khoảng nửa mở để các cung liền kề là KHÔNG chồng lấn ở đường biên,
    // nếu không một tia đúng trên biên sẽ được tính vào hai cung và ô "coi là
    // trống" bị sai lệch.
    static inline bool in_sector(float angle_deg, float start, float end) {
        const float s = normalize_deg(start);
        const float e = normalize_deg(end);
        if (s < e) return angle_deg >= s && angle_deg < e;
        return angle_deg >= s || angle_deg < e;   // cung vắt qua 0
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

    // Góc lệch lắp đặt (độ). Mặc định -90 để raw 90° = phía trước xe.
    float mount_offset_deg_ = DEFAULT_MOUNT_OFFSET_DEG;
    float cached_mount_offset_ = 1e9f;   // giá trị vô nghĩa => cache chưa dùng
};
