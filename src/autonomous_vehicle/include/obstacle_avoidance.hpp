// obstacle_avoidance.hpp
#ifndef OBSTACLE_AVOIDANCE_HPP
#define OBSTACLE_AVOIDANCE_HPP

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

// Forward declaration
struct LidarStatus;

// BYPASS STATE
enum class BypassState {
    NORMAL = 0,
    SLOW_DOWN = 1,
    DETECT_BYPASS_SIDE = 2,
    SWERVE_LEFT = 3,
    SWERVE_RIGHT = 4,
    BYPASS_LEFT = 5,
    BYPASS_RIGHT = 6,
    RETURN_LANE_LEFT = 7,
    RETURN_LANE_RIGHT = 8,
    EMERGENCY_STOP = 9
};

// Tên trạng thái dạng chuỗi, dùng cho log, GUI và topic chẩn đoán.
// Không dùng std::to_string vì enum class không có operator<<.
inline const char* bypass_state_name(BypassState s) {
    switch (s) {
        case BypassState::NORMAL:               return "NORMAL";
        case BypassState::SLOW_DOWN:            return "SLOW_DOWN";
        case BypassState::DETECT_BYPASS_SIDE:   return "DETECT_BYPASS";
        case BypassState::SWERVE_LEFT:          return "SWERVE_LEFT";
        case BypassState::SWERVE_RIGHT:         return "SWERVE_RIGHT";
        case BypassState::BYPASS_LEFT:          return "BYPASS_LEFT";
        case BypassState::BYPASS_RIGHT:         return "BYPASS_RIGHT";
        case BypassState::RETURN_LANE_LEFT:     return "RETURN_LEFT";
        case BypassState::RETURN_LANE_RIGHT:    return "RETURN_RIGHT";
        case BypassState::EMERGENCY_STOP:       return "EMERGENCY";
    }
    return "UNKNOWN";
}

// BYPASS COMMAND
struct BypassCommand {
    BypassState state;
    std::string state_name;
    uint8_t speed_control;
    int16_t dev_final_px;
    bool emergency_stop;
    // true khi camera quá hạn -> bộ điều khiển không được tin, phải giữ thẳng
    bool camera_stale;
    // true khi LiDAR quá hạn -> mất toàn bộ thông tin vật cản
    bool lidar_stale;

    BypassCommand()
        : state(BypassState::NORMAL),
          state_name("NORMAL"),
          speed_control(115),
          dev_final_px(0),
          emergency_stop(false),
          camera_stale(false),
          lidar_stale(false) {}
};

// Ngưỡng hạn dữ liệu (ms). Vượt ngưỡng -> lái xe theo chế độ an toàn.
static constexpr unsigned long CAMERA_STALE_MS = 200;
static constexpr unsigned long LIDAR_STALE_MS = 500;

// OBSTACLE AVOIDANCE CLASS
class ObstacleAvoidance {
public:
    ObstacleAvoidance();

    BypassCommand update(const LidarStatus& lidar,
                         int16_t dev_px,
                         float dominant_slope,
                         bool is_dual_lane,
                         const std::string& traffic_light_decision,
                         float current_speed_kmh,
                         bool camera_stale = false,
                         bool lidar_stale = false);
BypassState get_current_state() const { return current_state_; }

    // Tốc độ yêu cầu gửi ESP32 ở lượt update() gần nhất (km/h × 10).
    // Node điều khiển dùng giá trị này để gửi serial và dashboard dùng để
    // hiển thị, nên phải lưu lại thay vì chỉ trả về trong BypassCommand.
    uint8_t get_speed_command() const { return speed_command_; }

    // Lấy speed margin để truyền sang camera
    float get_speed_margin_cm() const { return speed_margin_cm_; }

private:

    BypassState current_state_;
    unsigned long state_start_time_;
    uint8_t speed_command_ = 0;

    std::optional<float> prev_bypass_left_dist_;
    std::optional<float> prev_bypass_right_dist_;
    std::optional<float> swerve_start_front_dist_;

    std::string prev_traffic_light_;

    float current_speed_kmh_;
    float speed_margin_cm_;

    // Lưu lần gọi gần nhất để các handler dùng chung vẫn ép được an toàn
    bool camera_stale_ = false;
    bool lidar_stale_ = false;

    // State handlers
    // Thân thật của update(). update() bọc lại để ghi speed_command_ ở một
    // chỗ duy nhất, vì các handler có nhiều nhánh return sớm.
    BypassCommand update_impl(const LidarStatus& lidar,
                              int16_t dev_px,
                              float dominant_slope,
                              bool is_dual_lane,
                              const std::string& traffic_light_decision,
                              float current_speed_kmh,
                              bool camera_stale,
                              bool lidar_stale);

    BypassCommand handle_normal(const LidarStatus& lidar, int16_t dev_px);
    BypassCommand handle_slow_down(const LidarStatus& lidar, float dominant_slope, int16_t dev_px);
    BypassCommand handle_detect_bypass_side(const LidarStatus& lidar);
    BypassCommand handle_swerve_left(const LidarStatus& lidar);
    BypassCommand handle_swerve_right(const LidarStatus& lidar);
    BypassCommand handle_bypass_left(const LidarStatus& lidar);
    BypassCommand handle_bypass_right(const LidarStatus& lidar);
    BypassCommand handle_return_lane_left(const LidarStatus& lidar, bool is_dual_lane);
    BypassCommand handle_return_lane_right(const LidarStatus& lidar, bool is_dual_lane);
    BypassCommand handle_emergency_stop(const LidarStatus& lidar, int16_t dev_px,
                                        const std::string& traffic_light_decision);
    BypassCommand handle_camera_stale(const LidarStatus& lidar);
    BypassCommand handle_lidar_stale(int16_t dev_px);

    // Helper
    void change_state(BypassState new_state);
    unsigned long get_state_elapsed_ms() const;
};

#endif // OBSTACLE_AVOIDANCE_HPP
