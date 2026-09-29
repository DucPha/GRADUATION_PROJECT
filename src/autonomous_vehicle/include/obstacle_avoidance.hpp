// obstacle_avoidance.hpp
#ifndef OBSTACLE_AVOIDANCE_HPP
#define OBSTACLE_AVOIDANCE_HPP

#include <iostream>
#include <optional>
#include <chrono>

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

// BYPASS COMMAND
struct BypassCommand {
    BypassState state;
    std::string state_name;
    uint8_t speed_control;
    int16_t dev_final_px;
    bool emergency_stop;

    BypassCommand()
        : state(BypassState::NORMAL),
          state_name("NORMAL"),
          speed_control(115),
          dev_final_px(0),
          emergency_stop(false) {}
};

// OBSTACLE AVOIDANCE CLASS
class ObstacleAvoidance {
public:
    ObstacleAvoidance();

    // ✅ THÊM current_speed_kmh
    BypassCommand update(const LidarStatus& lidar,
                         int16_t dev_px,
                         float dominant_slope,
                         bool is_dual_lane,
                         const std::string& traffic_light_decision,
                         float current_speed_kmh);

    BypassState get_current_state() const { return current_state_; }
    
    // ✅ THÊM: Lấy speed margin để truyền sang camera
    float get_speed_margin_cm() const { return speed_margin_cm_; }

private:
    BypassState current_state_;
    unsigned long state_start_time_;
    
    std::optional<float> prev_bypass_left_dist_;
    std::optional<float> prev_bypass_right_dist_;
    std::optional<float> swerve_start_front_dist_;
    
    std::string prev_traffic_light_;
    
    // ✅ THÊM: Lưu tốc độ và margin
    float current_speed_kmh_;
    float speed_margin_cm_;
    
    // State handlers
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

    // Helper
    void change_state(BypassState new_state);
    unsigned long get_state_elapsed_ms() const;
};

#endif // OBSTACLE_AVOIDANCE_HPP
