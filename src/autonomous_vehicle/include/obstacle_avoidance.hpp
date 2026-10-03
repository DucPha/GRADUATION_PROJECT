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
          // Mặc định DỪNG, không phải 11.5 km/h. Giá trị 115 của bản cũ là
          // một state đã bị xoá; nếu có bất kỳ đường nào quên gán speed thì
          // xe lập tức chạy 11.5 km/h. Mặc định 0 nghĩa là "chưa có quyết
          // định" -> an toàn.
          //
          // Dùng literal 0 vì khối hằng số tốc độ nằm DƯỚI struct này.
          speed_control(0),
          dev_final_px(0),
          emergency_stop(false),
          camera_stale(false),
          lidar_stale(false) {}
};

// Ngưỡng hạn dữ liệu (ms). Vượt ngưỡng -> lái xe theo chế độ an toàn.
static constexpr unsigned long CAMERA_STALE_MS = 200;
static constexpr unsigned long LIDAR_STALE_MS = 500;

// ============================================================================
// TỐC ĐỘ YÊU CẦU
// ============================================================================
// NORMAL lấy tốc độ do CameraLane lập (STRAIGHT 85 / CURVE 60 / SHARP 45,
// tức 8.5 / 6.0 / 4.5 km/h) thay vì hằng số cứng. Bản cũ gán cứng 35 cho
// NORMAL và 30 cho mọi state lách, nên bộ lập tốc độ trong detector được
// tính ra rồi bỏ không: xe chạy 3.5 km/h vào cua gấp y hệt đường thẳng.
//
// Các state có ràng buộc riêng (đang né vật cản, phanh, trả làn) giữ nguyên
// giá trị cũ - khi đang né vật cản thì tốc độ phải do logic an toàn quyết,
// không phải detector.

static constexpr uint8_t SPEED_HOLD_X10 = 0;      // dừng / phanh
static constexpr uint8_t SPEED_BYPASS_X10 = 30;   // né tránh, trả làn

// Detector mất lane -> target_speed_x10 = 0. KHÔNG dừng: xe còn đang trong làn,
// chỉ là detector chập chờn. Dừng sẽ giật xe mỗi lần detector mất vài frame;
// LiDAR vẫn lo phần né vật cản.
static constexpr uint8_t SPEED_NO_LANE_X10 = 30;

// Trần cho NORMAL, khớp SPEED_STRAIGHT_X10 của detector.
static constexpr uint8_t SPEED_NORMAL_MAX_X10 = 85;

// Ghi đè tốc độ NORMAL bằng tham số ROS (0 = dùng tốc độ detector). Giữ để
// chỉnh ngay trên xe khi hằng số bánh chưa đo thật: đặt 35 là quay lại
// hành vi cũ, đặt 0 là để detector quyết hoàn toàn.
static uint8_t g_speed_normal_override_x10 = 0;

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
                         uint8_t lane_target_speed_x10,
                         bool camera_stale = false,
                         bool lidar_stale = false);
BypassState get_current_state() const { return current_state_; }

    // Ghi đè tốc độ NORMAL (km/h x 10). 0 = dùng tốc độ detector.
    static void set_speed_normal_override(uint8_t x10) {
        g_speed_normal_override_x10 = x10;
    }
    static uint8_t get_speed_normal_override() {
        return g_speed_normal_override_x10;
    }

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

    // Tốc độ detector yêu cầu ở lượt update() gần nhất (km/h x 10), dùng cho
    // NORMAL. 0 = detector không có lane hợp lệ.
    uint8_t lane_speed_x10_ = 0;

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
                              uint8_t lane_target_speed_x10,
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

    // Tốc độ dùng cho NORMAL, lấy từ bộ lập của detector. Trả
    // SPEED_NO_LANE_X10 nếu detector không có lane hợp lệ, và luôn kẹp ở
    // SPEED_NORMAL_MAX_X10.
    uint8_t normal_speed_x10() const;
};

#endif // OBSTACLE_AVOIDANCE_HPP
