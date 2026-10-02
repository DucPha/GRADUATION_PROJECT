// obstacle_avoidance.hpp
#ifndef OBSTACLE_AVOIDANCE_HPP
#define OBSTACLE_AVOIDANCE_HPP

#include <chrono>
#include <cstdint>
#include <iostream>
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
          speed_control(35),
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

    // Trần góc lái gửi xuống ESP32. Firmware tự kẹp ở CAM_MAX_DEV (=50 px) nên
    // vượt giá trị này chỉ làm mất thông tin, không tăng độ lái. Để public vì
    // fusion_viz_node cũng phải kẹp sau khi trộn thêm bù lái của biển rẽ:
    // nếu mỗi bên tự kẹp bằng một con số khác nhau, góc lái cuối cùng gửi đi
    // phụ thuộc vào thứ tự cộng.
    static constexpr int16_t MAX_STEER_PX = 60;

    static int16_t clamp_px(int32_t v) {
        if (v > MAX_STEER_PX) return MAX_STEER_PX;
        if (v < -MAX_STEER_PX) return -MAX_STEER_PX;
        return static_cast<int16_t>(v);
    }

private:
    // Mốc thời gian an toàn cho các trạng thái trung gian (ms).
    // Không có mốc này, SWERVE có thể giữ góc lái cứng vô hạn.
    static constexpr unsigned long SWERVE_TIMEOUT_MS = 3000;
    static constexpr unsigned long BYPASS_TIMEOUT_MS = 8000;
    static constexpr unsigned long RETURN_TIMEOUT_MS = 5000;
    static constexpr unsigned long STOP_SETTLE_MS = 250;

    // Khoảng cách an toàn tối thiểu bên hông để quyết định lách (cm).
    // Đây là bản duy nhất: LidarModule chỉ đo, không quyết định lách.
    static constexpr float MIN_BYPASS_SPACE_CM = 60.0f;

    // Điều kiện "đã lách qua đuôi vật cản" (dùng ở handle_bypass_*):
    //  - vùng sau-bên phải/vuốt sau phải phải trống hơn BYPASS_REAR_CLEAR_CM
    //  - vùng bên phải/trái phải MỞ RỘNG hơn lúc bắt đầu SWERVE một lượng
    //    tối thiểu, tức xe đã thực sự dịch sang bên chứ không chỉ đứng yên.
    static constexpr float BYPASS_REAR_CLEAR_CM = 30.0f;
    static constexpr float BYPASS_SIDE_OPEN_DELTA_CM = 5.0f;

    // Thời gian giữ trạng thái BYPASS tối thiểu trước khi bắt đầu kiểm tra
    // điều kiện hoàn tất. Không có mốc này, một lần quét sạch ngay sau khi
    // SWERVE sẽ kết thúc lách quá sớm.
    static constexpr unsigned long BYPASS_SETTLE_MS = 500;

    // Trần tốc độ cho từng trạng thái (đơn vị km/h x 10). Ở NORMAL để 255 để
    // bộ lập tốc độ của camera (theo độ cong) là người quyết định; trước đây
    // đặt cứng 35 khiến xe chạy 3.5km/h bất kể camera tính ra bao nhiêu.
    static constexpr uint8_t SPEED_CEILING_NORMAL = 255;
    static constexpr uint8_t SPEED_SLOW_DOWN = 30;
    static constexpr uint8_t SPEED_SLOW_SHARP = 25;
    static constexpr uint8_t SPEED_SWERVE = 30;
    static constexpr uint8_t SPEED_BYPASS = 30;
    static constexpr uint8_t SPEED_RETURN = 30;

    // Góc lái bù thêm khi quay về làn (px). Giá trị cũ là ±80 cứng, vượt xa
    // dải sai số làn của camera (CAM_MAX_DEV = 50 px), nên chỉ cần góc lái
    // nhẹ là xe cắt hẳn sang làn kế.
    static constexpr int16_t RETURN_STEER_PX = 20;

    BypassState current_state_;
    unsigned long state_start_time_;
    
    std::optional<float> prev_bypass_left_dist_;
    std::optional<float> prev_bypass_right_dist_;
    std::optional<float> swerve_start_front_dist_;

    // Khoảng cách vật cản bên (đã né) chụp lại lúc bắt đầu SWERVE.
    // Dùng để biết đã "đi qua xong" chưa. Trước đây code so sánh khoảng
    // cách BÊN với khoảng cách PHÍA TRƯỚC -> điều kiện hoàn tất vô nghĩa.
    std::optional<float> swerve_start_left_dist_;
    std::optional<float> swerve_start_right_dist_;
    
    std::string prev_traffic_light_;
    
    // ✅ THÊM: Lưu tốc độ và margin
    float current_speed_kmh_;
    float speed_margin_cm_;

    // Sai số làn + độ cong của chu kỳ camera hiện tại, lưu lại mỗi lần
    // update(). Nhờ vậy các handler chuyển trạng thái (RETURN_LANE_*, kết thúc
    // EMERGENCY_STOP) vẫn trao lại đúng thông tin cho handle_normal, thay vì
    // dùng giá trị lái cứng và độ cong 0 làm mất đường cong khi rời trạng
    // thái tránh né.
    int16_t dev_px_ = 0;
    float dominant_slope_ = 0.0f;
    
    // State handlers
    // dominant_slope: bắt buộc truyền xuống, nếu không speed planner sẽ mất
    // thông tin độ cong mỗi khi rời trạng thái tránh né.
    BypassCommand handle_normal(const LidarStatus& lidar, int16_t dev_px,
                                float dominant_slope);
    BypassCommand handle_slow_down(const LidarStatus& lidar, float dominant_slope,
                                   int16_t dev_px, uint64_t now_ms);
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
    static unsigned long now_ms();
};

#endif // OBSTACLE_AVOIDANCE_HPP
