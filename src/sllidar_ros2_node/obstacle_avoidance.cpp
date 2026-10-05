// obstacle_avoidance.cpp
#include "obstacle_avoidance.hpp"
#include "lidar_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>

namespace {

// Nguồn thời gian duy nhất của state machine. Dùng steady_clock để đo ELAPSED
// (khoảng trôi qua), không dùng system_clock: đồng hồ hệ thống có thể nhảy
// khi NTP chỉnh, làm thời gian trạng thái âm.
unsigned long now_ms() {
    return static_cast<unsigned long>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
}

// Chống log spam: các handler chạy ở tần số điều khiển (100 Hz). std::cout
// không khóa nhưng vẫn tốn ~10-20 us/call và làm nhiễu console khi debug.
class Throttle {
public:
    explicit Throttle(unsigned long interval_ms)
        : interval_ms_(interval_ms) {}

    bool ready() {
        const unsigned long t = now_ms();
        if (last_ms_ == 0) {
            last_ms_ = t;
            return true;
        }
        if (t - last_ms_ >= interval_ms_) {
            last_ms_ = t;
            return true;
        }
        return false;
    }

private:
    unsigned long interval_ms_;
    unsigned long last_ms_ = 0;
};

}  // namespace

ObstacleAvoidance::ObstacleAvoidance()
    : current_state_(BypassState::NORMAL),
      // steady_clock cho mọi phép đo thời gian của state machine. Bản cũ khởi
      // tạo bằng steady_clock nhưng change_state()/get_state_elapsed_ms() đọc
      // system_clock: NTP chỉnh đồng hồ là state_start_time_ bị lệch hàng giờ
      // và get_state_elapsed_ms() có thể trả số âm -> so sánh < 300 / > 500
      // sai hoàn toàn (bỏ qua hoặc kẹt vĩnh viễn ở một trạng thái).
      state_start_time_(now_ms()),
      prev_bypass_left_dist_(std::nullopt),
      prev_bypass_right_dist_(std::nullopt),
      swerve_start_front_dist_(std::nullopt),
      prev_traffic_light_("NONE"),
      current_speed_kmh_(0.0f),
      speed_margin_cm_(0.0f) {}

void ObstacleAvoidance::change_state(BypassState new_state) {
    if (current_state_ != new_state) {
        const bool was_bypass =
            current_state_ == BypassState::BYPASS_LEFT ||
            current_state_ == BypassState::BYPASS_RIGHT;

        current_state_ = new_state;
        state_start_time_ = now_ms();

        // Rời trạng thái bypass -> reset bộ nhớ "đã ghi nhận vật cản" để lần lái
        // tiếp theo không dùng khoảng cách cũ làm ngưỡng so sánh.
        if (was_bypass &&
            new_state != BypassState::BYPASS_LEFT &&
            new_state != BypassState::BYPASS_RIGHT &&
            new_state != BypassState::SWERVE_LEFT &&
            new_state != BypassState::SWERVE_RIGHT) {
            prev_bypass_left_dist_ = std::nullopt;
            prev_bypass_right_dist_ = std::nullopt;
            swerve_start_front_dist_ = std::nullopt;
        }

        if (new_state == BypassState::BYPASS_LEFT) {
            prev_bypass_left_dist_ = std::nullopt;
        } else if (new_state == BypassState::BYPASS_RIGHT) {
            prev_bypass_right_dist_ = std::nullopt;
        } else if (new_state == BypassState::SWERVE_LEFT ||
                   new_state == BypassState::SWERVE_RIGHT) {
            swerve_start_front_dist_ = std::nullopt;
        }
    }
}

unsigned long ObstacleAvoidance::get_state_elapsed_ms() const {
    const unsigned long now = now_ms();
    return (now > state_start_time_) ? (now - state_start_time_) : 0ul;
}

BypassCommand ObstacleAvoidance::update(const LidarStatus& lidar,
                                        int16_t dev_px,
                                        float dominant_slope,
                                        bool is_dual_lane,
                                        const std::string& traffic_light_decision,
                                        float current_speed_kmh,
                                        uint8_t lane_target_speed_x10,
                                        bool camera_stale,
                                        bool lidar_stale) {
    const BypassCommand cmd = update_impl(
        lidar, dev_px, dominant_slope, is_dual_lane,
        traffic_light_decision, current_speed_kmh, lane_target_speed_x10,
        camera_stale, lidar_stale);

    // Ghi lại sau khi đã đi qua toàn bộ các nhánh con. Nhiều nhánh return
    // sớm (lidar_stale, camera_stale) nên ghi trong từng handler dễ sót,
    // còn đặt ở đây thì chắc chắn khớp đúng lệnh vừa gửi ESP32.
    speed_command_ = cmd.speed_control;
    return cmd;
}

BypassCommand ObstacleAvoidance::update_impl(const LidarStatus& lidar,
                                        int16_t dev_px,
                                        float dominant_slope,
                                        bool is_dual_lane,
                                        const std::string& traffic_light_decision,
                                        float current_speed_kmh,
                                        uint8_t lane_target_speed_x10,
                                        bool camera_stale,
                                        bool lidar_stale) {
    current_speed_kmh_ = current_speed_kmh;
    camera_stale_ = camera_stale;
    lidar_stale_ = lidar_stale;
    speed_margin_cm_ = 0.0f;
    lane_speed_x10_ = lane_target_speed_x10;

    if (!std::isfinite(current_speed_kmh)) current_speed_kmh = 0.0f;

    if (current_speed_kmh >= 4.0f) {
        float diff = current_speed_kmh - 4.0f;
        speed_margin_cm_ = diff * 10.0f;

        if (speed_margin_cm_ > 100.0f) {
            speed_margin_cm_ = 100.0f;
        }
    }

    // Mất LiDAR: không biết có vật cản hay không -> dừng. Ưu tiên cao nhất vì
    // đây là cảm biến an toàn.
    if (lidar_stale) {
        static Throttle log_stale(1000);
        if (log_stale.ready()) {
            std::cerr << "[SAFETY] LiDAR stale -> EMERGENCY_STOP" << std::endl;
        }
        change_state(BypassState::EMERGENCY_STOP);
        return handle_lidar_stale(dev_px);
    }

    // Mất camera: vẫn có LiDAR nên vẫn tránh vật cản được, nhưng không được
    // bánh lái theo số liệu lái cũ -> giữ thẳng và giảm tốc.
    if (camera_stale) {
        static Throttle log_stale(1000);
        if (log_stale.ready()) {
            std::cerr << "[SAFETY] Camera stale -> hold straight, speed=0" << std::endl;
        }
        return handle_camera_stale(lidar);
    }

    if (lidar.ob_front_cm && *lidar.ob_front_cm <= (55.0f )) {
        if (current_state_ != BypassState::EMERGENCY_STOP) {
            static Throttle log_stop(2000);
            if (log_stop.ready()) {
                std::cout << "[OBSTACLE] EMERGENCY! Front <= 55cm (speed="
                          << current_speed_kmh << "km/h)" << std::endl;
            }
            change_state(BypassState::EMERGENCY_STOP);
        }
        return handle_emergency_stop(lidar, dev_px, traffic_light_decision);
    }
    if (traffic_light_decision == "RED" || traffic_light_decision == "STOP") {
        if (current_state_ != BypassState::EMERGENCY_STOP) {
            static Throttle log_red(2000);
            if (log_red.ready() && prev_traffic_light_ != traffic_light_decision) {
                std::cout << "[TRAFFIC] RED/STOP detected! Switching to EMERGENCY_STOP"
                          << std::endl;
            }
            change_state(BypassState::EMERGENCY_STOP);
        }
        prev_traffic_light_ = traffic_light_decision;
        return handle_emergency_stop(lidar, dev_px, traffic_light_decision);
    }

    prev_traffic_light_ = traffic_light_decision;
    switch (current_state_) {
        case BypassState::NORMAL:
            return handle_normal(lidar, dev_px);
        case BypassState::SLOW_DOWN:
            return handle_slow_down(lidar, dominant_slope, dev_px);
        case BypassState::DETECT_BYPASS_SIDE:
            return handle_detect_bypass_side(lidar);
        case BypassState::SWERVE_LEFT:
            return handle_swerve_left(lidar);
        case BypassState::SWERVE_RIGHT:
            return handle_swerve_right(lidar);
        case BypassState::BYPASS_LEFT:
            return handle_bypass_left(lidar);
        case BypassState::BYPASS_RIGHT:
            return handle_bypass_right(lidar);
        case BypassState::RETURN_LANE_LEFT:
            return handle_return_lane_left(lidar, is_dual_lane);
        case BypassState::RETURN_LANE_RIGHT:
            return handle_return_lane_right(lidar, is_dual_lane);
        case BypassState::EMERGENCY_STOP:
            return handle_emergency_stop(lidar, dev_px, traffic_light_decision);
        default:
            change_state(BypassState::NORMAL);
            return handle_normal(lidar, dev_px);
    }
}

// ============================================================================
// CHẾ ĐỘ AN TOÀN KHI MẤT CẢM BIẾN
// ============================================================================

BypassCommand ObstacleAvoidance::handle_lidar_stale(int16_t dev_px) {
    BypassCommand cmd;
    cmd.state = BypassState::EMERGENCY_STOP;
    cmd.state_name = "EMERGENCY_STOP";
    cmd.speed_control = SPEED_HOLD_X10;
    cmd.dev_final_px = dev_px;
    cmd.emergency_stop = true;
    cmd.lidar_stale = true;
    cmd.camera_stale = camera_stale_;
    return cmd;
}

BypassCommand ObstacleAvoidance::handle_camera_stale(const LidarStatus& lidar) {
    BypassCommand cmd;
    cmd.camera_stale = true;

    // Có vật cản phía trước thì vẫn phải né; nếu trống thì dừng chờ camera.
    const bool obstacle_close =
        lidar.ob_front_cm && *lidar.ob_front_cm <= (60.0f + speed_margin_cm_);

    if (obstacle_close) {
        cmd.state = BypassState::SLOW_DOWN;
        cmd.state_name = "SLOW_DOWN";
        cmd.speed_control = SPEED_BYPASS_X10;
        cmd.dev_final_px = 0;
        cmd.emergency_stop = false;
        return cmd;
    }

    cmd.state = current_state_;
    cmd.state_name = current_state_ == BypassState::NORMAL
        ? "NORMAL" : current_state_ == BypassState::EMERGENCY_STOP
            ? "EMERGENCY_STOP" : "CAMERA_STALE";
    cmd.speed_control = SPEED_HOLD_X10;
    cmd.dev_final_px = 0;
    cmd.emergency_stop = current_state_ == BypassState::EMERGENCY_STOP;
    return cmd;
}

// Tốc độ NORMAL do detector lập. Ba nhánh:
//   - có lane + tốc độ hợp lệ -> dùng của detector (đã có sẵn EMA + state
//     machine STRAIGHT/CURVE/SHARP bên trong CameraLane).
//   - detector mất lane -> SPEED_NO_LANE_X10, lái thẳng, không dừng.
//   - detector trả về thứ lạ (0 khi có lane, hoặc > trần) -> kẹp về trần.
uint8_t ObstacleAvoidance::normal_speed_x10() const {
    if (g_speed_normal_override_x10 > 0) return g_speed_normal_override_x10;
    if (lane_speed_x10_ == 0) return SPEED_NO_LANE_X10;
    return std::min<uint8_t>(lane_speed_x10_, SPEED_NORMAL_MAX_X10);
}

BypassCommand ObstacleAvoidance::handle_normal(const LidarStatus& lidar, int16_t dev_px) {
    BypassCommand cmd;
    cmd.state = BypassState::NORMAL;
    cmd.state_name = "NORMAL";
    cmd.speed_control = normal_speed_x10();
    cmd.dev_final_px = dev_px;
    cmd.emergency_stop = false;

    if (lidar.right_min_cm && lidar.left_min_cm) {
        float right_dist = *lidar.right_min_cm;
        float left_dist = *lidar.left_min_cm;
        
        if (right_dist >= 25.0f && left_dist >= 25.0f) {
            cmd.dev_final_px = dev_px;
        }
        else if (right_dist < 20.0f && left_dist > 30.0f) {
            cmd.dev_final_px = static_cast<int16_t>(dev_px - 30);
        }
        else if (left_dist < 20.0f && right_dist > 30.0f) {
            cmd.dev_final_px = static_cast<int16_t>(dev_px + 30);
        }
    }

    if (lidar.ob_front_cm && *lidar.ob_front_cm <= (60.0f + speed_margin_cm_)) {
        std::cout << "[OBSTACLE] Front obstacle detected: "
                  << *lidar.ob_front_cm << "cm (threshold=" << (160.0f + speed_margin_cm_) 
                  << "), switching to SLOW_DOWN" << std::endl;
        change_state(BypassState::SLOW_DOWN);
        return handle_slow_down(lidar, 0.0f, dev_px);
    }

    return cmd;
}

BypassCommand ObstacleAvoidance::handle_slow_down(const LidarStatus& lidar,
                                                  float dominant_slope,
                                                  int16_t dev_px) {
    BypassCommand cmd;
    cmd.state = BypassState::SLOW_DOWN;
    cmd.state_name = "SLOW_DOWN";
    cmd.speed_control = SPEED_BYPASS_X10;
    cmd.dev_final_px = dev_px;
    cmd.emergency_stop = false;

    if (!lidar.ob_front_cm || *lidar.ob_front_cm > (170.0f + speed_margin_cm_)) {
        std::cout << "[OBSTACLE] Obstacle cleared, back to NORMAL" << std::endl;
        change_state(BypassState::NORMAL);
        return handle_normal(lidar, dev_px);
    }

    if (std::abs(dominant_slope) > 0.85f) {
        std::cout << "[OBSTACLE] Sharp turn detected, back to NORMAL" << std::endl;
        change_state(BypassState::NORMAL);
        return handle_normal(lidar, dev_px);
    }

    if (lidar.ob_front_cm && *lidar.ob_front_cm <= (140.0f + speed_margin_cm_)) {
        std::cout << "[OBSTACLE] Obstacle too close, switching to DETECT_BYPASS_SIDE" << std::endl;
        change_state(BypassState::DETECT_BYPASS_SIDE);
        return handle_detect_bypass_side(lidar);
    }

    return cmd;
}

BypassCommand ObstacleAvoidance::handle_detect_bypass_side(const LidarStatus& lidar) {
    BypassCommand cmd;
    cmd.state = BypassState::DETECT_BYPASS_SIDE;
    cmd.state_name = "DETECT_BYPASS_SIDE";
    cmd.speed_control = SPEED_HOLD_X10;
    cmd.dev_final_px = 0;
    cmd.emergency_stop = false;

    if (get_state_elapsed_ms() < 300) {
        return cmd;
    }
    bool left_clear = lidar.left_min_cm && *lidar.left_min_cm > 60.0f;
    bool right_clear = lidar.right_min_cm && *lidar.right_min_cm > 60.0f;

    if (left_clear) {
        std::cout << "[OBSTACLE] Left side clear (" << *lidar.left_min_cm
                  << "cm), switching to SWERVE_LEFT" << std::endl;
        change_state(BypassState::SWERVE_LEFT);
        return handle_swerve_left(lidar);
    }

    if (right_clear) {
        std::cout << "[OBSTACLE] Right side clear (" << *lidar.right_min_cm
                  << "cm), switching to SWERVE_RIGHT" << std::endl;
        change_state(BypassState::SWERVE_RIGHT);
        return handle_swerve_right(lidar);
    }

    std::cout << "[OBSTACLE] Both sides blocked, switching to EMERGENCY_STOP" << std::endl;
    change_state(BypassState::EMERGENCY_STOP);
    return handle_emergency_stop(lidar, 0, "NONE");
}


BypassCommand ObstacleAvoidance::handle_swerve_left(const LidarStatus& lidar) {
    BypassCommand cmd;
    cmd.state = BypassState::SWERVE_LEFT;
    cmd.state_name = "SWERVE_LEFT";
    cmd.speed_control = SPEED_SWERVE_X10;
    cmd.dev_final_px = -60;
    cmd.emergency_stop = false;

    if (!swerve_start_front_dist_ && lidar.ob_front_cm) {
        swerve_start_front_dist_ = *lidar.ob_front_cm;
        std::cout << "[SWERVE_LEFT] Start front distance: "
                  << *swerve_start_front_dist_ << "cm" << std::endl;
    }

    if (get_state_elapsed_ms() < 300) {
        return cmd;
    }

    bool front_clear = false;
    if (lidar.ob_front_cm && swerve_start_front_dist_) {
        front_clear = (*lidar.ob_front_cm > *swerve_start_front_dist_ + 5.0f);
        static Throttle log_d(1000);
        if (log_d.ready()) {
            std::cout << "[SWERVE_LEFT] Front: " << *lidar.ob_front_cm
                      << "cm (start: " << *swerve_start_front_dist_ << "cm)" << std::endl;
        }
    }

    if (front_clear /* && right_safe */) {
        std::cout << "[OBSTACLE] Swerve complete, switching to BYPASS_LEFT" << std::endl;
        change_state(BypassState::BYPASS_LEFT);
        return handle_bypass_left(lidar);
    }

    return cmd;
}

BypassCommand ObstacleAvoidance::handle_swerve_right(const LidarStatus& lidar) {
    BypassCommand cmd;
    cmd.state = BypassState::SWERVE_RIGHT;
    cmd.state_name = "SWERVE_RIGHT";
    cmd.speed_control = SPEED_SWERVE_X10;
    cmd.dev_final_px = 60;
    cmd.emergency_stop = false;

    if (!swerve_start_front_dist_ && lidar.ob_front_cm) {
        swerve_start_front_dist_ = *lidar.ob_front_cm;
        std::cout << "[SWERVE_RIGHT] Start front distance: "
                  << *swerve_start_front_dist_ << "cm" << std::endl;
    }

    if (get_state_elapsed_ms() < 300) {
        return cmd;
    }

    bool front_clear = false;
    if (lidar.ob_front_cm && swerve_start_front_dist_) {
        front_clear = (*lidar.ob_front_cm > *swerve_start_front_dist_ + 5.0f);
        static Throttle log_d(1000);
        if (log_d.ready()) {
            std::cout << "[SWERVE_RIGHT] Front: " << *lidar.ob_front_cm
                      << "cm (start: " << *swerve_start_front_dist_ << "cm)" << std::endl;
        }
    }


    if (front_clear /* && left_safe */) {
        std::cout << "[OBSTACLE] Swerve complete, switching to BYPASS_RIGHT" << std::endl;
        change_state(BypassState::BYPASS_RIGHT);
        return handle_bypass_right(lidar);
    }

    return cmd;
}

BypassCommand ObstacleAvoidance::handle_bypass_left(const LidarStatus& lidar) {
    BypassCommand cmd;
    cmd.state = BypassState::BYPASS_LEFT;
    cmd.state_name = "BYPASS_LEFT";
    cmd.speed_control = SPEED_BYPASS_X10;
    cmd.dev_final_px = 35;
    cmd.emergency_stop = false;

    // Wall-following (theo dõi vật cản bên phải)
    if (lidar.ob_right_cm) {
        float current_dist = *lidar.ob_right_cm;
        static Throttle log_wall(1000);
        // Target: 30cm ± 5cm
        if (current_dist > 30.0f) {
            cmd.dev_final_px = + 25;
            if (log_wall.ready()) {
                std::cout << "[BYPASS_LEFT] Too far (" << current_dist
                          << "cm), steer right" << std::endl;
            }
        }
        if (current_dist < 20.0f) {
            cmd.dev_final_px = - 25;
            if (log_wall.ready()) {
                std::cout << "[BYPASS_LEFT] Too close (" << current_dist
                          << "cm), steer left" << std::endl;
            }
        }
        if (current_dist >= 20.0f && current_dist <= 30.0f) {
            cmd.dev_final_px = 0;
        }
        if (prev_bypass_left_dist_) {
            float delta = current_dist - *prev_bypass_left_dist_;
            if (delta > 20.0f) {
                std::cout << "[BYPASS_LEFT] Obstacle disappeared (delta="
                          << delta << "cm)" << std::endl;
                cmd.dev_final_px = 0;
                prev_bypass_left_dist_ = current_dist;
                return cmd;
            }
        }
        prev_bypass_left_dist_ = current_dist;
    }

    if (get_state_elapsed_ms() > 500) {
        bool rear_clear = lidar.ob_right_rear_cm && *lidar.ob_right_rear_cm > 30.0f;
        bool right_clear = false;

        if (lidar.ob_right_cm && swerve_start_front_dist_) {
            right_clear = (*lidar.ob_right_cm > *swerve_start_front_dist_ + 5.0f);
        }

        if (rear_clear && right_clear) {
            std::cout << "[BYPASS_LEFT] Obstacle passed (rear=" << *lidar.ob_right_rear_cm
                      << "cm, right=" << *lidar.ob_right_cm
                      << "cm), switching to RETURN_LANE_RIGHT" << std::endl;
            change_state(BypassState::RETURN_LANE_RIGHT);
            return handle_return_lane_right(lidar, false);
        }

        static Throttle log_wait(1000);
        if (log_wait.ready()) {
            std::cout << "[BYPASS_LEFT] Waiting... rear_clear=" << rear_clear
                      << ", right_clear=" << right_clear << std::endl;
        }
    }

    return cmd;
}

BypassCommand ObstacleAvoidance::handle_bypass_right(const LidarStatus& lidar) {
    BypassCommand cmd;
    cmd.state = BypassState::BYPASS_RIGHT;
    cmd.state_name = "BYPASS_RIGHT";
    cmd.speed_control = SPEED_BYPASS_X10;
    cmd.dev_final_px = -35;
    cmd.emergency_stop = false;

    if (lidar.ob_left_cm) {
        float current_dist = *lidar.ob_left_cm;
        static Throttle log_wall(1000);
        if (current_dist > 30.0f) {
            cmd.dev_final_px = - 25;
            if (log_wall.ready()) {
                std::cout << "[BYPASS_RIGHT] Too far (" << current_dist
                          << "cm), steer left" << std::endl;
            }
        }
        if (current_dist < 20.0f) {
            cmd.dev_final_px = + 25;
            if (log_wall.ready()) {
                std::cout << "[BYPASS_RIGHT] Too close (" << current_dist
                          << "cm), steer right" << std::endl;
            }
        }
        if (current_dist >= 20.0f && current_dist <= 30.0f) {
            cmd.dev_final_px = 0;
        }
        if (prev_bypass_right_dist_) {
            float delta = current_dist - *prev_bypass_right_dist_;
            if (delta > 20.0f) {
                std::cout << "[BYPASS_RIGHT] Obstacle disappeared (delta="
                          << delta << "cm)" << std::endl;
                cmd.dev_final_px = 0;
                prev_bypass_right_dist_ = current_dist;
                return cmd;
            }
        }
        prev_bypass_right_dist_ = current_dist;
    }

    if (get_state_elapsed_ms() > 500) {
        bool rear_clear = lidar.ob_left_rear_cm && *lidar.ob_left_rear_cm > 30.0f;
        bool left_clear = false;

        if (lidar.ob_left_cm && swerve_start_front_dist_) {
            left_clear = (*lidar.ob_left_cm > *swerve_start_front_dist_ + 5.0f);
        }

        if (rear_clear && left_clear) {
            std::cout << "[BYPASS_RIGHT] Obstacle passed (rear=" << *lidar.ob_left_rear_cm
                      << "cm, left=" << *lidar.ob_left_cm
                      << "cm), switching to RETURN_LANE_LEFT" << std::endl;
            change_state(BypassState::RETURN_LANE_LEFT);
            return handle_return_lane_left(lidar, false);
        }

        static Throttle log_wait(1000);
        if (log_wait.ready()) {
            std::cout << "[BYPASS_RIGHT] Waiting... rear_clear=" << rear_clear
                      << ", left_clear=" << left_clear << std::endl;
        }
    }

    return cmd;
}

BypassCommand ObstacleAvoidance::handle_return_lane_left(const LidarStatus& lidar,
                                                         bool is_dual_lane) {
    BypassCommand cmd;
    cmd.state = BypassState::RETURN_LANE_LEFT;
    cmd.state_name = "RETURN_LANE_LEFT";
    cmd.speed_control = SPEED_RETURN_X10;
    cmd.dev_final_px = -80;
    cmd.emergency_stop = false;

    if (get_state_elapsed_ms() < 800) {
        return cmd;
    }

    if (is_dual_lane) {
        std::cout << "[OBSTACLE] Both lanes detected, back to NORMAL" << std::endl;
        change_state(BypassState::NORMAL);
        return handle_normal(lidar, 0);
    }

    if (lidar.ob_front_cm && *lidar.ob_front_cm < (130.0f + speed_margin_cm_)) {
        std::cout << "[OBSTACLE] Obstacle detected during return, back to DETECT_BYPASS_SIDE" << std::endl;
        change_state(BypassState::DETECT_BYPASS_SIDE);
        return handle_detect_bypass_side(lidar);
    }

    return cmd;
}

BypassCommand ObstacleAvoidance::handle_return_lane_right(const LidarStatus& lidar,
                                                          bool is_dual_lane) {
    BypassCommand cmd;
    cmd.state = BypassState::RETURN_LANE_RIGHT;
    cmd.state_name = "RETURN_LANE_RIGHT";
    cmd.speed_control = SPEED_RETURN_X10;
    cmd.dev_final_px = 80;
    cmd.emergency_stop = false;

    if (get_state_elapsed_ms() < 800) {
        return cmd;
    }

    if (is_dual_lane) {
        std::cout << "[OBSTACLE] Both lanes detected, back to NORMAL" << std::endl;
        change_state(BypassState::NORMAL);
        return handle_normal(lidar, 0);
    }

    if (lidar.ob_front_cm && *lidar.ob_front_cm < (130.0f + speed_margin_cm_)) {
        std::cout << "[OBSTACLE] Obstacle detected during return, back to DETECT_BYPASS_SIDE" << std::endl;
        change_state(BypassState::DETECT_BYPASS_SIDE);
        return handle_detect_bypass_side(lidar);
    }

    return cmd;
}

BypassCommand ObstacleAvoidance::handle_emergency_stop(const LidarStatus& lidar, 
                                                       int16_t dev_px,
                                                       const std::string& traffic_light_decision) {
    BypassCommand cmd;
    cmd.state = BypassState::EMERGENCY_STOP;
    cmd.state_name = "EMERGENCY_STOP";
    cmd.speed_control = SPEED_HOLD_X10;
    cmd.dev_final_px = dev_px;
    cmd.emergency_stop = true;

    if (traffic_light_decision == "GREEN") {
        if (!lidar.ob_front_cm || *lidar.ob_front_cm > (160.0f + speed_margin_cm_)) {
            if (prev_traffic_light_ != "GREEN") {
                std::cout << "[TRAFFIC] GREEN light! Returning to NORMAL" << std::endl;
            }
            prev_traffic_light_ = traffic_light_decision;
            change_state(BypassState::NORMAL);
            return handle_normal(lidar, dev_px);
        } else {
            static Throttle log_green(1000);
            if (log_green.ready()) {
                std::cout << "[TRAFFIC] GREEN but obstacle at "
                          << *lidar.ob_front_cm << "cm" << std::endl;
            }
        }
    }
    
    if (traffic_light_decision != "RED" && traffic_light_decision != "STOP") {
        if (!lidar.ob_front_cm || *lidar.ob_front_cm > (160.0f + speed_margin_cm_)) {
            std::cout << "[OBSTACLE] No RED/STOP and obstacle cleared, back to NORMAL" << std::endl;
            change_state(BypassState::NORMAL);
            return handle_normal(lidar, dev_px);
        }
    }

    prev_traffic_light_ = traffic_light_decision;
    return cmd;
}