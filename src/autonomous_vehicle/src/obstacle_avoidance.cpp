// obstacle_avoidance.cpp
#include "obstacle_avoidance.hpp"
#include "lidar_module.hpp"
#include <iostream>
#include <chrono>
#include <cmath>

ObstacleAvoidance::ObstacleAvoidance()
    : current_state_(BypassState::NORMAL),
      state_start_time_(now_ms()),
      prev_bypass_left_dist_(std::nullopt),
      prev_bypass_right_dist_(std::nullopt),
      swerve_start_front_dist_(std::nullopt),
      swerve_start_left_dist_(std::nullopt),
      swerve_start_right_dist_(std::nullopt),
      prev_traffic_light_("NONE"),
      current_speed_kmh_(0.0f),
      speed_margin_cm_(0.0f) {}

// Đồng hồ phải là steady: system_clock có thể nhảy (NTP) làm timeout sai và
// biến một khoảng chờ an toàn thành tức thời.
unsigned long ObstacleAvoidance::now_ms() {
    return static_cast<unsigned long>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
}

void ObstacleAvoidance::change_state(BypassState new_state) {
    if (current_state_ == new_state) return;

    current_state_ = new_state;
    state_start_time_ = now_ms();

    switch (new_state) {
        case BypassState::NORMAL:
            prev_bypass_left_dist_.reset();
            prev_bypass_right_dist_.reset();
            swerve_start_front_dist_.reset();
            swerve_start_left_dist_.reset();
            swerve_start_right_dist_.reset();
            break;

        case BypassState::SWERVE_LEFT:
        case BypassState::SWERVE_RIGHT:
            // Chụp lại khoảng cách lúc bắt đầu lách, làm mốc so sánh.
            swerve_start_front_dist_.reset();
            swerve_start_left_dist_.reset();
            swerve_start_right_dist_.reset();
            break;

        case BypassState::BYPASS_LEFT:
            prev_bypass_left_dist_.reset();
            break;

        case BypassState::BYPASS_RIGHT:
            prev_bypass_right_dist_.reset();
            break;

        default:
            break;
    }
}

unsigned long ObstacleAvoidance::get_state_elapsed_ms() const {
    return now_ms() - state_start_time_;
}

BypassCommand ObstacleAvoidance::update(const LidarStatus& lidar,
                                        int16_t dev_px,
                                        float dominant_slope,
                                        bool is_dual_lane,
                                        const std::string& traffic_light_decision,
                                        float current_speed_kmh) {
    current_speed_kmh_ = current_speed_kmh;
    dev_px_ = dev_px;
    dominant_slope_ = dominant_slope;
    speed_margin_cm_ = 0.0f;
    
    if (current_speed_kmh >= 4.0f) {
        float diff = current_speed_kmh - 4.0f;
        speed_margin_cm_ = diff * 10.0f;
        
        if (speed_margin_cm_ > 100.0f) {
            speed_margin_cm_ = 100.0f;
        }
    }
    
    // Ngưỡng phanh khẩn cấp KHÔNG dùng speed_margin: càng nhanh càng phải dừng sớm.
    constexpr float emergency_front_cm = 55.0f;
    if (lidar.ob_front_cm && *lidar.ob_front_cm <= emergency_front_cm) {
        if (current_state_ != BypassState::EMERGENCY_STOP) {
            change_state(BypassState::EMERGENCY_STOP);
        }
        return handle_emergency_stop(lidar, dev_px, traffic_light_decision);
    }
    if (traffic_light_decision == "RED" || traffic_light_decision == "STOP") {
        change_state(BypassState::EMERGENCY_STOP);
        prev_traffic_light_ = traffic_light_decision;
        return handle_emergency_stop(lidar, dev_px, traffic_light_decision);
    }

    prev_traffic_light_ = traffic_light_decision;
    const uint64_t t = now_ms();

    switch (current_state_) {
        case BypassState::NORMAL:
            return handle_normal(lidar, dev_px, dominant_slope);
        case BypassState::SLOW_DOWN:
            return handle_slow_down(lidar, dominant_slope, dev_px, t);
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
            return handle_normal(lidar, dev_px, dominant_slope);
    }
}
BypassCommand ObstacleAvoidance::handle_normal(const LidarStatus& lidar,
                                               int16_t dev_px,
                                               float dominant_slope) {
    BypassCommand cmd;
    cmd.state = BypassState::NORMAL;
    cmd.state_name = "NORMAL";
    // Trần cao: ở trạng thái NORMAL, tốc độ do bộ lập tốc độ của camera quyết
    // định. Đặt một trần thấp ở đây khiến min() trong fusion_viz_node luôn lấy
    // giá trị của né tránh, tức camera tính 8.5km/h vẫn chạy 3.5km/h.
    cmd.speed_control = SPEED_CEILING_NORMAL;
    cmd.dev_final_px = dev_px;
    cmd.emergency_stop = false;

    if (lidar.right_min_cm && lidar.left_min_cm) {
        const float right_dist = *lidar.right_min_cm;
        const float left_dist = *lidar.left_min_cm;

        if (right_dist >= 25.0f && left_dist >= 25.0f) {
            cmd.dev_final_px = dev_px;
        } else if (right_dist < 20.0f && left_dist > 30.0f) {
            cmd.dev_final_px = static_cast<int16_t>(dev_px - 30);
        } else if (left_dist < 20.0f && right_dist > 30.0f) {
            cmd.dev_final_px = static_cast<int16_t>(dev_px + 30);
        }
    }

    const float slow_down_enter_cm = 160.0f + speed_margin_cm_;
    if (lidar.ob_front_cm && *lidar.ob_front_cm <= slow_down_enter_cm) {
        change_state(BypassState::SLOW_DOWN);
        return handle_slow_down(lidar, dominant_slope, dev_px, now_ms());
    }

    return cmd;
}

BypassCommand ObstacleAvoidance::handle_slow_down(const LidarStatus& lidar,
                                                  float dominant_slope,
                                                  int16_t dev_px,
                                                  uint64_t now_ms_in) {
    // now_ms_in cho phép sau này giới hạn thời gian ở trạng thái giảm tốc mà
    // không phải sửa chữ ký ở nơi gọi. Hiện tại logic dựa trên khoảng cách
    // nên không dùng tới.
    (void)now_ms_in;

    BypassCommand cmd;
    cmd.state = BypassState::SLOW_DOWN;
    cmd.state_name = "SLOW_DOWN";
    cmd.speed_control = SPEED_SLOW_DOWN;
    cmd.dev_final_px = dev_px;
    cmd.emergency_stop = false;

    // Phân biệt 2 trường hợp rất khác nhau:
    //   has_data == false        -> LiDAR không trả về điểm nào (sensor
    //                               chết / mất nguồn). KHÔNG coi là trống.
    //   has_data == true và
    //   ob_front_cm == nullopt   -> vùng phía trước TRỐNG.
    // Gộp hai case lại bằng `!lidar.ob_front_cm` sẽ khiến SLOW_DOWN kẹt vĩnh
    // viễn khi vật cản rời khỏi tầm, xe đứng yên không chạy lại được.
    if (!lidar.has_data) {
        cmd.speed_control = 0;
        cmd.dev_final_px = 0;
        cmd.emergency_stop = true;
        return cmd;
    }

    if (!lidar.ob_front_cm || *lidar.ob_front_cm > (170.0f + speed_margin_cm_)) {
        change_state(BypassState::NORMAL);
        return handle_normal(lidar, dev_px, dominant_slope);
    }

    // THỨ TỰ QUAN TRỌNG: kiểm tra khoảng cách vật cản TRƯỚC, sau đó mới đến
    // độ cong. Nhánh "đường cong" đứng trước và return sớm sẽ khiến
    // SLOW_DOWN kẹt vĩnh viễn khi xe đang vào cua gắt + có vật cản phía trước.
    // An toàn chống vật cản phải thắng mọi tối ưu tốc độ theo độ cong.
    if (*lidar.ob_front_cm <= (140.0f + speed_margin_cm_)) {
        change_state(BypassState::DETECT_BYPASS_SIDE);
        return handle_detect_bypass_side(lidar);
    }

    // Vẫn trong vùng 140..170: đường cong gắt thì giảm tốc thêm.
    if (std::abs(dominant_slope) > 0.85f) {
        cmd.speed_control = SPEED_SLOW_SHARP;
    }

    return cmd;
}

BypassCommand ObstacleAvoidance::handle_detect_bypass_side(const LidarStatus& lidar) {
    BypassCommand cmd;
    cmd.state = BypassState::DETECT_BYPASS_SIDE;
    cmd.state_name = "DETECT_BYPASS_SIDE";
    cmd.speed_control = 0;
    cmd.dev_final_px = 0;
    cmd.emergency_stop = false;

    if (get_state_elapsed_ms() < STOP_SETTLE_MS) {
        return cmd;
    }

    // Cần cả hai vùng bên mới quyết định được; thiếu dữ liệu thì chờ,
    // không được coi là "trống" rồi lao vào một bên.
    // `nullopt` = vùng không có vật cản = TRỐNG, không phải mất dữ liệu.
    // Chỉ khi cả scan không có điểm nào (has_data == false) mới coi là cảm
    // biến chết. Gate sai ở đây sẽ kiểm tra được: xe báo "hết dữ liệu" mà
    // trước mất xuất hiệm khi khung đường sạch -> kẹt DETECT_BYPASS_SIDE.
    if (!lidar.has_data) {
        cmd.emergency_stop = true;
        return cmd;
    }

    constexpr float NO_OBSTACLE_CM = 100000.0f;
    const float left_cm  = lidar.left_min_cm  ? *lidar.left_min_cm  : NO_OBSTACLE_CM;
    const float right_cm = lidar.right_min_cm ? *lidar.right_min_cm : NO_OBSTACLE_CM;
    const bool left_clear  = left_cm  > MIN_BYPASS_SPACE_CM;
    const bool right_clear = right_cm > MIN_BYPASS_SPACE_CM;

    // Ưu tiên bên rộng hơn để giữ an toàn.
    if (left_clear && right_clear) {
        const bool prefer_left = left_cm >= right_cm;
        change_state(prefer_left ? BypassState::SWERVE_LEFT : BypassState::SWERVE_RIGHT);
        return prefer_left ? handle_swerve_left(lidar) : handle_swerve_right(lidar);
    }

    if (left_clear) {
        change_state(BypassState::SWERVE_LEFT);
        return handle_swerve_left(lidar);
    }

    if (right_clear) {
        change_state(BypassState::SWERVE_RIGHT);
        return handle_swerve_right(lidar);
    }

    // Cả hai đều kín: dừng hẳn.
    cmd.emergency_stop = true;
    return cmd;
}


BypassCommand ObstacleAvoidance::handle_swerve_left(const LidarStatus& lidar) {
    BypassCommand cmd;
    cmd.state = BypassState::SWERVE_LEFT;
    cmd.state_name = "SWERVE_LEFT";
    cmd.speed_control = SPEED_SWERVE;
    cmd.dev_final_px = -60;
    cmd.emergency_stop = false;

    if (!swerve_start_front_dist_ && lidar.ob_front_cm) {
        swerve_start_front_dist_ = *lidar.ob_front_cm;
    }

    // Chụp lại khoảng cách vật cản BÊN phải (vật cản ta đang né sẽ nằm bên
    // phải sau khi lái trái). Dùng để biết đã đi qua xong hay chưa.
    if (!swerve_start_right_dist_ && lidar.ob_right_cm) {
        swerve_start_right_dist_ = *lidar.ob_right_cm;
    }

    if (get_state_elapsed_ms() < STOP_SETTLE_MS) {
        return cmd;
    }

    // ob_front_cm == nullopt (scan còn sống, vùng trước trống) CŨNG là đã
    // thoát. Nếu chỉ chờ "có điểm" thì vật cản rời khỏi vùng trước sẽ không
    // bao giờ được coi là clear, và SWERVE chỉ kết bằng timeout.
    const bool front_clear =
        swerve_start_front_dist_ &&
        (!lidar.ob_front_cm || *lidar.ob_front_cm > *swerve_start_front_dist_ + 5.0f);

    if (front_clear) {
        change_state(BypassState::BYPASS_LEFT);
        return handle_bypass_left(lidar);
    }

    // Timeout an toàn: không giữ góc lái cứng -60 mãi. Dừng hẳn.
    if (get_state_elapsed_ms() > SWERVE_TIMEOUT_MS) {
        cmd.speed_control = 0;
        cmd.dev_final_px = 0;
        cmd.emergency_stop = true;
    }

    return cmd;
}

BypassCommand ObstacleAvoidance::handle_swerve_right(const LidarStatus& lidar) {
    BypassCommand cmd;
    cmd.state = BypassState::SWERVE_RIGHT;
    cmd.state_name = "SWERVE_RIGHT";
    cmd.speed_control = SPEED_SWERVE;
    cmd.dev_final_px = 60;
    cmd.emergency_stop = false;

    if (!swerve_start_front_dist_ && lidar.ob_front_cm) {
        swerve_start_front_dist_ = *lidar.ob_front_cm;
    }

    if (!swerve_start_left_dist_ && lidar.ob_left_cm) {
        swerve_start_left_dist_ = *lidar.ob_left_cm;
    }

    if (get_state_elapsed_ms() < STOP_SETTLE_MS) {
        return cmd;
    }

    // ob_front_cm == nullopt (scan còn sống, vùng trước trống) CŨNG là đã
    // thoát. Nếu chỉ chờ "có điểm" thì vật cản rời khỏi vùng trước sẽ không
    // bao giờ được coi là clear, và SWERVE chỉ kết bằng timeout.
    const bool front_clear =
        swerve_start_front_dist_ &&
        (!lidar.ob_front_cm || *lidar.ob_front_cm > *swerve_start_front_dist_ + 5.0f);

    if (front_clear) {
        change_state(BypassState::BYPASS_RIGHT);
        return handle_bypass_right(lidar);
    }

    if (get_state_elapsed_ms() > SWERVE_TIMEOUT_MS) {
        cmd.speed_control = 0;
        cmd.dev_final_px = 0;
        cmd.emergency_stop = true;
    }

    return cmd;
}

BypassCommand ObstacleAvoidance::handle_bypass_left(const LidarStatus& lidar) {
    BypassCommand cmd;
    cmd.state = BypassState::BYPASS_LEFT;
    cmd.state_name = "BYPASS_LEFT";
    cmd.speed_control = SPEED_BYPASS;
    cmd.dev_final_px = 35;
    cmd.emergency_stop = false;

    // Bám tường theo khoảng cách vật cản bên phải: giữ trong 20..30cm.
    // Vật cản xa bên phải -> lái phải (tiến về vật cản); vật cản sát bên
    // phải -> lái trái (xa vật cản).
    if (lidar.ob_right_cm) {
        const float current_dist = *lidar.ob_right_cm;

        if (current_dist > 30.0f) {
            cmd.dev_final_px = 25;
        } else if (current_dist < 20.0f) {
            cmd.dev_final_px = -25;
        } else {
            cmd.dev_final_px = 0;
        }

        if (prev_bypass_left_dist_) {
            const float delta = current_dist - *prev_bypass_left_dist_;
            if (delta > 20.0f) {
                // Vật cản biến mất đột ngột -> đã thoát khỏi bóng vật cản.
                cmd.dev_final_px = 0;
                prev_bypass_left_dist_ = current_dist;
                return cmd;
            }
        }
        prev_bypass_left_dist_ = current_dist;
    }

    if (get_state_elapsed_ms() > BYPASS_SETTLE_MS) {
        // Hai điều kiện ĐỒNG BỘ (cùng lấy từ vùng phía sau bên phải):
        //  1. Vùng sau-phải đã trống  -> đã lách qua đuôi vật cản
        //  2. Vùng bên phải đã mở rộng so với lúc bắt đầu SWERVE
        // So khoảng cách BÊN với khoảng cách PHÍA TRƯỚC là điều kiện vô
        // nghĩa và khiến bypass không bao giờ hoàn tất.
        const float rear_cm = lidar.ob_right_rear_cm ? *lidar.ob_right_rear_cm : 1e5f;
        const bool rear_clear = rear_cm > BYPASS_REAR_CLEAR_CM;
        const bool right_open = lidar.ob_right_cm && swerve_start_right_dist_ &&
            (*lidar.ob_right_cm > *swerve_start_right_dist_ + BYPASS_SIDE_OPEN_DELTA_CM);

        if (rear_clear && right_open) {
            change_state(BypassState::RETURN_LANE_RIGHT);
            return handle_return_lane_right(lidar, false);
        }
    }

    // Chặn trên: vật cản trước lại gần -> phải cân nhắc lại phương án né
    if (lidar.ob_front_cm && *lidar.ob_front_cm <= (70.0f + speed_margin_cm_)) {
        change_state(BypassState::DETECT_BYPASS_SIDE);
        return handle_detect_bypass_side(lidar);
    }

    if (get_state_elapsed_ms() > BYPASS_TIMEOUT_MS) {
        cmd.speed_control = 0;
        cmd.dev_final_px = 0;
        cmd.emergency_stop = true;
    }

    return cmd;
}

BypassCommand ObstacleAvoidance::handle_bypass_right(const LidarStatus& lidar) {
    BypassCommand cmd;
    cmd.state = BypassState::BYPASS_RIGHT;
    cmd.state_name = "BYPASS_RIGHT";
    cmd.speed_control = SPEED_BYPASS;
    cmd.dev_final_px = -35;
    cmd.emergency_stop = false;

    if (lidar.ob_left_cm) {
        float current_dist = *lidar.ob_left_cm;
        if (current_dist > 30.0f) {
            cmd.dev_final_px = -25;
        }
        if (current_dist < 20.0f) {
            cmd.dev_final_px = +25;
        }
        if (current_dist >= 20.0f && current_dist <= 30.0f) {
            cmd.dev_final_px = 0;
        }
        if (prev_bypass_right_dist_) {
            float delta = current_dist - *prev_bypass_right_dist_;
            if (delta > 20.0f) {
                // Vật cản đã lùi ra xa: giữ thẳng, chờ mốc "đã qua" bên dưới.
                cmd.dev_final_px = 0;
                prev_bypass_right_dist_ = current_dist;
                return cmd;
            }
        }
        prev_bypass_right_dist_ = current_dist;
    }

    if (get_state_elapsed_ms() > BYPASS_SETTLE_MS) {
        // Giống handle_bypass_left: so sánh cùng một vùng (sau - bên trái)
        // với mốc chụp lúc SWERVE, không dùng khoảng cách phía trước.
        const float rear_cm = lidar.ob_left_rear_cm ? *lidar.ob_left_rear_cm : 100000.0f;
        const bool rear_clear = rear_cm > BYPASS_REAR_CLEAR_CM;
        const bool left_open = lidar.ob_left_cm && swerve_start_left_dist_ &&
            (*lidar.ob_left_cm > *swerve_start_left_dist_ + BYPASS_SIDE_OPEN_DELTA_CM);

        if (rear_clear && left_open) {
            change_state(BypassState::RETURN_LANE_LEFT);
            return handle_return_lane_left(lidar, false);
        }
    }

    if (lidar.ob_front_cm && *lidar.ob_front_cm <= (70.0f + speed_margin_cm_)) {
        change_state(BypassState::DETECT_BYPASS_SIDE);
        return handle_detect_bypass_side(lidar);
    }

    if (get_state_elapsed_ms() > BYPASS_TIMEOUT_MS) {
        cmd.speed_control = 0;
        cmd.dev_final_px = 0;
        cmd.emergency_stop = true;
        return cmd;
    }

    return cmd;
}

BypassCommand ObstacleAvoidance::handle_return_lane_left(const LidarStatus& lidar,
                                                         bool is_dual_lane) {
    BypassCommand cmd;
    cmd.state = BypassState::RETURN_LANE_LEFT;
    cmd.state_name = "RETURN_LANE_LEFT";
    cmd.speed_control = SPEED_RETURN;
    // Vẫn giữ góc lái "về làn" để cắt ngang sau khi đã đi qua vật cản, nhưng
    // có chặn trên: nếu góc lái của camera vẫn lệch mạnh thì cộng dồn hai
    // nguồn có thể khiến xe lách sang làn kế.
    cmd.dev_final_px = clamp_px(static_cast<int32_t>(dev_px_) - RETURN_STEER_PX);
    cmd.emergency_stop = false;

    if (get_state_elapsed_ms() < 800) {
        return cmd;
    }

    // Nhận lại sai số làn thật từ camera, không dùng giá trị 0 cứng.
    if (is_dual_lane) {
        change_state(BypassState::NORMAL);
        return handle_normal(lidar, dev_px_, dominant_slope_);
    }

    if (lidar.ob_front_cm && *lidar.ob_front_cm < (130.0f + speed_margin_cm_)) {
        change_state(BypassState::DETECT_BYPASS_SIDE);
        return handle_detect_bypass_side(lidar);
    }

    if (get_state_elapsed_ms() > RETURN_TIMEOUT_MS) {
        cmd.speed_control = 0;
        cmd.emergency_stop = true;
        return cmd;
    }

    return cmd;
}

BypassCommand ObstacleAvoidance::handle_return_lane_right(const LidarStatus& lidar,
                                                          bool is_dual_lane) {
    BypassCommand cmd;
    cmd.state = BypassState::RETURN_LANE_RIGHT;
    cmd.state_name = "RETURN_LANE_RIGHT";
    cmd.speed_control = SPEED_RETURN;
    cmd.dev_final_px = clamp_px(static_cast<int32_t>(dev_px_) + RETURN_STEER_PX);
    cmd.emergency_stop = false;

    if (get_state_elapsed_ms() < 800) {
        return cmd;
    }

    if (is_dual_lane) {
        change_state(BypassState::NORMAL);
        return handle_normal(lidar, dev_px_, dominant_slope_);
    }

    if (lidar.ob_front_cm && *lidar.ob_front_cm < (130.0f + speed_margin_cm_)) {
        change_state(BypassState::DETECT_BYPASS_SIDE);
        return handle_detect_bypass_side(lidar);
    }

    if (get_state_elapsed_ms() > RETURN_TIMEOUT_MS) {
        cmd.speed_control = 0;
        cmd.emergency_stop = true;
        return cmd;
    }

    return cmd;
}

BypassCommand ObstacleAvoidance::handle_emergency_stop(const LidarStatus& lidar, 
                                                       int16_t dev_px,
                                                       const std::string& traffic_light_decision) {
    BypassCommand cmd;
    cmd.state = BypassState::EMERGENCY_STOP;
    cmd.state_name = "EMERGENCY_STOP";
    cmd.speed_control = 0;
    cmd.dev_final_px = dev_px;
    cmd.emergency_stop = true;

    // Nhả ga khi đèn xanh VÀ vùng phía trước đã trống. Chỉ một trong hai điều
    // kiện là chưa đủ: đèn xanh mà còn vật cản 40cm phía trước thì xe chạy
    // thẳng vào vật cản.
    if (traffic_light_decision == "GREEN") {
        if (!lidar.ob_front_cm || *lidar.ob_front_cm > (160.0f + speed_margin_cm_)) {
            prev_traffic_light_ = traffic_light_decision;
            change_state(BypassState::NORMAL);
            return handle_normal(lidar, dev_px, dominant_slope_);
        }
    }

    if (traffic_light_decision != "RED" && traffic_light_decision != "STOP") {
        if (!lidar.ob_front_cm || *lidar.ob_front_cm > (160.0f + speed_margin_cm_)) {
            change_state(BypassState::NORMAL);
            return handle_normal(lidar, dev_px, dominant_slope_);
        }
    }

    prev_traffic_light_ = traffic_light_decision;
    return cmd;
}