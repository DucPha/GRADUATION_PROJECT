// obstacle_avoidance.cpp
#include "obstacle_avoidance.hpp"
#include "lidar_module.hpp"
#include <iostream>
#include <chrono>
#include <cmath>

ObstacleAvoidance::ObstacleAvoidance()
    : current_state_(BypassState::NORMAL),
      state_start_time_(std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count()),
      prev_bypass_left_dist_(std::nullopt),
      prev_bypass_right_dist_(std::nullopt),
      swerve_start_front_dist_(std::nullopt),
      swerve_start_left_dist_(std::nullopt),
      swerve_start_right_dist_(std::nullopt),
      prev_traffic_light_("NONE"),
      current_speed_kmh_(0.0f),
      speed_margin_cm_(0.0f) {}

void ObstacleAvoidance::change_state(BypassState new_state) {
    if (current_state_ != new_state) {
        current_state_ = new_state;
        state_start_time_ = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        
        if (new_state == BypassState::BYPASS_LEFT) {
            prev_bypass_left_dist_ = std::nullopt;
        } else if (new_state == BypassState::BYPASS_RIGHT) {
            prev_bypass_right_dist_ = std::nullopt;
        } else if (new_state == BypassState::SWERVE_LEFT ||
                   new_state == BypassState::SWERVE_RIGHT) {
            swerve_start_front_dist_ = std::nullopt;
            swerve_start_left_dist_ = std::nullopt;
            swerve_start_right_dist_ = std::nullopt;
        }
    }
}

unsigned long ObstacleAvoidance::get_state_elapsed_ms() const {
    unsigned long now = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    return now - state_start_time_;
}

BypassCommand ObstacleAvoidance::update(const LidarStatus& lidar,
                                        int16_t dev_px,
                                        float dominant_slope,
                                        bool is_dual_lane,
                                        const std::string& traffic_light_decision,
                                        float current_speed_kmh) {
    current_speed_kmh_ = current_speed_kmh;
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
            std::cout << "[OBSTACLE] EMERGENCY! Front <= " << emergency_front_cm
                      << "cm (speed=" << current_speed_kmh << "km/h)" << std::endl;
            change_state(BypassState::EMERGENCY_STOP);
        }
        return handle_emergency_stop(lidar, dev_px, traffic_light_decision);
    }
    if (traffic_light_decision == "RED" || traffic_light_decision == "STOP") {
        if (current_state_ != BypassState::EMERGENCY_STOP) {
            if (prev_traffic_light_ != traffic_light_decision) {
                std::cout << "[TRAFFIC] 🚨 " << traffic_light_decision 
                          << " detected! Switching to EMERGENCY_STOP" << std::endl;
            }
            change_state(BypassState::EMERGENCY_STOP);
        }
        prev_traffic_light_ = traffic_light_decision;
        return handle_emergency_stop(lidar, dev_px, traffic_light_decision);
    }

    prev_traffic_light_ = traffic_light_decision;
    switch (current_state_) {
        case BypassState::NORMAL:
            return handle_normal(lidar, dev_px, dominant_slope);
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
            return handle_normal(lidar, dev_px, dominant_slope);
    }
}
BypassCommand ObstacleAvoidance::handle_normal(const LidarStatus& lidar,
                                               int16_t dev_px,
                                               float dominant_slope) {
    BypassCommand cmd;
    cmd.state = BypassState::NORMAL;
    cmd.state_name = "NORMAL";
    cmd.speed_control = 35;
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

    const float slow_down_enter_cm = 160.0f + speed_margin_cm_;
    if (lidar.ob_front_cm && *lidar.ob_front_cm <= slow_down_enter_cm) {
        std::cout << "[OBSTACLE] Front obstacle detected: "
                  << *lidar.ob_front_cm << "cm (threshold=" << slow_down_enter_cm
                  << "), switching to SLOW_DOWN" << std::endl;
        change_state(BypassState::SLOW_DOWN);
        return handle_slow_down(lidar, dominant_slope, dev_px);
    }

    return cmd;
}

BypassCommand ObstacleAvoidance::handle_slow_down(const LidarStatus& lidar,
                                                  float dominant_slope,
                                                  int16_t dev_px) {
    BypassCommand cmd;
    cmd.state = BypassState::SLOW_DOWN;
    cmd.state_name = "SLOW_DOWN";
    cmd.speed_control = 30;
    cmd.dev_final_px = dev_px;
    cmd.emergency_stop = false;

    // Phan biet 2 truong hop rat khac nhau:
    //   has_data == false        -> LiDAR khong tra ve diem nao (sensor
    //                               chet / mat nguon). KHONG coi la trong.
    //   has_data == true va
    //   ob_front_cm == nullopt  -> vung phia truoc TRONG.
    // Ban cu gop hai case lai bang `!lidar.ob_front_cm` -> khi vat can di
    // khoi tam, ob_front_cm thanh nullopt va SLOW_DOWN ket vinh vien,
    // xe dung yen khong chay lai duoc.
    if (!lidar.has_data) {
        std::cout << "[OBSTACLE] LiDAR returned no valid points, holding STOP"
                  << std::endl;
        cmd.speed_control = 0;
        cmd.dev_final_px = 0;
        cmd.emergency_stop = true;
        return cmd;
    }

    if (!lidar.ob_front_cm || *lidar.ob_front_cm > (170.0f + speed_margin_cm_)) {
        std::cout << "[OBSTACLE] Obstacle cleared, back to NORMAL" << std::endl;
        change_state(BypassState::NORMAL);
        return handle_normal(lidar, dev_px, dominant_slope);
    }

    // THỨ TỰ QUAN TRỌNG: kiểm tra khoảng cách vật cản TRƯỚC, sau đó mới đến
    // độ cong. Trước đây nhánh "đường cong" đứng trước và `return` sớm, nên
    // khi xe đang vào cua gắt + có vật cản phía trước thì SLOW_DOWN kẹt vĩnh
    // viễn và KHÔNG BAO GIỜ chuyển sang DETECT_BYPASS_SIDE.
    // An toàn chống vật cản phải thắng mọi tối ưu tốc độ theo độ cong.
    if (*lidar.ob_front_cm <= (140.0f + speed_margin_cm_)) {
        std::cout << "[OBSTACLE] Obstacle too close, switching to DETECT_BYPASS_SIDE" << std::endl;
        change_state(BypassState::DETECT_BYPASS_SIDE);
        return handle_detect_bypass_side(lidar);
    }

    // Vẫn trong vùng 140..170: đường cong gắt thì giảm tốc thêm.
    // (Bản cũ trả về NORMAL ở nhánh này -> xe chạy thẳng vào vật cản.)
    if (std::abs(dominant_slope) > 0.85f) {
        cmd.speed_control = 25; // 2.5 km/h: cua gắt + vật cản = đi rất chậm
        return cmd;
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

    if (get_state_elapsed_ms() < 300) {
        return cmd;
    }

    // Cần cả hai vùng bên mới quyết định được; thiếu dữ liệu thì chờ,
    // không được coi là "trống" rồi lao vào một bên.
    // `nullopt` = vung do KHONG co vat can = TRONG, khong phai mat du lieu.
    // Chi khi ca scan khong co diem nao (has_data == false) moi coi la sensor
    // chet. Gate sai o day se kiem tra duoc: xe bao "het du lieu" ma truoc
    // mat xuat hiem khi khung duong sach -> kiet DETECT_BYPASS_SIDE vinh vien.
    if (!lidar.has_data) {
        std::cout << "[OBSTACLE] LiDAR returned no valid points, holding STOP"
                  << std::endl;
        cmd.emergency_stop = true;
        return cmd;
    }

    constexpr float NO_OBSTACLE_CM = 100000.0f;
    const float left_cm  = lidar.left_min_cm  ? *lidar.left_min_cm  : NO_OBSTACLE_CM;
    const float right_cm = lidar.right_min_cm ? *lidar.right_min_cm : NO_OBSTACLE_CM;
    const bool left_clear  = left_cm  > 60.0f;
    const bool right_clear = right_cm > 60.0f;

    // Ưu tiên bên rộng hơn để giữ an toàn.
    if (left_clear && right_clear) {
        const bool prefer_left = left_cm >= right_cm;
        std::cout << "[OBSTACLE] Both sides clear (L=" << left_cm
                  << "cm R=" << right_cm << "cm), prefer "
                  << (prefer_left ? "LEFT" : "RIGHT") << std::endl;
        change_state(prefer_left ? BypassState::SWERVE_LEFT : BypassState::SWERVE_RIGHT);
        return prefer_left ? handle_swerve_left(lidar) : handle_swerve_right(lidar);
    }

    if (left_clear) {
        std::cout << "[OBSTACLE] Left side clear (" << left_cm
                  << "cm), switching to SWERVE_LEFT" << std::endl;
        change_state(BypassState::SWERVE_LEFT);
        return handle_swerve_left(lidar);
    }

    if (right_clear) {
        std::cout << "[OBSTACLE] Right side clear (" << right_cm
                  << "cm), switching to SWERVE_RIGHT" << std::endl;
        change_state(BypassState::SWERVE_RIGHT);
        return handle_swerve_right(lidar);
    }

    // Cả hai đều kín: dừng hẳn thay vì ép EMERGENCY (tránh phanh đột ngột
    // khi thực tế chỉ là hội quá chật).
    std::cout << "[OBSTACLE] Both sides blocked (L=" << left_cm
              << "cm R=" << right_cm
              << "cm), holding STOP" << std::endl;
    cmd.emergency_stop = true;
    return cmd;
}


BypassCommand ObstacleAvoidance::handle_swerve_left(const LidarStatus& lidar) {
    BypassCommand cmd;
    cmd.state = BypassState::SWERVE_LEFT;
    cmd.state_name = "SWERVE_LEFT";
    cmd.speed_control = 30;
    cmd.dev_final_px = -60;
    cmd.emergency_stop = false;

    if (!swerve_start_front_dist_ && lidar.ob_front_cm) {
        swerve_start_front_dist_ = *lidar.ob_front_cm;
        std::cout << "[SWERVE_LEFT] Start front distance: "
                  << *swerve_start_front_dist_ << "cm" << std::endl;
    }

    // Chụp lại khoảng cách vật cản BÊN phải (vật cản ta đang né sẽ nằm bên
    // phải sau khi lái trái). Dùng để biết đã đi qua xong hay chưa.
    if (!swerve_start_right_dist_ && lidar.ob_right_cm) {
        swerve_start_right_dist_ = *lidar.ob_right_cm;
    }

    if (get_state_elapsed_ms() < 300) {
        return cmd;
    }

    // ob_front_cm == nullopt (scan con song, vung truoc trong) cung la DA THOAT.
    // Neu chi chu "co diem" thi vay can ra khoi vung truoc san khong bao gio
    // duoc coi la clear, va SWERVE chi ket bang timeout.
    const bool front_clear =
        swerve_start_front_dist_ &&
        (!lidar.ob_front_cm || *lidar.ob_front_cm > *swerve_start_front_dist_ + 5.0f);

    if (lidar.ob_front_cm && swerve_start_front_dist_) {
        std::cout << "[SWERVE_LEFT] Front: " << *lidar.ob_front_cm
                  << "cm (start: " << *swerve_start_front_dist_ << "cm)" << std::endl;
    } else if (swerve_start_front_dist_) {
        std::cout << "[SWERVE_LEFT] Front: CLEAR (start: "
                  << *swerve_start_front_dist_ << "cm)" << std::endl;
    }

    if (front_clear /* && right_safe */) {
        std::cout << "[OBSTACLE] Swerve complete, switching to BYPASS_LEFT" << std::endl;
        change_state(BypassState::BYPASS_LEFT);
        return handle_bypass_left(lidar);
    }

    // Timeout an toàn: nếu sau SWERVE_TIMEOUT_MS vẫn chưa thoát được,
    // KHÔNG được giữ lái góc cứng -60 mãi. Quay lại chọn hướng né khác,
    // hoặc dừng hẳn nếu cả hai bên đều kín.
    if (get_state_elapsed_ms() > SWERVE_TIMEOUT_MS) {
        std::cout << "[SWERVE_LEFT] Timeout " << SWERVE_TIMEOUT_MS
                  << "ms without escape, stopping safely" << std::endl;
        cmd.speed_control = 0;
        cmd.dev_final_px = 0;
        cmd.emergency_stop = true;
        return cmd;
    }

    return cmd;
}

BypassCommand ObstacleAvoidance::handle_swerve_right(const LidarStatus& lidar) {
    BypassCommand cmd;
    cmd.state = BypassState::SWERVE_RIGHT;
    cmd.state_name = "SWERVE_RIGHT";
    cmd.speed_control = 30;
    cmd.dev_final_px = 60;
    cmd.emergency_stop = false;

    if (!swerve_start_front_dist_ && lidar.ob_front_cm) {
        swerve_start_front_dist_ = *lidar.ob_front_cm;
        std::cout << "[SWERVE_RIGHT] Start front distance: "
                  << *swerve_start_front_dist_ << "cm" << std::endl;
    }

    if (!swerve_start_left_dist_ && lidar.ob_left_cm) {
        swerve_start_left_dist_ = *lidar.ob_left_cm;
    }

    if (get_state_elapsed_ms() < 300) {
        return cmd;
    }

    // ob_front_cm == nullopt (scan con song, vung truoc trong) cung la DA THOAT.
    // Neu chi chu "co diem" thi vay can ra khoi vung truoc san khong bao gio
    // duoc coi la clear, va SWERVE chi ket bang timeout.
    const bool front_clear =
        swerve_start_front_dist_ &&
        (!lidar.ob_front_cm || *lidar.ob_front_cm > *swerve_start_front_dist_ + 5.0f);

    if (lidar.ob_front_cm && swerve_start_front_dist_) {
        std::cout << "[SWERVE_RIGHT] Front: " << *lidar.ob_front_cm
                  << "cm (start: " << *swerve_start_front_dist_ << "cm)" << std::endl;
    } else if (swerve_start_front_dist_) {
        std::cout << "[SWERVE_RIGHT] Front: CLEAR (start: "
                  << *swerve_start_front_dist_ << "cm)" << std::endl;
    }


    if (front_clear /* && left_safe */) {
        std::cout << "[OBSTACLE] Swerve complete, switching to BYPASS_RIGHT" << std::endl;
        change_state(BypassState::BYPASS_RIGHT);
        return handle_bypass_right(lidar);
    }

    if (get_state_elapsed_ms() > SWERVE_TIMEOUT_MS) {
        std::cout << "[SWERVE_RIGHT] Timeout " << SWERVE_TIMEOUT_MS
                  << "ms without escape, stopping safely" << std::endl;
        cmd.speed_control = 0;
        cmd.dev_final_px = 0;
        cmd.emergency_stop = true;
        return cmd;
    }

    return cmd;
}

BypassCommand ObstacleAvoidance::handle_bypass_left(const LidarStatus& lidar) {
    BypassCommand cmd;
    cmd.state = BypassState::BYPASS_LEFT;
    cmd.state_name = "BYPASS_LEFT";
    cmd.speed_control = 30;
    cmd.dev_final_px = 35;
    cmd.emergency_stop = false;

    // ✅ Wall-following logic TRƯỚC (theo dõi vật cản bên phải)
    if (lidar.ob_right_cm) {
        float current_dist = *lidar.ob_right_cm;
        // Target: 30cm ± 5cm
        if (current_dist > 30.0f) {
            cmd.dev_final_px = + 25;
            std::cout << "[BYPASS_LEFT] Too far (" << current_dist
                      << "cm), steer right +20" << std::endl;
        } 
        if (current_dist < 20.0f) {
            cmd.dev_final_px = - 25;
            std::cout << "[BYPASS_LEFT] Too close (" << current_dist
                      << "cm), steer left -20" << std::endl;
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
        // Hai điều kiện ĐỒNG BỘ (cùng lấy từ vùng phía sau bên phải):
        //  1. Vùng sau-phải đã trống  -> đã lách qua đuôi vật cản
        //  2. Vùng bên phải đã mở rộng so với lúc bắt đầu SWERVE
        // Trước đây nhánh 2 so khoảng cách BÊN với khoảng cách PHÍA TRƯỚC
        // -> điều kiện vô nghĩa, bypass thường không bao giờ hoàn tất.
        const float rear_cm = lidar.ob_right_rear_cm ? *lidar.ob_right_rear_cm : 100000.0f;
        const bool rear_clear = rear_cm > 30.0f;
        const bool right_open = lidar.ob_right_cm && swerve_start_right_dist_ &&
                                (*lidar.ob_right_cm > *swerve_start_right_dist_ + 5.0f);

        if (lidar.ob_right_cm && swerve_start_right_dist_) {
            std::cout << "[BYPASS_LEFT] Right: " << *lidar.ob_right_cm
                      << "cm, Start: " << *swerve_start_right_dist_ << "cm" << std::endl;
        }

        if (rear_clear && right_open) {
            std::cout << "[OBSTACLE] Obstacle passed (rear="
                      << (lidar.ob_right_rear_cm ? *lidar.ob_right_rear_cm : -1.0f)
                      << "cm, right="
                      << (lidar.ob_right_cm ? *lidar.ob_right_cm : -1.0f)
                      << "cm), switching to RETURN_LANE_RIGHT" << std::endl;
            change_state(BypassState::RETURN_LANE_RIGHT);
            return handle_return_lane_right(lidar, false);
        } else {
            std::cout << "[BYPASS_LEFT] Waiting... rear_clear=" << rear_clear
                      << ", right_open=" << right_open << std::endl;
        }
    }

    // Chặn trên: vật cản trước lại gần -> phải cân nhắc lại phương án né
    if (lidar.ob_front_cm && *lidar.ob_front_cm <= (70.0f + speed_margin_cm_)) {
        std::cout << "[BYPASS_LEFT] New front obstacle, re-evaluating" << std::endl;
        change_state(BypassState::DETECT_BYPASS_SIDE);
        return handle_detect_bypass_side(lidar);
    }

    if (get_state_elapsed_ms() > BYPASS_TIMEOUT_MS) {
        std::cout << "[BYPASS_LEFT] Timeout " << BYPASS_TIMEOUT_MS
                  << "ms, stopping safely" << std::endl;
        cmd.speed_control = 0;
        cmd.dev_final_px = 0;
        cmd.emergency_stop = true;
        return cmd;
    }

    return cmd;
}

BypassCommand ObstacleAvoidance::handle_bypass_right(const LidarStatus& lidar) {
    BypassCommand cmd;
    cmd.state = BypassState::BYPASS_RIGHT;
    cmd.state_name = "BYPASS_RIGHT";
    cmd.speed_control = 30;
    cmd.dev_final_px = -35;
    cmd.emergency_stop = false;

    if (lidar.ob_left_cm) {
        float current_dist = *lidar.ob_left_cm;
        if (current_dist > 30.0f) {
            cmd.dev_final_px = - 25;
            std::cout << "[BYPASS_RIGHT] Too far (" << current_dist
                      << "cm), steer left -20" << std::endl;
        } 
        if (current_dist < 20.0f) {
            cmd.dev_final_px = + 25;
            std::cout << "[BYPASS_RIGHT] Too close (" << current_dist
                      << "cm), steer right +20" << std::endl;
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
        // Giống handle_bypass_left: so sánh cùng một vùng (sau - bên trái)
        // với mốc chụp lúc SWERVE, không dùng khoảng cách phía trước.
        const float rear_cm = lidar.ob_left_rear_cm ? *lidar.ob_left_rear_cm : 100000.0f;
        const bool rear_clear = rear_cm > 30.0f;
        const bool left_open = lidar.ob_left_cm && swerve_start_left_dist_ &&
                               (*lidar.ob_left_cm > *swerve_start_left_dist_ + 5.0f);

        if (lidar.ob_left_cm && swerve_start_left_dist_) {
            std::cout << "[BYPASS_RIGHT] Left: " << *lidar.ob_left_cm
                      << "cm, Start: " << *swerve_start_left_dist_ << "cm" << std::endl;
        }

        if (rear_clear && left_open) {
            std::cout << "[OBSTACLE] Obstacle passed (rear="
                      << (lidar.ob_left_rear_cm ? *lidar.ob_left_rear_cm : -1.0f)
                      << "cm, left="
                      << (lidar.ob_left_cm ? *lidar.ob_left_cm : -1.0f)
                      << "cm), switching to RETURN_LANE_LEFT" << std::endl;
            change_state(BypassState::RETURN_LANE_LEFT);
            return handle_return_lane_left(lidar, false);
        } else {
            std::cout << "[BYPASS_RIGHT] Waiting... rear_clear=" << rear_clear
                      << ", left_open=" << left_open << std::endl;
        }
    }

    if (lidar.ob_front_cm && *lidar.ob_front_cm <= (70.0f + speed_margin_cm_)) {
        std::cout << "[BYPASS_RIGHT] New front obstacle, re-evaluating" << std::endl;
        change_state(BypassState::DETECT_BYPASS_SIDE);
        return handle_detect_bypass_side(lidar);
    }

    if (get_state_elapsed_ms() > BYPASS_TIMEOUT_MS) {
        std::cout << "[BYPASS_RIGHT] Timeout " << BYPASS_TIMEOUT_MS
                  << "ms, stopping safely" << std::endl;
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
    cmd.speed_control = 30;
    cmd.dev_final_px = -80;
    cmd.emergency_stop = false;

    if (get_state_elapsed_ms() < 800) {
        return cmd;
    }

    // Nhận lại sai số làn thật từ camera, không dùng giá trị 0 cứng.
    if (is_dual_lane) {
        std::cout << "[OBSTACLE] Both lanes detected, back to NORMAL" << std::endl;
        change_state(BypassState::NORMAL);
        return handle_normal(lidar, cmd.dev_final_px, 0.0f);
    }

    if (lidar.ob_front_cm && *lidar.ob_front_cm < (130.0f + speed_margin_cm_)) {
        std::cout << "[OBSTACLE] Obstacle detected during return, back to DETECT_BYPASS_SIDE" << std::endl;
        change_state(BypassState::DETECT_BYPASS_SIDE);
        return handle_detect_bypass_side(lidar);
    }

    if (get_state_elapsed_ms() > RETURN_TIMEOUT_MS) {
        std::cout << "[RETURN_LANE_LEFT] Timeout " << RETURN_TIMEOUT_MS
                  << "ms, stopping safely" << std::endl;
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
    cmd.speed_control = 30;
    cmd.dev_final_px = 80;
    cmd.emergency_stop = false;

    if (get_state_elapsed_ms() < 800) {
        return cmd;
    }

    if (is_dual_lane) {
        std::cout << "[OBSTACLE] Both lanes detected, back to NORMAL" << std::endl;
        change_state(BypassState::NORMAL);
        return handle_normal(lidar, cmd.dev_final_px, 0.0f);
    }

    if (lidar.ob_front_cm && *lidar.ob_front_cm < (130.0f + speed_margin_cm_)) {
        std::cout << "[OBSTACLE] Obstacle detected during return, back to DETECT_BYPASS_SIDE" << std::endl;
        change_state(BypassState::DETECT_BYPASS_SIDE);
        return handle_detect_bypass_side(lidar);
    }

    if (get_state_elapsed_ms() > RETURN_TIMEOUT_MS) {
        std::cout << "[RETURN_LANE_RIGHT] Timeout " << RETURN_TIMEOUT_MS
                  << "ms, stopping safely" << std::endl;
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

    if (traffic_light_decision == "GREEN") {
        if (!lidar.ob_front_cm || *lidar.ob_front_cm > (160.0f + speed_margin_cm_)) {
            if (prev_traffic_light_ != "GREEN") {
                std::cout << "[TRAFFIC] GREEN light! Returning to NORMAL" << std::endl;
            }
            prev_traffic_light_ = traffic_light_decision;
            change_state(BypassState::NORMAL);
            return handle_normal(lidar, dev_px, 0.0f);
        } else {
            std::cout << "[TRAFFIC] GREEN but obstacle at " << *lidar.ob_front_cm << "cm" << std::endl;
        }
    }
    
    if (traffic_light_decision != "RED" && traffic_light_decision != "STOP") {
        if (!lidar.ob_front_cm || *lidar.ob_front_cm > (160.0f + speed_margin_cm_)) {
            std::cout << "[OBSTACLE] No RED/STOP and obstacle cleared, back to NORMAL" << std::endl;
            change_state(BypassState::NORMAL);
            return handle_normal(lidar, dev_px, 0.0f);
        }
    }

    prev_traffic_light_ = traffic_light_decision;
    return cmd;
}