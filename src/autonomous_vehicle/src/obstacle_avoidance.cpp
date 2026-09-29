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
    
    if (lidar.ob_front_cm && *lidar.ob_front_cm <= (55.0f )) {
        if (current_state_ != BypassState::EMERGENCY_STOP) {
            std::cout << "[OBSTACLE] EMERGENCY! Front <= " << (55.0f ) 
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
BypassCommand ObstacleAvoidance::handle_normal(const LidarStatus& lidar, int16_t dev_px) {
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
    cmd.speed_control = 30;
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
    cmd.speed_control = 0;
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
    cmd.speed_control = 30;
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
        std::cout << "[SWERVE_LEFT] Front: " << *lidar.ob_front_cm
                  << "cm (start: " << *swerve_start_front_dist_ << "cm)" << std::endl;
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
    cmd.speed_control = 30;
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
        std::cout << "[SWERVE_RIGHT] Front: " << *lidar.ob_front_cm
                  << "cm (start: " << *swerve_start_front_dist_ << "cm)" << std::endl;
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
        bool rear_clear = lidar.ob_right_rear_cm && *lidar.ob_right_rear_cm > 30.0f;
        bool right_clear = false;

        if (lidar.ob_right_cm && swerve_start_front_dist_) {
            right_clear = (*lidar.ob_right_cm > *swerve_start_front_dist_ + 5.0f);
            std::cout << "[BYPASS_LEFT] Right: " << *lidar.ob_right_cm
                      << "cm, Start: " << *swerve_start_front_dist_ << "cm" << std::endl;
        }

        if (rear_clear && right_clear) {
            std::cout << "[OBSTACLE] Obstacle passed (rear=" << *lidar.ob_right_rear_cm
                      << "cm, right=" << *lidar.ob_right_cm
                      << "cm), switching to RETURN_LANE_RIGHT" << std::endl;
            change_state(BypassState::RETURN_LANE_RIGHT);
            return handle_return_lane_right(lidar, false);
        } else {
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
        bool rear_clear = lidar.ob_left_rear_cm && *lidar.ob_left_rear_cm > 30.0f;
        bool left_clear = false;

        if (lidar.ob_left_cm && swerve_start_front_dist_) {
            left_clear = (*lidar.ob_left_cm > *swerve_start_front_dist_ + 5.0f);
            std::cout << "[BYPASS_RIGHT] Left: " << *lidar.ob_left_cm
                      << "cm, Start: " << *swerve_start_front_dist_ << "cm" << std::endl;
        }

        if (rear_clear && left_clear) {
            std::cout << "[OBSTACLE] Obstacle passed (rear=" << *lidar.ob_left_rear_cm
                      << "cm, left=" << *lidar.ob_left_cm
                      << "cm), switching to RETURN_LANE_LEFT" << std::endl;
            change_state(BypassState::RETURN_LANE_LEFT);
            return handle_return_lane_left(lidar, false);
        } else {
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
    cmd.speed_control = 30;
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
    cmd.speed_control = 30;
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
            return handle_normal(lidar, dev_px);
        } else {
            std::cout << "[TRAFFIC] GREEN but obstacle at " << *lidar.ob_front_cm << "cm" << std::endl;
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