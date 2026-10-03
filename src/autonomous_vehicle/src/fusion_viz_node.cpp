#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/string.hpp>
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <opencv2/opencv.hpp>
#include <json/json.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <exception>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "camera_lane.hpp"
#include "lane_mask.hpp"
#include "lidar_module.hpp"
#include "serial_esp32.hpp"
#include "obstacle_avoidance.hpp"

using std::placeholders::_1;
using namespace cv;
namespace {

// Cao đủ chứa 5 dòng chữ nhỏ (ESP32 + AI dồn chung một cột) - bản cũ 100px
// khiến dòng "Light:"/"Turn:" ở dưới bị cắt.
constexpr int HEADER_H = 130;
constexpr int RIGHT_PANEL_W = 400;

// Chu kỳ timer từ tần số Hz. Chặn dưới 1 Hz để không tạo period = 0.
std::chrono::nanoseconds period_from_hz(double hz) {
    const double safe_hz = (hz > 1.0) ? hz : 1.0;
    return std::chrono::nanoseconds(static_cast<int64_t>(1e9 / safe_hz));
}

void draw_header(cv::Mat& full_img, const LaneOutput* lane, const LidarStatus& lidar,
                double cam_fps, double lidar_fps, bool serial_ok, const ESP32Feedback& esp_fb,
                BypassState bypass_state, int16_t dev_final,
                const std::string& traffic_light_decision,  
                const std::string& turn_decision) {       
    
    cv::rectangle(full_img, cv::Rect(0, 0, full_img.cols, HEADER_H), cv::Scalar(30, 30, 30), cv::FILLED);
    char text[64];
    auto fmt = [](std::optional<float> d) -> std::string {
        if (!d) return "---";
        float val = *d;
        char buf[16];
        if (val >= 199.5f) {
            snprintf(buf, sizeof(buf), ">200cm");
        } else {
            snprintf(buf, sizeof(buf), "%dcm", (int)val);
        }
        return std::string(buf);
    };
    // Bố cục 4 cột. Bản cũ xếp 5 cột ở x = 15 / 300 / 615 / 865 / 1050 trong
    // khi ảnh chỉ rộng 1000 (map 600 + panel 400) => cột thứ 4 và 5 nằm ngoài
    // mép phải, ESP32 và AI không bao giờ hiện. Nay gộp ESP32 + AI vào một
    // cột hẹp ở mép phải.
    int x1 = 10, y1 = 25;
    cv::putText(full_img, "LIDAR", {x1, y1}, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(100, 255, 255), 2, cv::LINE_AA);
    y1 += 23;
    snprintf(text, sizeof(text), "FPS: %.0f | Status: %s", lidar_fps, lidar.has_data ? "OK" : "---");
    cv::putText(full_img, text, {x1, y1}, cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(200,200,200), 1, cv::LINE_AA);
    y1 += 22;
    snprintf(text, sizeof(text), "Front: %s | Rear: %s",
            fmt(lidar.front_min_cm).c_str(),
            fmt(lidar.rear_bypass_min_cm).c_str());
    cv::putText(full_img, text, {x1, y1}, cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(200,200,200), 1, cv::LINE_AA);
    y1 += 22;
    snprintf(text, sizeof(text), "Left: %s | Right: %s",
            fmt(lidar.left_min_cm).c_str(),
            fmt(lidar.right_min_cm).c_str());
    cv::putText(full_img, text, {x1, y1}, cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(200,200,200), 1, cv::LINE_AA);
    int x2 = 250, y2 = 25;
    cv::putText(full_img, "OBSTACLE AVOIDANCE", {x2, y2}, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 200, 100), 2, cv::LINE_AA);
    y2 += 23;

    std::string state_str = "State: ";
    cv::Scalar state_color;
    switch (bypass_state) {
        case BypassState::NORMAL: state_str += "NORMAL"; state_color = cv::Scalar(100, 255, 100); break;
        case BypassState::SLOW_DOWN: state_str += "SLOW_DOWN"; state_color = cv::Scalar(0, 255, 255); break;
        case BypassState::DETECT_BYPASS_SIDE: state_str += "DETECT_BYPASS"; state_color = cv::Scalar(0, 165, 255); break;
        case BypassState::SWERVE_LEFT: state_str += "SWERVE_LEFT"; state_color = cv::Scalar(255, 0, 255); break;
        case BypassState::SWERVE_RIGHT: state_str += "SWERVE_RIGHT"; state_color = cv::Scalar(255, 0, 255); break;
        case BypassState::BYPASS_LEFT: state_str += "BYPASS_LEFT"; state_color = cv::Scalar(255, 128, 0); break;
        case BypassState::BYPASS_RIGHT: state_str += "BYPASS_RIGHT"; state_color = cv::Scalar(255, 128, 0); break;
        case BypassState::RETURN_LANE_LEFT: state_str += "RETURN_LEFT"; state_color = cv::Scalar(255, 255, 0); break;
        case BypassState::RETURN_LANE_RIGHT: state_str += "RETURN_RIGHT"; state_color = cv::Scalar(255, 255, 0); break;
        case BypassState::EMERGENCY_STOP: state_str += "EMERGENCY!"; state_color = cv::Scalar(0, 0, 255); break;
        default: state_str += "UNKNOWN"; state_color = cv::Scalar(128, 128, 128); break;
    }

    cv::putText(full_img, state_str, {x2, y2}, cv::FONT_HERSHEY_SIMPLEX, 0.55, state_color, 2, cv::LINE_AA);
    y2 += 22;
    snprintf(text, sizeof(text), "Dev Final: %d px", dev_final);
    cv::putText(full_img, text, {x2, y2}, cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(200,200,200), 1, cv::LINE_AA);
    int x3 = 520, y3 = 25;
    cv::putText(full_img, "LANE DETECTION", {x3, y3}, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(100, 255, 100), 2, cv::LINE_AA);
    y3 += 23;

    double cam_fps_display = std::min(cam_fps, 250.0);
    char fps_text[64];
    snprintf(fps_text, sizeof(fps_text), "FPS: %.1f", cam_fps_display);
    cv::putText(full_img, fps_text, {x3, y3}, cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(200,200,200), 1, cv::LINE_AA);
    y3 += 22;

    std::string dir = (lane && lane->valid) ? lane->camera_cmd : "---";
    snprintf(text, sizeof(text), "DIR: %s", dir.c_str());
    cv::putText(full_img, text, {x3, y3}, cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(200,200,200), 1, cv::LINE_AA);
    y3 += 22;

    snprintf(text, sizeof(text), "Dev: %d px | %s", lane ? lane->dev_final_px : 0,
             (lane && lane->detector_mode == 1) ? "IPM" : "SCAN");
    cv::putText(full_img, text, {x3, y3}, cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(200,200,200), 1, cv::LINE_AA);

    // Ảnh rộng đúng 1000 (map 600 + panel 400). 5 cột phải nằm trong đó:
    //   10 / 250 / 520 / 745 / 880, mỗi cột ~120px, cột cuối kết ở 990.
    // Bản cũ xếp ở 15 / 300 / 615 / 865 / 1050 -> cột 4 và 5 nằm ngoài mép
    // phải nên ESP32 và AI không bao giờ hiện.
    int x4 = 745, y4 = 25;
    cv::putText(full_img, "ESP32", {x4, y4}, cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(255, 200, 100), 2, cv::LINE_AA);
    y4 += 21;
    snprintf(text, sizeof(text), "%s", serial_ok ? "OK" : "FAIL");
    cv::putText(full_img, text, {x4, y4}, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200,200,200), 1, cv::LINE_AA);
    y4 += 20;
    snprintf(text, sizeof(text), "V: %.1f", esp_fb.valid ? esp_fb.velocity_kmh : 0.0f);
    cv::putText(full_img, text, {x4, y4}, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200,200,200), 1, cv::LINE_AA);

    int x5 = 880, y5 = 25;
    cv::putText(full_img, "AI", {x5, y5},
                cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(255, 100, 255), 2, cv::LINE_AA);
    y5 += 21;

    cv::Scalar traffic_color;
    if (traffic_light_decision == "RED" || traffic_light_decision == "STOP") {
        traffic_color = cv::Scalar(0, 0, 255);
    } else if (traffic_light_decision == "YELLOW" || traffic_light_decision == "20") {
        traffic_color = cv::Scalar(0, 255, 255);
    } else if (traffic_light_decision == "GREEN") {
        traffic_color = cv::Scalar(0, 255, 0);
    } else {
        traffic_color = cv::Scalar(150, 150, 150);
    }

    snprintf(text, sizeof(text), "L:%s", traffic_light_decision.c_str());
    cv::putText(full_img, text, {x5, y5},
                cv::FONT_HERSHEY_SIMPLEX, 0.5, traffic_color, 1, cv::LINE_AA);
    y5 += 20;

    cv::Scalar turn_color = (turn_decision == "NONE") ?
        cv::Scalar(150, 150, 150) : cv::Scalar(0, 255, 255);

    snprintf(text, sizeof(text), "T:%s", turn_decision.c_str());
    cv::putText(full_img, text, {x5, y5},
                cv::FONT_HERSHEY_SIMPLEX, 0.5, turn_color, 1, cv::LINE_AA);
}

void draw_sector_lines(cv::Mat& map, const cv::Point& origin) {
    int radius = (int)(80 * LidarModule::PX_PER_CM);
    std::vector<std::pair<int, cv::Scalar>> sector_angles = {
        {60, cv::Scalar(60,60,60)}, {120, cv::Scalar(60,60,60)},
        {240, cv::Scalar(60,60,60)}, {300, cv::Scalar(60,60,60)},
    };

    for (auto& [angle_deg, color] : sector_angles) {
        float rad = angle_deg * CV_PI / 180.0f;
        int ex = origin.x + (int)(radius * std::cos(rad));
        int ey = origin.y - (int)(radius * std::sin(rad));
        cv::line(map, origin, cv::Point(ex, ey), color, 2, cv::LINE_AA);

        int lx = origin.x + (int)((radius + 20) * std::cos(rad));
        int ly = origin.y - (int)((radius + 20) * std::sin(rad));
        cv::putText(map, std::to_string(angle_deg) + "Deg", cv::Point(lx - 20, ly + 5),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv::LINE_AA);
    }
}

void draw_camera_panel(cv::Mat& panel, const cv::Mat& camera_img) {
    panel.setTo(cv::Scalar(40, 40, 40));
    if (!camera_img.empty() && camera_img.rows > 0 && camera_img.cols > 0) {
        cv::Mat small;
        cv::resize(camera_img, small, cv::Size(320, 200));
        int cam_x = (panel.cols - 320) / 2;
        int cam_y = (panel.rows - 200) / 2;
        cv::Rect cam_roi(cam_x, cam_y, 320, 200);
        if (cam_roi.x >= 0 && cam_roi.y >= 0 &&
            cam_roi.x + cam_roi.width <= panel.cols &&
            cam_roi.y + cam_roi.height <= panel.rows) {
            small.copyTo(panel(cam_roi));
        }
    }
}

void draw_ai_detection_panel(cv::Mat& panel, 
                             const std::string& traffic_light_decision,
                             const std::string& turn_decision) {
    panel.setTo(cv::Scalar(40, 40, 40));
    cv::putText(panel, "AI SIGN", cv::Point(10, 35),
               cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 100), 2, cv::LINE_AA);
    
    struct SignInfo {
        std::string display_name;
        std::string function;
        cv::Scalar color;
    };
    
    static const std::map<std::string, SignInfo> sign_map = {
        {"STOP",   {"STOP SIGN",           "Dung lai ngay / Stop immediately",      cv::Scalar(0, 0, 255)}},
        {"RED",    {"RED LIGHT",           "Den do - Cam di / Red - Stop",          cv::Scalar(0, 0, 255)}},
        {"YELLOW", {"YELLOW LIGHT",        "Den vang - Giam toc / Yellow - Slow",   cv::Scalar(0, 255, 255)}},
        {"GREEN",  {"GREEN LIGHT",         "Den xanh - Duoc di / Green - Go",       cv::Scalar(0, 255, 0)}},
        {"20",     {"SPEED LIMIT 20",      "Gioi han 20km/h / Max 20km/h",          cv::Scalar(0, 165, 255)}},
        {"TURN_LEFT",  {"<- TURN LEFT",    "Re trai / Turn left ahead",             cv::Scalar(255, 0, 255)}},
        {"TURN_RIGHT", {"TURN RIGHT ->",   "Re phai / Turn right ahead",            cv::Scalar(255, 0, 255)}},
        {"NONE",   {"NO DETECTION",        "Khong phat hien / No sign detected",    cv::Scalar(100, 100, 100)}}
    };
    
    int y = 75;
    int line_spacing = 28;
    int sign_spacing = 15;
    int sign_count = 0;
    std::vector<std::string> active_signs;
    
    if (traffic_light_decision != "NONE") {
        active_signs.push_back(traffic_light_decision);
    }
    
    if (turn_decision != "NONE") {
        active_signs.push_back(turn_decision);
    }
    if (active_signs.empty()) {
        auto info = sign_map.at("NONE");
        
        cv::putText(panel, info.display_name, cv::Point(15, y),
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, info.color, 2, cv::LINE_AA);
        y += line_spacing;
        
        cv::putText(panel, info.function, cv::Point(20, y),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(150, 150, 150), 1, cv::LINE_AA);
        
        return;
    }
    
    for (const auto& sign : active_signs) {
        if (y + line_spacing * 2 + sign_spacing > panel.rows - 20) {
            const int remaining = static_cast<int>(active_signs.size()) - sign_count;
            if (remaining > 0) {
                char buf[64];
                snprintf(buf, sizeof(buf), "... +%d more signs", remaining);
                cv::putText(panel, buf, cv::Point(15, y),
                           cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(150, 150, 150), 1, cv::LINE_AA);
            }
            break;
        }
        
        auto it = sign_map.find(sign);
        if (it != sign_map.end()) {
            const auto& info = it->second;
            cv::putText(panel, info.display_name, cv::Point(15, y),
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, info.color, 2, cv::LINE_AA);
            y += line_spacing;
            cv::putText(panel, info.function, cv::Point(20, y),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1, cv::LINE_AA);
            y += line_spacing + sign_spacing;
            
            sign_count++;
        }
    }
}
}
class FusionVizNode : public rclcpp::Node {
public:
    FusionVizNode()
        : Node("fusion_viz_node"),
          cam_index_(declare_parameter("cam_index", 0)),
          fps_(declare_parameter("camera_fps", 30)),
          cam_fps_(0.0), lidar_fps_(0.0),
          last_cam_time_(now()),
          last_lidar_time_(now()),
          serial_ok_(false),
          bypass_state_(BypassState::NORMAL),
          last_signs_update_(now()),
          traffic_light_decision_("NONE"),   
          turn_decision_("NONE")             
    {
        cam_ = std::make_unique<CameraLane>(cam_index_, fps_, true);

        // ROI theo tỉ lệ chiều cao ảnh (mép trên = tầm xa, mép dưới = gần).
        // Camera nghiêng xuống nên khoảng cách là hàm siêu tuyến theo hàng:
        // ROI hẹp ở đáy chỉ che vài chục cm. Cho chỉnh bằng tham số để tìm
        // trên xe thay vì sửa lại code.
        cam_->set_roi_factors(
            static_cast<float>(declare_parameter("roi_factor_dual",
                                                 CameraLane::ROI_FACTOR_SLOW)),
            static_cast<float>(declare_parameter("roi_factor_single",
                                                 CameraLane::SINGLE_ROI_FACTOR)));

        if (!cam_->start()) RCLCPP_WARN(get_logger(), "Failed to open camera");

        std::string serial_port = declare_parameter("serial_port", "/dev/ttyUSB0");
        serial_ = std::make_unique<SerialESP32>(serial_port);

        // Tốc độ NORMAL. 0 = để bộ lập của detector quyết (STRAIGHT 85 /
        // CURVE 60 / SHARP 45). Đặt 35 để quay lại hành vi cũ khi cần đo
        // lại hằng số bánh.
        //
        // declare_parameter với literal 0 trả về int64_t, không phải int -
        // std::min(255, <int64_t>) không có hàm nào khớp nên không compile.
        // Ép kiểu rõ ràng thành int trước khi kẹp.
        {
            const int64_t raw = declare_parameter("speed_normal_x10", 0);
            const int clamped = std::max(0, static_cast<int>(
                std::min<int64_t>(255, raw)));
            ObstacleAvoidance::set_speed_normal_override(
                static_cast<uint8_t>(clamped));
            RCLCPP_INFO(get_logger(),
                        "Speed NORMAL: %s",
                        clamped > 0
                            ? "OVERRIDE (see oa.speed_override)"
                            : "from lane planner");
        }

        RCLCPP_INFO(get_logger(), "Waiting 2s for ESP32 to boot...");
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        int retry_count = 0;
        while (!serial_ok_ && retry_count < 5) {
            RCLCPP_INFO(get_logger(), "Opening serial port %s (%d/5)...",
                       serial_port.c_str(), retry_count + 1);
            serial_ok_ = serial_->open();
            if (!serial_ok_) {
                RCLCPP_WARN(get_logger(), "Failed, retrying in 1s...");
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                retry_count++;
            }
        }

        if (!serial_ok_) {
            RCLCPP_ERROR(get_logger(), "Failed to open serial after 5 retries");
        } else {
            RCLCPP_INFO(get_logger(), " Serial port opened successfully");
        }

        // RPLiDAR publishes with RELIABLE QoS, SensorDataQoS is BEST_EFFORT -> mismatch
        auto scan_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
        sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
            declare_parameter("scan_topic", "/scan"),
            scan_qos,
            std::bind(&FusionVizNode::on_scan, this, _1)
        );

        // Góc hiệu chỉnh lắp đặt LiDAR. KHÔNG được bỏ qua: LiDARModule mặc
        // định -90°, nếu LiDAR thực tế không lắp lệch đúng góc đó thì cung
        // "phía trước" của logic né tránh lệch sang cung khác => ob_front_cm
        // không bao giờ thấy vật cản thật phía trước => xe không giảm tốc,
        // không dừng. Tham số này là thứ duy nhất bù được sai lệch lắp đặt.
        lidar_.set_mount_offset_deg(static_cast<float>(
            declare_parameter("lidar_mount_offset_deg",
                             static_cast<double>(
                                 LidarModule::DEFAULT_MOUNT_OFFSET_DEG))));

        sub_signs_ = create_subscription<std_msgs::msg::String>(
            "/autocar/sign_detection",
            rclcpp::QoS(10),
            std::bind(&FusionVizNode::on_signs, this, _1)
        );

        sub_signs_img_ = create_subscription<sensor_msgs::msg::Image>(
            "/autocar/sign_image",
            rclcpp::QoS(10),
            std::bind(&FusionVizNode::on_signs_img, this, _1)
        );

        sub_traffic_light_ = create_subscription<std_msgs::msg::String>(
            "/traffic_light/decision",
            rclcpp::QoS(10),
            std::bind(&FusionVizNode::on_traffic_light, this, _1)
        );
        
        sub_turn_detector_ = create_subscription<std_msgs::msg::String>(
            "/turn_detector/decision",
            rclcpp::QoS(10),
            std::bind(&FusionVizNode::on_turn_detector, this, _1)
        );
        
        RCLCPP_INFO(get_logger(), "AI Detection subscribers initialized");

        pub_img_ = create_publisher<sensor_msgs::msg::Image>(
            declare_parameter("image_topic", "/fusion_viz/image"), 1);
        pub_raw_img_ = create_publisher<sensor_msgs::msg::Image>(
            declare_parameter("raw_image_topic", "/image_raw"), 1);

        // ------------------------------------------------------------------
        // KÊNH CHẨN ĐOÁN CHO GUI (autonomous_vehicle_gui)
        //
        // Tách khỏi /fusion_viz/image vì hai lý do:
        //  1. /fusion_viz/image là ảnh dàn hình 1265x900 dựng bằng OpenCV để
        //     nhìn nhanh. GUI cần 4 ảnh riêng + JSON trạng thái để hiệu
        //     chỉnh được từng tham số, không dùng lại được bản dàn hình.
        //  2. Cho phép dbg_rate_hz độc lập với viz_hz. Số phải cập nhật
        //     nhanh (status) và ảnh chậm được không cần cùng nhịp.
        //
        // CompressedImage thay vì Image: JPEG ở 1265x900 khoảng 90 KB so với
        // 3.4 MB thô - giảm ~97% băng thông. GUI chỉ cần vẽ, không cần
        // pixel chính xác.
        // ------------------------------------------------------------------
        dbg_rate_hz_ = declare_parameter("dbg_rate_hz", 10.0);
        enable_dbg_ = declare_parameter("enable_dbg", true);

        if (enable_dbg_) {
            pub_dbg_status_ = create_publisher<std_msgs::msg::String>(
                "/autocar/dbg/status", 10);
            pub_dbg_raw_ = create_publisher<sensor_msgs::msg::CompressedImage>(
                "/autocar/dbg/cam_raw/compressed", 1);
            pub_dbg_vis_ = create_publisher<sensor_msgs::msg::CompressedImage>(
                "/autocar/dbg/lane_vis/compressed", 1);
            pub_dbg_bin_ = create_publisher<sensor_msgs::msg::CompressedImage>(
                "/autocar/dbg/lane_bin/compressed", 1);
            pub_dbg_roi_ = create_publisher<sensor_msgs::msg::CompressedImage>(
                "/autocar/dbg/lane_roi/compressed", 1);
            dbg_timer_ = create_wall_timer(
                period_from_hz(dbg_rate_hz_),
                std::bind(&FusionVizNode::on_dbg, this));
            RCLCPP_INFO(get_logger(),
                        "Debug channel ON (%.1f Hz) -> /autocar/dbg/*", dbg_rate_hz_);
        }

        enable_viz_ = declare_parameter("enable_viz", true);

        // ------------------------------------------------------------------
        // HAI TIMER TÁCH BIỆT
        //
        // Bản cũ gộp cả điều khiển lẫn vẽ vào một timer 120 Hz: mỗi vòng đều
        // dựng ảnh 1265x900, encode và publish. Đó là đường an toàn của xe mà
        // lại phụ thuộc tốc độ encode + DDS -> chỉ cần người xem ảnh chậm là
        // xe mất lệnh điều khiển. Nay tách:
        //   control_hz : gửi lệnh ESP32, chạy ở tần số cố định, không vẽ gì.
        //   viz_hz     : dựng + publish ảnh, hạ từ 120 xuống 10 Hz. Ảnh 1000x700
//                ở 120 Hz là ~126 MB/s trên cổng DDS, gần bằng hết
                //                băng thông của một card mạng 100Mbps.
        // ------------------------------------------------------------------
        const double control_hz = declare_parameter("control_hz", 100.0);
        control_timer_ = create_wall_timer(
            period_from_hz(control_hz),
            std::bind(&FusionVizNode::on_control, this));

        const double viz_hz = declare_parameter("viz_hz", 10.0);
        if (enable_viz_) {
            viz_timer_ = create_wall_timer(
                period_from_hz(viz_hz),
                std::bind(&FusionVizNode::on_viz, this));
            RCLCPP_INFO(get_logger(),
                        "Control %.0f Hz | Viz %.1f Hz", control_hz, viz_hz);
        } else {
            RCLCPP_INFO(get_logger(), "Control %.0f Hz | Viz DISABLED", control_hz);
        }

        // ------------------------------------------------------------------
        // RAW TICK: /image_raw cho node AI, tách khỏi viz.
        // Bật theo mặc định vì không bật thì hai detector im lặng, tức xe coi
        // đèn giao thông luôn là xanh. Tắt được bằng raw_image_hz:=0 khi
        // debug ngoài xe và không muốn tốn băng thông.
        // ------------------------------------------------------------------
        const double raw_hz = declare_parameter("raw_image_hz", 5.0);
        raw_pub_enabled_ = raw_hz > 0.0;
        if (raw_pub_enabled_) {
            raw_timer_ = create_wall_timer(
                period_from_hz(raw_hz),
                std::bind(&FusionVizNode::on_raw, this));
            RCLCPP_INFO(get_logger(), "Raw image %.1f Hz", raw_hz);
        }
    }

    ~FusionVizNode() override {
        if (cam_) cam_->stop();
        if (serial_) serial_->close();
    }

private:
    void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        auto t = now();
        double dt = (t - last_lidar_time_).seconds();
        if (dt > 0.001) {
            double inst = 1.0 / dt;
            if (inst <= 250.0) lidar_fps_ = 0.85 * lidar_fps_ + 0.15 * inst;
        }
        last_lidar_time_ = t;
        last_lidar_ = lidar_.update(*msg);
    }

    void on_signs(const std_msgs::msg::String::SharedPtr msg) {
        try {
            Json::CharReaderBuilder reader;
            std::string errors;
            std::istringstream s(msg->data);
            Json::parseFromStream(reader, s, &last_signs_json_, &errors);
            last_signs_update_ = now();
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "Failed to parse signs JSON");
        }
    }

    void on_signs_img(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            last_signs_img_ = cv_ptr->image.clone();
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        }
    }
    void on_traffic_light(const std_msgs::msg::String::SharedPtr msg) {
        traffic_light_decision_ = msg->data;
        
        static std::string last_decision = "NONE";
        if (msg->data != last_decision && msg->data != "NONE") {
            RCLCPP_INFO(get_logger(), "🚦 Traffic Light: %s", msg->data.c_str());
            last_decision = msg->data;
        }
    }
    void on_turn_detector(const std_msgs::msg::String::SharedPtr msg) {
        turn_decision_ = msg->data;
        
        static std::string last_turn = "NONE";
        if (msg->data != last_turn && msg->data != "NONE") {
            RCLCPP_INFO(get_logger(), "↩️  Turn Signal: %s", msg->data.c_str());
            last_turn = msg->data;
        }
    }

    // ======================================================================
    // CONTROL TICK - đường an toàn, không được phụ thuộc visualization
    // ======================================================================
    void on_control() {
        const ESP32Feedback esp_fb = serial_->get_latest_feedback();
        const float current_speed = esp_fb.valid ? esp_fb.velocity_kmh : 0.0f;

        LaneOutput lo;
        const bool has_cam = (cam_ && cam_->get_latest(lo, current_speed));

        if (has_cam) {
            const auto t = now();
            const double dt = (t - last_cam_time_).seconds();
            if (dt > 0.001) {
                const double inst = 1.0 / dt;
                if (inst <= 250.0) cam_fps_ = 0.9 * cam_fps_ + 0.1 * inst;
            }
            last_cam_time_ = t;
        }

        // LiDAR stale khi chưa từng nhận scan nào, hoặc scan cuối đã quá hạn.
        // Bỏ nhánh "chưa có scan" thì lúc khởi động xe sẽ tin là LiDAR khoẻ
        // trong khi thực tế không có dữ liệu vật cản nào.
        const bool lidar_stale =
            !last_lidar_.has_data ||
            (now() - last_lidar_time_).seconds() * 1000.0 >
                static_cast<double>(LIDAR_STALE_MS);

        // lo.stale do CameraLane tự tính từ steady_clock. Không đọc được
        // frame nào cũng phải coi là stale, không phải dev = 0 hợp lệ.
        const bool camera_stale = !has_cam || lo.stale;

        // lane.curvature là |slope| thô (px/px). curve_angle_deg là ĐỘ (0..90),
        // nên bản cũ truyền curve_angle_deg / 57.3 vào ngưỡng slope là sai đơn
        // vị: 30 độ -> 0.52 thay vì tan(30 độ) = 0.58, còn đường thẳng thì vẫn
        // ra 0 nên không lộ, chỉ sai khi vào cua.
        const int16_t deviation = has_cam ? lo.dev_final_px : 0;
        const bool is_dual_lane = has_cam && lo.is_dual_lane;
        const float dominant_slope = has_cam ? lo.curvature : 0.0f;

        // Tốc độ detector yêu cầu (km/h x 10). 0 khi detector không có lane hợp lệ,
        // lúc đó ObstacleAvoidance tự dùng SPEED_NO_LANE_X10.
        const uint8_t lane_speed_x10 = has_cam ? lo.target_speed_x10 : 0;

        const BypassCommand bypass_cmd = obstacle_avoidance_.update(
            last_lidar_, deviation, dominant_slope, is_dual_lane,
            traffic_light_decision_, current_speed,
            lane_speed_x10,
            camera_stale, lidar_stale);

        bypass_state_ = bypass_cmd.state;
        dev_final_ = bypass_cmd.dev_final_px;

        // Chuyển sang viz (cùng executor một luồng nên không cần khoá).
        last_lane_ = lo;
        has_cam_ = has_cam;
        last_esp_fb_ = esp_fb;

        if (serial_ok_ && serial_->is_open()) {
            SerialCommand scmd;
            scmd.dev_final_px = bypass_cmd.dev_final_px;
            scmd.speed_control = bypass_cmd.speed_control;
            scmd.emergency_stop = bypass_cmd.emergency_stop;

            if (!serial_->send_command(scmd)) {
                static auto last_warn = now();
                if ((now() - last_warn).seconds() >= 1.0) {
                    RCLCPP_WARN(get_logger(), "Failed to send serial command");
                    last_warn = now();
                }
            }
        }
    }

    // ======================================================================
    // VIZ TICK - chỉ để xem, không được ảnh hưởng điều khiển
    // ======================================================================
    void on_viz() {
        if (!enable_viz_) return;

        int map_w = LidarModule::MAP_W;
        int map_h = LidarModule::MAP_H;
        int total_width = map_w + RIGHT_PANEL_W;
        int total_height = HEADER_H + map_h;
        cv::Mat full_img(total_height, total_width, CV_8UC3, cv::Scalar(30,30,30));
        cv::Rect map_roi(0, HEADER_H, map_w, map_h);
        cv::Mat map = full_img(map_roi);
        map.setTo(cv::Scalar(25,25,25));

        cv::Point O = LidarModule::origin();
        circle(map, O, 6, Scalar(0,255,0), FILLED, LINE_AA);
        circle(map, O, (int)(20*LidarModule::PX_PER_CM), Scalar(60,60,60), 1, LINE_AA);
        circle(map, O, (int)(40*LidarModule::PX_PER_CM), Scalar(60,60,60), 1, LINE_AA);
        circle(map, O, (int)(60*LidarModule::PX_PER_CM), Scalar(70,70,70), 1, LINE_AA);
        circle(map, O, (int)(80*LidarModule::PX_PER_CM), Scalar(80,80,80), 2, LINE_AA);
        line(map, Point(O.x,0), Point(O.x,map.rows-1), Scalar(70,70,70),1,LINE_AA);
        line(map, Point(0,O.y), Point(map.cols-1,O.y), Scalar(70,70,70),1,LINE_AA);

        draw_sector_lines(map, O);

        if (last_lidar_.has_data) {
            for (const auto& p : last_lidar_.points_px)
                circle(map, p, 2, Scalar(0,180,255), FILLED, LINE_AA);
        }
        const ESP32Feedback& esp_fb = last_esp_fb_;
        const LaneOutput& lo = last_lane_;
        const bool has_cam = has_cam_;

        // Hai panel phải cộng đúng MAP_H, không cộng cứng 250+350=600 khi MAP_H
        // đổi thì Rect vượt biên -> cv::Mat roi sai kích thước.
        const int panel_h = map_h / 2;
        Mat panel1 = full_img(Rect(map_w, HEADER_H, RIGHT_PANEL_W, panel_h));
        draw_camera_panel(panel1, has_cam ? lo.vis : cv::Mat());

        Mat panel2 = full_img(Rect(map_w, HEADER_H + panel_h, RIGHT_PANEL_W, map_h - panel_h));
        draw_ai_detection_panel(panel2, traffic_light_decision_, turn_decision_);

        draw_header(full_img, has_cam ? &lo : nullptr, last_lidar_,
                cam_fps_, lidar_fps_, serial_ok_, esp_fb,
                bypass_state_, dev_final_,
                traffic_light_decision_, turn_decision_);
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", full_img).toImageMsg();
        pub_img_->publish(*msg);
    }

    // ======================================================================
    // RAW TICK - /image_raw là nguồn ảnh cho hai node AI (NCNN).
    //
    // Phát ở đây chứ không nhét vào on_viz: hai node AI không quan tâm tới
    // HUD, và trước đây khi enable_viz:=false thì /image_raw im luôn ->
    // detector không bao giờ chạy, trong khi xe vẫn chạy với
    // traffic_light_decision = "NONE" tức coi như đèn xanh.
    // ======================================================================
    void on_raw() {
        if (!raw_pub_enabled_) return;

        const LaneOutput& lo = last_lane_;
        if (!has_cam_ || lo.raw.empty()) return;

        auto raw_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", lo.raw).toImageMsg();
        pub_raw_img_->publish(*raw_msg);
    }

    // ======================================================================
    // DBG TICK - kênh chẩn đoán cho GUI Qt.
    //
    // Chạy ở dbg_rate_hz (mặc định 10 Hz), tách khỏi cả control lẫn viz để
    // thêm/xoá kênh debug không đổi nhịp gửi lệnh ESP32.
    // ======================================================================
    void on_dbg() {
        if (!enable_dbg_) return;

        const LaneOutput& lo = last_lane_;
        const bool has_cam = has_cam_;
        const ESP32Feedback& esp_fb = last_esp_fb_;

        // --- Tuổi dữ liệu, tính một lần cho cả status lẫn quyết định ảnh ---
        const double t = now().seconds();
        const double lidar_age_ms = (t - last_lidar_time_.seconds()) * 1000.0;
        const double cam_age_ms = has_cam
            ? static_cast<double>(lo.age_ms)
            : (t - last_cam_time_.seconds()) * 1000.0;
        const bool lidar_stale =
            !last_lidar_.has_data ||
            lidar_age_ms > static_cast<double>(LIDAR_STALE_MS);
        const bool camera_stale = !has_cam || lo.stale;

        std_msgs::msg::String st;
        st.data = build_status_json(
            cam_fps_, lidar_fps_, last_lidar_, lidar_age_ms, lidar_stale,
            lo, has_cam, cam_age_ms, camera_stale,
            dev_final_, bypass_state_, obstacle_avoidance_,
            esp_fb, serial_ok_,
            traffic_light_decision_, turn_decision_);
        pub_dbg_status_->publish(st);

        // --- Ảnh ---
        // Chỉ gửi ảnh khi camera còn sống: gửi ảnh cũ kèm status báo stale còn
        // tệ hơn không gửi, vì GUI sẽ hiện "NO SIGNAL" và người vận hành
        // biết ngay camera đã chết thay vì nhìn một ảnh treo.
if (!has_cam || camera_stale) return;

        const builtin_interfaces::msg::Time stamp = now();
        publish_compressed(pub_dbg_raw_, lo.raw, stamp);

        if (!lo.vis.empty()) publish_compressed(pub_dbg_vis_, lo.vis, stamp);

        // Mask nhị phân ở 320x200: GUI hiển thị ở khung nhỏ nên để nguyên,
        // resize ở phía GUI (rẻ hơn nhiều so với encode ở kích thước lớn).
        if (!lo.mask.empty()) publish_compressed(pub_dbg_bin_, lo.mask, stamp);

        // ROI: cắt đúng dải hàng detector thật sự xử lý. Dùng roi_y0 do
        // CameraLane báo, không suy ra lại từ hằng số - nếu không, ảnh ROI
        // hiển thị sẽ lệch với ảnh detector thật sự chạy khi tốc độ đổi.
        if (!lo.raw.empty() && lo.roi_y0 > 0 && lo.roi_y0 < lo.raw.rows) {
            const cv::Rect r(0, lo.roi_y0, lo.raw.cols, lo.raw.rows - lo.roi_y0);
            publish_compressed(pub_dbg_roi_, lo.raw(r), stamp);
        }
    }

    // ------------------------------------------------------------------------
    // JSON status
    //
    // Dùng Json::FastWriter để không phải escape thủ công. Nhóm theo đúng
    // panel của GUI: lidar / lane / oa / esp / ai. -1 là "không có dữ liệu",
    // GUI tô đỏ ô đó thay vì hiện số 0 giả.
    // ------------------------------------------------------------------------
    static std::string num_opt(const std::optional<float>& v) {
        return v.has_value() ? std::to_string(*v) : std::string("-1");
    }

    // Không static: dùng now() của Node để đóng dấu thời gian lên ảnh.
    std::string build_status_json(
        double cam_fps, double lidar_fps,
        const LidarStatus& lidar, double lidar_age_ms, bool lidar_stale,
        const LaneOutput& lo, bool has_cam, double cam_age_ms, bool camera_stale,
        int16_t dev_final, BypassState state, const ObstacleAvoidance& oa,
        const ESP32Feedback& esp_fb, bool serial_ok,
        const std::string& traffic, const std::string& turn) {

        Json::Value root;

        root["lidar"]["ok"] = lidar.has_data && !lidar_stale;
        root["lidar"]["has_data"] = lidar.has_data;
        root["lidar"]["stale"] = lidar_stale;
        root["lidar"]["fps"] = lidar_fps;
        root["lidar"]["age_ms"] = lidar_age_ms;
        root["lidar"]["alert"] = lidar.alert;
        root["lidar"]["detail"] = lidar.detail;
        root["lidar"]["front"] = num_opt(lidar.front_min_cm);
        root["lidar"]["rear"] = num_opt(lidar.rear_bypass_min_cm);
        root["lidar"]["left"] = num_opt(lidar.left_min_cm);
        root["lidar"]["right"] = num_opt(lidar.right_min_cm);
        root["lidar"]["ob_front"] = num_opt(lidar.ob_front_cm);
        root["lidar"]["ob_left"] = num_opt(lidar.ob_left_cm);
        root["lidar"]["ob_right"] = num_opt(lidar.ob_right_cm);
        root["lidar"]["ob_left_rear"] = num_opt(lidar.ob_left_rear_cm);
        root["lidar"]["ob_right_rear"] = num_opt(lidar.ob_right_rear_cm);
        root["lidar"]["points"] = static_cast<Json::UInt>(lidar.points_px.size());

        root["lane"]["valid"] = has_cam && lo.valid && !camera_stale;
        root["lane"]["stale"] = camera_stale;
        root["lane"]["age_ms"] = cam_age_ms;
        root["lane"]["cmd"] = lo.camera_cmd;
        root["lane"]["dev"] = has_cam ? lo.dev_final_px : 0;
        root["lane"]["mode"] = static_cast<int>(lo.detector_mode);
        root["lane"]["dual"] = lo.is_dual_lane;
        root["lane"]["conf"] = std::string(lane_confidence_name(lo.confidence));
        root["lane"]["curve"] = lo.curve_angle_deg;
        root["lane"]["curvature"] = lo.curvature;
        root["lane"]["fps"] = cam_fps;
        root["lane"]["proc_ms"] = lo.processing_ms;
        root["lane"]["pts"] = static_cast<Json::UInt>(lo.pixels_used);
        root["lane"]["roi_y0"] = lo.roi_y0;
        root["lane"]["frame_id"] = static_cast<Json::UInt64>(lo.frame_id);
        root["lane"]["speed_x10"] = static_cast<int>(lo.target_speed_x10);
        root["lane"]["speed_kmh"] = static_cast<double>(lo.target_speed_x10) / 10.0;
        root["lane"]["color"] = lo.lane_color_name;

        root["oa"]["state"] = bypass_state_name(state);
        root["oa"]["dev"] = dev_final;
        root["oa"]["speed"] = static_cast<int>(oa.get_speed_command());
        // Tốc độ thực sự gửi đi, km/h. Giá trị /10 nhanh chóng khó đọc khi
        // so sánh với lane.speed_kmh và esp.v trên cùng dashboard.
        root["oa"]["speed_kmh"] = static_cast<double>(oa.get_speed_command()) / 10.0;
        root["oa"]["estop"] = (state == BypassState::EMERGENCY_STOP);
        root["oa"]["camera_stale"] = camera_stale;
        root["oa"]["lidar_stale"] = lidar_stale;
        root["oa"]["margin_cm"] = oa.get_speed_margin_cm();
        root["oa"]["speed_override"] = static_cast<int>(ObstacleAvoidance::get_speed_normal_override());

        root["esp"]["ok"] = serial_ok;
        root["esp"]["valid"] = esp_fb.valid;
        root["esp"]["v"] = esp_fb.velocity_kmh;
        // feedback_age_ms() trả (unsigned long)-1 khi CHƯA có gói telemetry nào.
        // Đổ thẳng ra JSON thành 1.8e19, GUI toDouble() đọc vào rồi hiện
        // "age = 18446744073709551615 ms". -1 = chưa có gói là giá trị mà GUI
        // đã quy ước là "không có dữ liệu".
        const unsigned long esp_age = serial_->feedback_age_ms();
        root["esp"]["age_ms"] = (esp_age == static_cast<unsigned long>(-1))
            ? -1.0
            : static_cast<double>(esp_age);

        root["ai"]["light"] = traffic;
        root["ai"]["turn"] = turn;

        Json::FastWriter w;
        return w.write(root);
    }

    // ------------------------------------------------------------------------
    // Encode JPEG rồi publish CompressedImage.
    //
    // Chất lượng 80: ảnh dashboard chỉ để người nhìn, không xử lý tiếp. Giảm
    // từ 95 xuống 80 cắt ~40% dung lượng mà mắt không thấy khác biệt.
    // ------------------------------------------------------------------------
    static void publish_compressed(
        const rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr& pub,
        const cv::Mat& img,
        const builtin_interfaces::msg::Time& stamp) {

        if (img.empty() || !pub) return;

        sensor_msgs::msg::CompressedImage m;
        m.format = "jpeg";

        // Ảnh 1 kênh (mask nhị phân) không encode được trực tiếp ra JPEG
        // có nghĩa màu - OpenCV sẽ cho ảnh 3 kênh nếu chuyển trước.
        const cv::Mat* src = &img;
        cv::Mat bgr_tmp;
        if (img.channels() == 1) {
            cv::cvtColor(img, bgr_tmp, cv::COLOR_GRAY2BGR);
            src = &bgr_tmp;
        }

        const std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 80};
        if (!cv::imencode(".jpg", *src, m.data, params)) return;

        m.header.stamp = stamp;
        pub->publish(m);
    }


    int cam_index_;
    int fps_;
    double cam_fps_, lidar_fps_;
    rclcpp::Time last_cam_time_, last_lidar_time_;

    // Kết quả tick điều khiển, viz chỉ đọc.
    bool enable_viz_ = true;
    bool raw_pub_enabled_ = false;
    LaneOutput last_lane_;
    bool has_cam_ = false;
    ESP32Feedback last_esp_fb_;
    int16_t dev_final_ = 0;

    std::unique_ptr<CameraLane> cam_;
    LidarModule lidar_;
    LidarStatus last_lidar_;
    std::unique_ptr<SerialESP32> serial_;
    bool serial_ok_;
    ObstacleAvoidance obstacle_avoidance_;
    BypassState bypass_state_;

    Json::Value last_signs_json_;
    cv::Mat last_signs_img_;
    rclcpp::Time last_signs_update_;

    std::string traffic_light_decision_; 
    std::string turn_decision_;          

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_signs_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_signs_img_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_traffic_light_;  
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_turn_detector_;   

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_raw_img_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr viz_timer_;
    rclcpp::TimerBase::SharedPtr raw_timer_;

    // --- Kênh chẩn đoán cho GUI ---
    bool enable_dbg_ = false;
    double dbg_rate_hz_ = 10.0;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_dbg_status_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_dbg_raw_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_dbg_vis_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_dbg_bin_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_dbg_roi_;
    rclcpp::TimerBase::SharedPtr dbg_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FusionVizNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
