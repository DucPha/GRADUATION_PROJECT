#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "camera_lane.hpp"
#include "lidar_module.hpp"
#include "serial_esp32.hpp"
#include "obstacle_avoidance.hpp"

using std::placeholders::_1;
using namespace cv;

namespace {

// Độ phân giải chuẩn Full HD 1080p
constexpr int CANVAS_W = 1920;
constexpr int CANVAS_H = 1080;
constexpr int TOP_BAR_H = 65;
const std::string WINDOW_NAME = "AUTONOMOUS SYSTEM COCKPIT [ESC to Shutdown]";

namespace UITheme {
    const cv::Scalar BG          = cv::Scalar(243, 245, 249); // Nền xám khói dịu mắt
    const cv::Scalar CARD_BG     = cv::Scalar(255, 255, 255); // Trắng thẻ
    const cv::Scalar HEADER_BG   = cv::Scalar(233, 238, 245); // Header thẻ
    const cv::Scalar BORDER      = cv::Scalar(205, 212, 222); // Viền xám mảnh
    const cv::Scalar TITLE_TXT   = cv::Scalar(35, 45, 60);    // Chữ xám đậm
    const cv::Scalar BODY_TXT    = cv::Scalar(95, 105, 120);  // Chữ nhãn phụ
    const cv::Scalar BRAND_BLUE  = cv::Scalar(185, 95, 10);   // Xanh dương kỹ thuật MATLAB
    const cv::Scalar SUCCESS     = cv::Scalar(40, 160, 40);   // Xanh lá báo OK
    const cv::Scalar WARNING     = cv::Scalar(0, 140, 240);   // Cam cảnh báo
    const cv::Scalar DANGER      = cv::Scalar(30, 30, 220);   // Đỏ phanh/nguy hiểm
}

void draw_ui_card(cv::Mat& canvas, const cv::Rect& r, const std::string& title, const std::string& subtitle = "") {
    cv::rectangle(canvas, r, UITheme::CARD_BG, cv::FILLED);
    cv::rectangle(canvas, r, UITheme::BORDER, 1, cv::LINE_AA);

    cv::Rect header_rect(r.x, r.y, r.width, 38);
    cv::rectangle(canvas, header_rect, UITheme::HEADER_BG, cv::FILLED);
    cv::line(canvas, cv::Point(r.x, r.y + 38), cv::Point(r.x + r.width, r.y + 38), UITheme::BORDER, 1, cv::LINE_AA);

    cv::putText(canvas, title, cv::Point(r.x + 16, r.y + 25),
                cv::FONT_HERSHEY_DUPLEX, 0.62, UITheme::TITLE_TXT, 1, cv::LINE_AA);

    if (!subtitle.empty()) {
        cv::putText(canvas, subtitle, cv::Point(r.x + r.width - 150, r.y + 25),
                    cv::FONT_HERSHEY_SIMPLEX, 0.48, UITheme::BODY_TXT, 1, cv::LINE_AA);
    }
}

void render_subframe(cv::Mat& target_roi, const cv::Mat& src_img, const std::string& placeholder) {
    if (src_img.empty() || src_img.cols == 0 || src_img.rows == 0) {
        cv::putText(target_roi, placeholder, 
                    cv::Point(target_roi.cols / 2 - 90, target_roi.rows / 2),
                    cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(160, 165, 175), 1, cv::LINE_AA);
        return;
    }

    cv::Mat bgr;
    if (src_img.channels() == 1) {
        cv::cvtColor(src_img, bgr, cv::COLOR_GRAY2BGR);
    } else {
        bgr = src_img;
    }

    double scale = std::min(static_cast<double>(target_roi.cols) / bgr.cols,
                            static_cast<double>(target_roi.rows) / bgr.rows);
    int nw = std::clamp(static_cast<int>(bgr.cols * scale), 1, target_roi.cols);
    int nh = std::clamp(static_cast<int>(bgr.rows * scale), 1, target_roi.rows);

    cv::Mat resized;
    cv::resize(bgr, resized, cv::Size(nw, nh));

    int ox = (target_roi.cols - nw) / 2;
    int oy = (target_roi.rows - nh) / 2;
    resized.copyTo(target_roi(cv::Rect(ox, oy, nw, nh)));
}

// Radar LiDAR nền đen tuyền kỹ thuật cao
void render_polar_lidar(cv::Mat& target_roi, const LidarStatus& lidar) {
    // 1. Phủ nền đen than
    target_roi.setTo(cv::Scalar(16, 20, 24));

    cv::Point center(target_roi.cols / 2, target_roi.rows / 2 + 15);

    // 2. Lưới cự ly phân tầng (Xanh rêu dạ quang mờ)
    const int radii[] = {60, 120, 180, 230};
    const char* labels[] = {"0.5m", "1.0m", "1.5m", "2.0m"};
    for (int i = 0; i < 4; ++i) {
        cv::circle(target_roi, center, radii[i], cv::Scalar(38, 55, 45), 1, cv::LINE_AA);
        cv::putText(target_roi, labels[i], cv::Point(center.x + radii[i] - 34, center.y - 6),
                    cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(70, 110, 85), 1, cv::LINE_AA);
    }

    // Trục chữ thập
    cv::line(target_roi, cv::Point(center.x, 20), cv::Point(center.x, target_roi.rows - 20),
             cv::Scalar(35, 50, 42), 1, cv::LINE_AA);
    cv::line(target_roi, cv::Point(20, center.y), cv::Point(target_roi.cols - 20, center.y),
             cv::Scalar(35, 50, 42), 1, cv::LINE_AA);

    // Mũi tên định hướng đầu xe (Chỉ thẳng lên trên)
    cv::line(target_roi, center, cv::Point(center.x, center.y - 25), cv::Scalar(0, 230, 255), 2, cv::LINE_AA);
    cv::circle(target_roi, center, 7, cv::Scalar(0, 230, 255), cv::FILLED, cv::LINE_AA);

    if (!lidar.has_data) {
        cv::putText(target_roi, "NO LIDAR DATA (/scan)", cv::Point(center.x - 110, center.y - 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.58, cv::Scalar(0, 0, 240), 1, cv::LINE_AA);
        return;
    }

    // 3. Vẽ điểm chướng ngại vật (Đảo dấu Y để phía trước xe vẽ ở nửa trên)
    cv::Point O_orig = LidarModule::origin();
    float scale = 0.75f; // Scale lớn phù hợp khung 560x450
    for (const auto& p : lidar.points_px) {
        int px = center.x + static_cast<int>((p.x - O_orig.x) * scale);
        // Đảo chiều Y: (p.y - O_orig.y) < 0 là phía trước -> py = center.y + (...)
        int py = center.y + static_cast<int>((p.y - O_orig.y) * scale);

        if (px >= 2 && px < target_roi.cols - 2 && py >= 2 && py < target_roi.rows - 2) {
            // Điểm đỏ cam neon nổi bật trên nền đen
            cv::circle(target_roi, cv::Point(px, py), 2, cv::Scalar(48, 59, 255), cv::FILLED, cv::LINE_AA);
        }
    }
}

} // namespace

class FusionVizNode : public rclcpp::Node {
public:
    FusionVizNode()
        : Node("fusion_viz_node"),
          // -1 = tự dò /dev/video*. Camera USB này hay rớt khỏi bus rồi cắm
          // lại với index khác (video0 -> video1), nên chỉ định cứng sẽ hỏng.
          cam_index_(declare_parameter("cam_index", -1)),
          fps_(declare_parameter("camera_fps", 30)),
          cam_fps_(0.0), lidar_fps_(0.0),
          last_frame_id_(-1),
          last_cam_time_(now()),
          last_lidar_time_(now()),
          last_cam_progress_time_(now()),
          last_raw_pub_time_(rclcpp::Time(0, 0, RCL_ROS_TIME)),
          serial_ok_(false),
          last_serial_retry_(rclcpp::Time(0, 0, RCL_ROS_TIME)),
          bypass_state_(BypassState::NORMAL),
          traffic_light_time_(now()),
          turn_time_(now()),
          traffic_light_decision_("NONE"),
          turn_decision_("NONE")
    {
        cam_ = std::make_unique<CameraLane>(
            cam_index_,
            fps_,
            true   // V4L2
            // auto_exposure: camera này không có control exposure nào nên
            // giá trị không ảnh hưởng; giữ auto vì đó là trạng thái mặc định
            // và là trạng thái cho FPS cao nhất (đo được 63 ở 352x288).
        );
        if (!cam_->start()) {
            RCLCPP_ERROR(get_logger(),
                         "Camera open failed: %s",
                         cam_->last_error().c_str());
        }

        serial_port_ = declare_parameter("serial_port", "/dev/ttyUSB0");
        serial_ = std::make_unique<SerialESP32>(serial_port_);
        serial_->set_baudrate(declare_parameter("baudrate", SerialESP32::BAUDRATE));

        RCLCPP_INFO(get_logger(), "Waiting 2s for ESP32 to boot...");
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        int retry_count = 0;
        std::vector<std::string> ports_to_try = {serial_port_};
        // Thử cả /dev/ttyACM0 nếu port mặc định là USB0 (ESP32 thường dùng ACM)
        if (serial_port_ == "/dev/ttyUSB0") {
            ports_to_try.push_back("/dev/ttyACM0");
        }
        
        while (!serial_ok_ && retry_count < 5) {
            for (const auto& port : ports_to_try) {
                if (port != serial_port_) {
                    serial_ = std::make_unique<SerialESP32>(port);
                    serial_->set_baudrate(declare_parameter("baudrate", SerialESP32::BAUDRATE));
                }
                RCLCPP_INFO(get_logger(), "Opening serial port %s (%d/5)...",
                    port.c_str(), retry_count + 1);
                serial_ok_ = serial_->open();
                if (serial_ok_) {
                    serial_port_ = port;  // Cập nhật port thành công
                    break;
                }
                RCLCPP_WARN(get_logger(), "Failed on %s, trying next...", port.c_str());
            }
            if (!serial_ok_) {
                RCLCPP_WARN(get_logger(), "All ports failed, retrying in 1s...");
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                retry_count++;
            }
        }

        if (!serial_ok_) {
            RCLCPP_ERROR(get_logger(),
                "Failed to open serial after 5 retries; will keep retrying every %.0fs",
                SERIAL_RETRY_PERIOD_S);
        } else {
            RCLCPP_INFO(get_logger(), "Serial port opened successfully");
        }

        sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
            declare_parameter("scan_topic", "/scan"),
            rclcpp::SensorDataQoS(),
            std::bind(&FusionVizNode::on_scan, this, _1)
        );

        sub_traffic_light_ = create_subscription<std_msgs::msg::String>(
            declare_parameter("traffic_light_topic", "/traffic_light/decision"),
            rclcpp::QoS(10),
            std::bind(&FusionVizNode::on_traffic_light, this, _1)
        );

        sub_turn_detector_ = create_subscription<std_msgs::msg::String>(
            declare_parameter("turn_detector_topic", "/turn_detector/decision"),
            rclcpp::QoS(10),
            std::bind(&FusionVizNode::on_turn_detector, this, _1)
        );

        RCLCPP_INFO(get_logger(), "AI Detection subscribers initialized");

        pub_img_ = create_publisher<sensor_msgs::msg::Image>(
            declare_parameter("image_topic", "/fusion_viz/image"), 1);
        pub_raw_img_ = create_publisher<sensor_msgs::msg::Image>(
            declare_parameter("raw_image_topic", "/image_raw"), 1);

        ai_timeout_s_ = declare_parameter("ai_timeout_s", 2.0);
        raw_image_hz_ = declare_parameter("raw_image_hz", 5.0);
        camera_stale_s_ = declare_parameter("camera_stale_timeout_s", 1.0);
        lidar_stale_s_ = declare_parameter("lidar_stale_timeout_s", 0.5);
        turn_blend_ = declare_parameter("turn_blend_px", 30);
        turn_speed_x10_ = static_cast<uint8_t>(declare_parameter("turn_speed_x10", 40));
        lidar_.set_mount_offset_deg(
            static_cast<float>(declare_parameter("lidar_mount_offset_deg",
                static_cast<double>(LidarModule::DEFAULT_MOUNT_OFFSET_DEG))));

        // Cửa sổ kéo thả tự do
        cv::namedWindow(WINDOW_NAME, cv::WINDOW_NORMAL);
        cv::resizeWindow(WINDOW_NAME, 1600, 900); // Kích thước mở đầu mặc định vừa vặn

        double hz = declare_parameter("viz_hz", 120.0);
        auto period_ns = std::chrono::nanoseconds(
            static_cast<int64_t>(1e9 / std::max(1.0, hz))
        );

        timer_ = create_wall_timer(period_ns, std::bind(&FusionVizNode::on_timer, this));
        RCLCPP_INFO(get_logger(), "FusionVizNode started with %.1f Hz timer", hz);
    }

    ~FusionVizNode() override {
        shutdown_hardware();
    }

    void shutdown_hardware() {
        if (serial_ && serial_->is_open()) {
            SerialCommand stop_cmd;
            stop_cmd.dev_final_px = 0;
            stop_cmd.speed_control = 0;
            stop_cmd.emergency_stop = true;
            serial_->send_command(stop_cmd);
            serial_->close();
        }
        if (cam_) cam_->stop();
        cv::destroyAllWindows();
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
        lidar_.update(*msg, last_lidar_);
    }

    void on_traffic_light(const std_msgs::msg::String::SharedPtr msg) {
        traffic_light_time_ = now();
        traffic_light_decision_ = msg->data;
    }

    void on_turn_detector(const std_msgs::msg::String::SharedPtr msg) {
        turn_decision_ = msg->data;
        turn_time_ = now();
    }

    void on_timer() {
        const auto t_now = now();

        // 1. Watchdog AI
        if ((t_now - traffic_light_time_).seconds() > ai_timeout_s_) {
            traffic_light_decision_ = "NONE";
        }
        if ((t_now - turn_time_).seconds() > ai_timeout_s_) {
            turn_decision_ = "NONE";
        }

        // 2. Feedback ESP32
        ESP32Feedback esp_fb = serial_->get_latest_feedback();
        if (esp_fb.is_stale(100)) {
            esp_fb.valid = false;
        }
        float current_speed = esp_fb.valid ? esp_fb.velocity_kmh : 0.0f;
        if (!std::isfinite(current_speed)) current_speed = 0.0f;

        // 3. Lane Detection
        LaneOutput lo;
        bool has_cam = (cam_ && cam_->get_latest(lo, current_speed));
        if (has_cam) {
            if (last_frame_id_ >= 0 && lo.frame_id > static_cast<uint64_t>(last_frame_id_)) {
                auto t = now();
                double dt = (t - last_cam_time_).seconds();
                if (dt > 0.001) {
                    double inst = 1.0 / dt;
                    if (inst <= 250.0) cam_fps_ = 0.9 * cam_fps_ + 0.1 * inst;
                }
                last_cam_time_ = t;
            }
            last_frame_id_ = static_cast<int>(lo.frame_id);
        }

        // Watchdogs
        const double cam_stale_s = (t_now - last_cam_progress_time_).seconds();
        if (has_cam && lo.frame_id != last_seen_frame_id_) {
            last_seen_frame_id_ = lo.frame_id;
            last_cam_progress_time_ = t_now;
        }
        const bool cam_stale = has_cam && (cam_stale_s > camera_stale_s_);
        const double lidar_stale_s = (t_now - last_lidar_time_).seconds();
        const bool lidar_stale = lidar_stale_s > lidar_stale_s_;

        if (serial_ && serial_->is_open() && serial_->link_down()) {
            serial_->close();
            serial_ok_ = false;
        } else if (serial_ && !serial_->is_open() &&
            (t_now - last_serial_retry_).seconds() > SERIAL_RETRY_PERIOD_S) {
            last_serial_retry_ = t_now;
            serial_->close();
            serial_ok_ = serial_->open();
        }
        const bool serial_link_ok = serial_ && serial_->is_open() && !serial_->link_down();

        // 4. Quyết định điều khiển
        const bool cam_valid = has_cam && lo.valid && !cam_stale;
        const bool sensors_ok = cam_valid && !lidar_stale && serial_link_ok;
        int16_t deviation = cam_valid ? lo.dev_final_px : 0;
        bool is_dual_lane = cam_valid && !lo.left.empty() && !lo.right.empty();
        float dominant_slope = cam_valid
            ? std::tan(lo.curve_angle_deg * static_cast<float>(CV_PI) / 180.0f)
            : 0.0f;

        int16_t turn_blend_px = 0;
        if (turn_decision_ == "TURN_LEFT")       turn_blend_px = -turn_blend_;
        else if (turn_decision_ == "TURN_RIGHT") turn_blend_px = +turn_blend_;

        auto bypass_cmd = obstacle_avoidance_.update(last_lidar_, deviation,
            dominant_slope, is_dual_lane,
            traffic_light_decision_, current_speed);
        bypass_state_ = bypass_cmd.state;
        int16_t dev_final = bypass_cmd.dev_final_px;
        bool emergency_stop = bypass_cmd.emergency_stop;
        uint8_t speed_control = bypass_cmd.speed_control;

        if (turn_blend_px != 0 && !emergency_stop) {
            dev_final = ObstacleAvoidance::clamp_px(
                static_cast<int32_t>(dev_final) + turn_blend_px);
            speed_control = std::min(speed_control, turn_speed_x10_);
        }

        if (cam_valid && !emergency_stop && lo.target_speed_x10 > 0) {
            speed_control = std::min(lo.target_speed_x10, speed_control);
        }

        if (emergency_stop || !sensors_ok) {
            speed_control = 0;
        }

        if (serial_link_ok) {
            SerialCommand scmd;
            scmd.dev_final_px = dev_final; // Luôn gửi góc lệch để servo xoay
            scmd.speed_control = speed_control;
            scmd.emergency_stop = emergency_stop; // Chỉ ngắt khi phím ESC hoặc có vật cản
            serial_->send_command(scmd);
        }

        // =========================================================================
        // 5. VẼ DASHBOARD FULL HD 1920x1080 (MATLAB High-Res Instrumentation)
        // =========================================================================
        cv::Mat ui(CANVAS_H, CANVAS_W, CV_8UC3, UITheme::BG);

        // TOP HEADER BAR
        cv::rectangle(ui, cv::Rect(0, 0, CANVAS_W, TOP_BAR_H), UITheme::CARD_BG, cv::FILLED);
        cv::line(ui, cv::Point(0, TOP_BAR_H), cv::Point(CANVAS_W, TOP_BAR_H), UITheme::BORDER, 1, cv::LINE_AA);

        cv::circle(ui, cv::Point(35, 32), 10, UITheme::BRAND_BLUE, cv::FILLED, cv::LINE_AA);
        cv::putText(ui, "AUTONOMOUS VEHICLE COCKPIT & SENSOR FUSION SYSTEM", cv::Point(55, 40),
                    cv::FONT_HERSHEY_DUPLEX, 0.78, UITheme::TITLE_TXT, 1, cv::LINE_AA);
        cv::putText(ui, "|   Press [ESC] to Emergency Stop & Exit", cv::Point(780, 40),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, UITheme::DANGER, 1, cv::LINE_AA);

        auto draw_badge = [&](int x, const std::string& label, bool ok) {
            cv::Scalar color = ok ? UITheme::SUCCESS : UITheme::DANGER;
            cv::Rect b(x, 17, 130, 30);
            cv::rectangle(ui, b, UITheme::HEADER_BG, cv::FILLED);
            cv::rectangle(ui, b, UITheme::BORDER, 1, cv::LINE_AA);
            cv::circle(ui, cv::Point(x + 16, 32), 5, color, cv::FILLED, cv::LINE_AA);
            cv::putText(ui, label, cv::Point(x + 28, 38),
                        cv::FONT_HERSHEY_DUPLEX, 0.50, UITheme::TITLE_TXT, 1, cv::LINE_AA);
        };

        draw_badge(CANVAS_W - 460, "ESP32-S3", serial_link_ok);
        draw_badge(CANVAS_W - 310, "RPLIDAR", !lidar_stale);
        draw_badge(CANVAS_W - 160, "HD CAMERA", cam_valid);

        // ĐỊNH VỊ 5 Ô CARD ĐỒ HỌA KÍCH THƯỚC LỚN
        // Cột 1 & 2: Mỗi ô rộng 565px, cao 470px
        cv::Rect card_cam(20, 85, 565, 470);
        cv::Rect card_roi(20, 580, 565, 470);
        cv::Rect card_bin(605, 85, 565, 470);
        cv::Rect card_lidar(605, 580, 565, 470);
        // Cột 3: Telemetry Panel rộng 710px bao quát toàn bộ chiều cao
        cv::Rect card_telemetry(1190, 85, 710, 965);

        char fps_str[48];
        // Hiển thị cả FPS đo được lẫn độ phân giải thực tế, vì camera hay tự
        // hạ độ phân giải khi băng thông USB bận — không có thông tin này thì
        // rất dễ tưởng pipeline bị chậm do code.
        snprintf(fps_str, sizeof(fps_str), "%.1f FPS / %dx%d",
                 cam_fps_, cam_->actual_width(), cam_->actual_height());
        draw_ui_card(ui, card_cam, "1. Camera Input + Lane Overlay", fps_str);
        draw_ui_card(ui, card_roi, "2. Bird's-Eye View (IPM)", "Warp");
        draw_ui_card(ui, card_bin, "3. Black Lane Segmentation", "Filter");

        snprintf(fps_str, sizeof(fps_str), "%.1f Hz", lidar_fps_);
        draw_ui_card(ui, card_lidar, "4. 2D Polar Obstacle Radar", fps_str);
        draw_ui_card(ui, card_telemetry, "5. Telemetry & Actuation Diagnostics", "100Hz PID");

        // Nhúng các frame camera
        cv::Rect in_cam(card_cam.x + 2, card_cam.y + 40, card_cam.width - 4, card_cam.height - 42);
        cv::Rect in_roi(card_roi.x + 2, card_roi.y + 40, card_roi.width - 4, card_roi.height - 42);
        cv::Rect in_bin(card_bin.x + 2, card_bin.y + 40, card_bin.width - 4, card_bin.height - 42);
        cv::Rect in_lidar(card_lidar.x + 2, card_lidar.y + 40, card_lidar.width - 4, card_lidar.height - 42);

        cv::Mat target_cam = ui(in_cam);
        cv::Mat target_roi = ui(in_roi);
        cv::Mat target_bin = ui(in_bin);
        cv::Mat target_lidar = ui(in_lidar);

        // Ô 1: ảnh gốc + overlay vạch/đường tâm (lo.vis). Đây là cái "tài sĩ
        // lái xe" cần xem: vạch đỏ, tâm xanh, thang IPM, đường nắp xe.
        render_subframe(target_cam,
                        (has_cam && !lo.vis.empty()) ? lo.vis :
                        ((has_cam && !lo.raw.empty()) ? lo.raw : cv::Mat()),
                        "No Raw Stream");

        // Ô 2: bird's-eye THẬT của mask (lo.bird_mask), không phải ảnh gốc có
        // overlay. Trước đây ô này vẽ lo.vis (chính là ảnh thô + đường vạch)
        // nên nhìn "đúng" nhưng không kiểm chứng được phép chiếu IPM có đúng
        // hay không - mà đó mới là thứ quyết định xe có giữ được làn hay không.
        render_subframe(target_roi, (has_cam && !lo.bird_mask.empty()) ? lo.bird_mask : cv::Mat(),
                        "No IPM Result");

        // Ô 3: mask đã lọc blob + mask nắp xe (đúng thứ dùng để lái).
        render_subframe(target_bin, (has_cam && !lo.binary_mask.empty()) ? lo.binary_mask : cv::Mat(),
                        "No Segmented Lane");

        render_polar_lidar(target_lidar, last_lidar_);

        // BẢNG THÔNG SỐ TELEMETRY BÊN PHẢI (Khoảng cách dòng thoáng)
        int tx = card_telemetry.x + 35;
        int ty = card_telemetry.y + 80;
        const int row_step = 42;

        auto print_telemetry_row = [&](const std::string& label, const std::string& val, cv::Scalar val_col) {
            cv::putText(ui, label, cv::Point(tx, ty),
                        cv::FONT_HERSHEY_SIMPLEX, 0.58, UITheme::BODY_TXT, 1, cv::LINE_AA);
            cv::putText(ui, val, cv::Point(tx + 380, ty),
                        cv::FONT_HERSHEY_DUPLEX, 0.68, val_col, 1, cv::LINE_AA);
            cv::line(ui, cv::Point(tx, ty + 12), cv::Point(tx + 640, ty + 12), cv::Scalar(238, 242, 246), 1);
            ty += row_step;
        };

        // GROUP 1: STEERING & ACTUATION
        cv::putText(ui, "[ STEERING & TRAJECTORY ]", cv::Point(tx, ty - 15),
                    cv::FONT_HERSHEY_DUPLEX, 0.60, UITheme::BRAND_BLUE, 1, cv::LINE_AA);
        ty += 15;

        char buf[64];
        snprintf(buf, sizeof(buf), "%d px", dev_final);
        print_telemetry_row("Final Steering Dev:", buf, UITheme::BRAND_BLUE);

        snprintf(buf, sizeof(buf), "%d px", has_cam ? lo.dev_final_px : 0);
        print_telemetry_row("Lane Deviation (Raw):", buf, UITheme::TITLE_TXT);

        // Trạng thái detector: IPM / scanline, số vạch tìm được, và việc đang
        // "giữ lái" bằng kết quả cũ vì camera mất vạch. Nhờ đó biết ngay lúc
        // nào xe đang chạy mà không thấy làn.
        char det_buf[64];
        const char* mode_str = !has_cam ? "NO DATA"
                             : (lo.detector_mode == 1 ? "IPM" : "SCANLINE");
        snprintf(det_buf, sizeof(det_buf), "%s | pts=%d | lane_w=%.0fpx",
                 mode_str, has_cam ? lo.pixels_used : 0,
                 has_cam ? lo.lane_width_est_px : 0.0f);
        print_telemetry_row("Lane Detector:", det_buf,
                            cam_valid ? UITheme::TITLE_TXT : UITheme::DANGER);

        const int lost = has_cam ? lo.held_frames : 0;
        if (lost > 0) {
            snprintf(det_buf, sizeof(det_buf), "HOLDING LANE %d/%d", lost,
                     static_cast<int>(CameraLane::MAX_LOST_FRAMES));
            print_telemetry_row("Lane Hold:", det_buf, UITheme::WARNING);
        } else {
            print_telemetry_row("Lane Hold:", "OFF (tracking)", UITheme::SUCCESS);
        }

        snprintf(buf, sizeof(buf), "%+.1f deg", has_cam ? lo.curve_angle_deg : 0.0f);
        print_telemetry_row("Curvature Angle:", buf, UITheme::TITLE_TXT);

        snprintf(buf, sizeof(buf), "%.1f km/h", current_speed);
        print_telemetry_row("Velocity (ESP32 Hall):", buf, UITheme::SUCCESS);

        snprintf(buf, sizeof(buf), "%.1f km/h", static_cast<float>(speed_control) / 10.0f);
        print_telemetry_row("Speed Setpoint:", buf, UITheme::SUCCESS);

        // GROUP 2: SAFETY & OBSTACLES
        ty += 25;
        cv::putText(ui, "[ OBSTACLE & SAFETY CLEARANCE ]", cv::Point(tx, ty - 15),
                    cv::FONT_HERSHEY_DUPLEX, 0.60, UITheme::BRAND_BLUE, 1, cv::LINE_AA);
        ty += 15;

        auto fmt_cm = [](std::optional<float> d) -> std::string {
            if (!d) return "---";
            if (*d >= 199.5f) return ">200 cm";
            return std::to_string(static_cast<int>(*d)) + " cm";
        };

        float f_dist = last_lidar_.front_min_cm.value_or(999.0f);
        print_telemetry_row("Front Clearance:", fmt_cm(last_lidar_.front_min_cm),
                            f_dist < 45.0f ? UITheme::DANGER : UITheme::SUCCESS);

        snprintf(buf, sizeof(buf), "%s | %s", fmt_cm(last_lidar_.left_min_cm).c_str(), fmt_cm(last_lidar_.right_min_cm).c_str());
        print_telemetry_row("Side Clearance (L | R):", buf, UITheme::TITLE_TXT);

        std::string bypass_str = "NORMAL PATH";
        cv::Scalar bypass_col = UITheme::SUCCESS;
        if (bypass_state_ == BypassState::EMERGENCY_STOP) {
            bypass_str = "EMERGENCY STOP"; bypass_col = UITheme::DANGER;
        } else if (bypass_state_ == BypassState::SLOW_DOWN) {
            bypass_str = "SLOWING DOWN"; bypass_col = UITheme::WARNING;
        } else if (bypass_state_ != BypassState::NORMAL) {
            bypass_str = "BYPASSING OBSTACLE"; bypass_col = UITheme::WARNING;
        }
        print_telemetry_row("Maneuver State:", bypass_str, bypass_col);

        // GROUP 3: AI INTELLIGENCE
        ty += 25;
        cv::putText(ui, "[ VISION & AI DETECTIONS ]", cv::Point(tx, ty - 15),
                    cv::FONT_HERSHEY_DUPLEX, 0.60, UITheme::BRAND_BLUE, 1, cv::LINE_AA);
        ty += 15;

        cv::Scalar tl_col = UITheme::BODY_TXT;
        if (traffic_light_decision_ == "RED" || traffic_light_decision_ == "STOP") tl_col = UITheme::DANGER;
        else if (traffic_light_decision_ == "GREEN") tl_col = UITheme::SUCCESS;
        else if (traffic_light_decision_ == "YELLOW") tl_col = UITheme::WARNING;

        print_telemetry_row("Traffic Sign:", traffic_light_decision_, tl_col);
        print_telemetry_row("Navigation Arrow:", turn_decision_, 
                            turn_decision_ != "NONE" ? cv::Scalar(180, 0, 180) : UITheme::BODY_TXT);

        // Xuất bản topic ROS 2
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", ui).toImageMsg();
        pub_img_->publish(*msg);

        // Phát /image_raw cho các node AI
        if (has_cam && !lo.raw.empty() && raw_image_hz_ > 0.0) {
            const double raw_period_s = 1.0 / raw_image_hz_;
            if ((t_now - last_raw_pub_time_).seconds() >= raw_period_s) {
                last_raw_pub_time_ = t_now;
                auto raw_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", lo.raw).toImageMsg();
                pub_raw_img_->publish(*raw_msg);
            }
        }

        // =========================================================================
        // 6. HIỂN THỊ CỬA SỔ & BẮT PHÍM ESC ĐỂ THOÁT
        // =========================================================================
        cv::imshow(WINDOW_NAME, ui);
        int key = cv::waitKey(1);
        if (key == 27) { // 27 = Phím ESC
            RCLCPP_WARN(get_logger(), "ESC Key Pressed! Executing Emergency Stop and Terminating...");
            shutdown_hardware();
            rclcpp::shutdown();
        }
    }

    int cam_index_;
    int fps_;
    double cam_fps_, lidar_fps_;
    int last_frame_id_;
    rclcpp::Time last_cam_time_, last_lidar_time_;

    std::unique_ptr<CameraLane> cam_;
    LidarModule lidar_;
    LidarStatus last_lidar_;
    std::unique_ptr<SerialESP32> serial_;
    std::string serial_port_;
    bool serial_ok_;
    static constexpr double SERIAL_RETRY_PERIOD_S = 3.0;
    rclcpp::Time last_serial_retry_;
    ObstacleAvoidance obstacle_avoidance_;
    BypassState bypass_state_;

    std::string traffic_light_decision_;
    std::string turn_decision_;

    double ai_timeout_s_;
    double raw_image_hz_;
    double camera_stale_s_;
    double lidar_stale_s_;
    int16_t turn_blend_;
    uint8_t turn_speed_x10_;

    uint64_t last_seen_frame_id_ = UINT64_MAX;
    rclcpp::Time last_cam_progress_time_;
    rclcpp::Time last_raw_pub_time_;

    rclcpp::Time traffic_light_time_;
    rclcpp::Time turn_time_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_traffic_light_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_turn_detector_;   

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_raw_img_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FusionVizNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}