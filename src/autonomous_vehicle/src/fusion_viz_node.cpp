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
#include <json/json.h>

#include "camera_lane.hpp"
#include "lidar_module.hpp"
#include "serial_esp32.hpp"
#include "obstacle_avoidance.hpp"

using std::placeholders::_1;
using namespace cv;
namespace {

constexpr int HEADER_H = 100;
constexpr int RIGHT_PANEL_W = 400;

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
    int x1 = 15, y1 = 25;
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
    int x2 = 300, y2 = 25;
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
    int x3 = 615, y3 = 25;
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

    snprintf(text, sizeof(text), "Dev: %d px", lane ? lane->dev_px : 0);
    cv::putText(full_img, text, {x3, y3}, cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(200,200,200), 1, cv::LINE_AA);

    int x4 = 865, y4 = 25;
    cv::putText(full_img, "ESP32", {x4, y4}, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 200, 100), 2, cv::LINE_AA);
    y4 += 23;
    snprintf(text, sizeof(text), "Status: %s", serial_ok ? "OK" : "FAIL");
    cv::putText(full_img, text, {x4, y4}, cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(200,200,200), 1, cv::LINE_AA);
    y4 += 22;
    snprintf(text, sizeof(text), "V: %.1f km/h", esp_fb.valid ? esp_fb.velocity_kmh : 0.0f);
    cv::putText(full_img, text, {x4, y4}, cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(200,200,200), 1, cv::LINE_AA);

    int x5 = 1050, y5 = 25;
    cv::putText(full_img, "AI DETECTION", {x5, y5}, 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 100, 255), 2, cv::LINE_AA);
    y5 += 23;

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
    
    snprintf(text, sizeof(text), "Light: %s", traffic_light_decision.c_str());
    cv::putText(full_img, text, {x5, y5}, 
                cv::FONT_HERSHEY_SIMPLEX, 0.55, traffic_color, 2, cv::LINE_AA);
    y5 += 22;

    cv::Scalar turn_color = (turn_decision == "NONE") ? 
        cv::Scalar(150, 150, 150) : cv::Scalar(0, 255, 255);
    
    snprintf(text, sizeof(text), "Turn: %s", turn_decision.c_str());
    cv::putText(full_img, text, {x5, y5}, 
                cv::FONT_HERSHEY_SIMPLEX, 0.55, turn_color, 1, cv::LINE_AA);
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
            int remaining = active_signs.size() - sign_count;
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
          last_frame_id_(-1),
          last_frame_time_(std::chrono::steady_clock::now()),
          last_cam_time_(now()),
          last_lidar_time_(now()),
          serial_ok_(false),
          bypass_state_(BypassState::NORMAL),
          last_signs_update_(now()),
          traffic_light_decision_("NONE"),   
          turn_decision_("NONE")             
    {
        cam_ = std::make_unique<CameraLane>(cam_index_, fps_, true);
        if (!cam_->start()) RCLCPP_WARN(get_logger(), "Failed to open camera");

        std::string serial_port = declare_parameter("serial_port", "/dev/ttyUSB0");
        serial_ = std::make_unique<SerialESP32>(serial_port);

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

        sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
            declare_parameter("scan_topic", "/scan"),
            rclcpp::SensorDataQoS(),
            std::bind(&FusionVizNode::on_scan, this, _1)
        );

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
        pub_raw_img_ = create_publisher<sensor_msgs::msg::Image>("/image_raw", 1);

        double hz = declare_parameter("viz_hz", 120.0);
        auto period_ns = std::chrono::nanoseconds(
            static_cast<int64_t>(1e9 / std::max(1.0, hz))
        );

        timer_ = create_wall_timer(period_ns, std::bind(&FusionVizNode::on_timer, this));
        RCLCPP_INFO(get_logger(), "FusionVizNode started with %.1f Hz timer", hz);
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

    void on_timer() {
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
        ESP32Feedback esp_fb = serial_->get_latest_feedback();
        float current_speed = esp_fb.valid ? esp_fb.velocity_kmh : 0.0f;
        LaneOutput lo;
        bool has_cam = (cam_ && cam_->get_latest(lo, current_speed));
        if (has_cam) {
            auto t = now();
            double dt = (t - last_cam_time_).seconds();
            if (dt > 0.001) {
                double inst = 1.0/dt;
                if (inst <= 250.0) cam_fps_ = 0.9 * cam_fps_ + 0.1 * inst;
            }
            last_cam_time_ = t;
        }
        Mat panel1 = full_img(Rect(map_w, HEADER_H, RIGHT_PANEL_W, 250));
        draw_camera_panel(panel1, has_cam ? lo.vis : cv::Mat());

        Mat panel2 = full_img(Rect(map_w, HEADER_H + 250, RIGHT_PANEL_W, 350));
        draw_ai_detection_panel(panel2, traffic_light_decision_, turn_decision_);
        int16_t deviation = has_cam ? lo.dev_px : 0;
        bool is_dual_lane = has_cam && (!lo.left.empty() && !lo.right.empty());
        float dominant_slope = has_cam ? lo.curve_angle_deg / 57.3f : 0.0f;
        auto bypass_cmd = obstacle_avoidance_.update(last_lidar_, deviation, dominant_slope, is_dual_lane, traffic_light_decision_, current_speed );
        bypass_state_ = bypass_cmd.state;
        int16_t dev_final = bypass_cmd.dev_final_px;
        if (serial_ok_ && serial_->is_open()) {
            SerialCommand scmd;
            scmd.dev_final_px = dev_final;
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
        draw_header(full_img, has_cam ? &lo : nullptr, last_lidar_,
                cam_fps_, lidar_fps_, serial_ok_, esp_fb,
                bypass_state_, dev_final,
                traffic_light_decision_, turn_decision_);
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", full_img).toImageMsg();
        pub_img_->publish(*msg);

        if (has_cam && !lo.raw.empty()) {
            auto raw_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", lo.raw).toImageMsg();
            pub_raw_img_->publish(*raw_msg);
        }
    }


    int cam_index_;
    int fps_;
    double cam_fps_, lidar_fps_;
    int last_frame_id_;
    std::chrono::steady_clock::time_point last_frame_time_;
    rclcpp::Time last_cam_time_, last_lidar_time_;

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
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FusionVizNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
