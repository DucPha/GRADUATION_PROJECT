// ============================================================================
// FUSION NODE - node dieu khien duy nhat cua xe
// ----------------------------------------------------------------------------
// Chuoi: CameraLane (nhanh 2 lan) -> SerialESP32 -> ESP32-S3 qua USB-OTG.
//
// Phase 1 chi dung camera de dieu khien. Lidar van subscribe /scan de quan sat
// nhung KHONG dua ket qua cua no vao quyet dinh lai xe: ObstacleAvoidance co
// co emergency stop khi mat du lieu, dung cho giai doan sau.
//
// Dieu kien dung chan QUAN TRONG: firmware khep PWM bang [95,180], nen
// speed = 0 se bi day len 95 (~1.55 km/h) va xe tu bo. Tinh hieu dung that su
// de dung xe la co emg = 1, khong phai speed = 0.
// ============================================================================

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>

#include <opencv2/core/mat.hpp>

#include "camera_node.hpp"
#include "esp32s3_node.hpp"
#include "lidar_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <optional>
#include <string>

using namespace std::chrono_literals;

namespace {

std::chrono::nanoseconds period_from_hz(double hz) {
    const double safe_hz = (hz > 1.0) ? hz : 1.0;
    return std::chrono::nanoseconds(static_cast<int64_t>(1e9 / safe_hz));
}

}   // namespace

class FusionNode : public rclcpp::Node {

public:

    FusionNode()
        : rclcpp::Node("fusion_node") {

        // ---- Tham so ----
        const auto serial_port = declare_parameter<std::string>("serial_port", "");
        const auto camera_index = declare_parameter<int>("camera_index", -1);
        const auto camera_fps = declare_parameter<int>("camera_fps", 30);
        // CameraLane khong con hieu ROI_TOP_FRAC; mac dinh lay tu CameraProfile (0.58)
        const auto roi_top_frac = declare_parameter<double>(
            "roi_top_frac", CameraProfile{}.roi_top_frac);
        const auto speed_x10 = declare_parameter<int>("speed_x10", 40);
        const auto speed_hold_x10 = declare_parameter<int>("speed_hold_x10", 20);
        const auto dev_sign = declare_parameter<int>("dev_sign", 1);
        const auto lane_lost_stop_ms = declare_parameter<int>("lane_lost_stop_ms", 400);
        const auto control_hz = declare_parameter<int>("control_hz", 100);
        const auto viz_hz = declare_parameter<int>("viz_hz", 10);
        const auto enable_viz = declare_parameter<bool>("enable_viz", true);
        const auto enable_lidar = declare_parameter<bool>("enable_lidar", true);
        const auto lidar_mount_offset_deg = declare_parameter<double>(
            "lidar_mount_offset_deg", LidarModule::DEFAULT_MOUNT_OFFSET_DEG
        );
        const auto log_level = declare_parameter<std::string>("log_level", "info");

        speed_x10_ = speed_x10;
        speed_hold_x10_ = speed_hold_x10;
        lane_lost_stop_ms_ = lane_lost_stop_ms;
        dev_sign_ = (dev_sign < 0) ? -1 : 1;

        if (log_level == "debug") {
            get_logger().set_level(rclcpp::Logger::Level::Debug);
        } else if (log_level == "warn") {
            get_logger().set_level(rclcpp::Logger::Level::Warn);
        }

        // ---- Camera ----
        camera_ = std::make_unique<CameraLane>(camera_index, camera_fps);
        camera_->set_roi_top_frac(static_cast<float>(roi_top_frac));

        if (camera_->start()) {
            RCLCPP_INFO(get_logger(), "Camera started");
        } else {
            RCLCPP_ERROR(
                get_logger(),
                "Camera failed to start, xe se khong chay. "
                "Kiem tra /dev/video* va quyen truy cap (quyen: sudo usermod -aG video $USER)."
            );
        }

        // ---- Serial ----
        serial_ = std::make_unique<SerialESP32>(serial_port);
        if (serial_->open()) {
            RCLCPP_INFO(get_logger(), "Serial opened");
        } else {
            RCLCPP_ERROR(get_logger(), "serial FAIL, khong gui duoc lenh xuong ESP32");
        }

        // ---- Topic ----
        status_pub_ = create_publisher<std_msgs::msg::String>("/lane/status", 10);

        if (enable_viz) {
            vis_pub_ = create_publisher<sensor_msgs::msg::Image>("/lane/vis", 2);
        }

        if (enable_lidar) {
            lidar_.set_mount_offset_deg(static_cast<float>(lidar_mount_offset_deg));
            lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan", 10,
                [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
                    lidar_.update(*msg, lidar_status_);
                    lidar_seen_at_ = std::chrono::steady_clock::now();
                    lidar_has_data_ = lidar_status_.has_data;
                }
            );
        }

        // ---- Timer ----
        control_timer_ = create_wall_timer(
            period_from_hz(control_hz), [this]() { control_tick(); }
        );

        if (enable_viz) {
            viz_timer_ = create_wall_timer(
                period_from_hz(viz_hz), [this]() { viz_tick(); }
            );
        }

        status_timer_ = create_wall_timer(500ms, [this]() { status_tick(); });
    }

    // Khi node chet, phai yeu cau dung ngay. Gui lenh EMG mot lan cuoi.
    ~FusionNode() override {
        control_timer_.reset();
        viz_timer_.reset();
        status_timer_.reset();

        if (serial_ && serial_->is_open()) {
            SerialCommand stop;
            stop.dev_final_px = 0;
            stop.speed_control = 0;
            stop.emergency_stop = true;
            serial_->send_command(stop);
        }

        if (camera_) {
            camera_->stop();
        }

        if (serial_) {
            serial_->close();
        }
    }

private:

    // =========================================================================
    // VONG DIEU KHIEN 100 Hz
    // =========================================================================

    void control_tick() {
        CameraLane::LaneOutput lane;
        camera_->get_latest(lane);

        const bool camera_ok = camera_->is_running() && !lane.stale;
        const auto t_now = std::chrono::steady_clock::now();

        int dev = 0;
        int speed = 0;
        bool emg = false;

        // Chi bao cao, khong gui xuong ESP32
        float dev_cm = 0.0f;
        float width_cm = 0.0f;

        if (!camera_ok) {
            // Mat camera -> dung ngay
            last_dev_px_ = 0;
            lane_lost_since_ = {};
            two_lanes_ = false;
            width_cm = 0.0f;
            emg = true;
        } else if (lane.two_lanes) {
            // Du 2 vanh: dung huong moi va reset dong ho mat lane
            last_dev_px_ = dev_sign_ * lane.dev_px;
            lane_lost_since_ = t_now;
            two_lanes_ = true;
            dev = last_dev_px_;
            speed = speed_x10_;
            dev_cm = static_cast<float>(dev_sign_) * lane.dev_cm;
            width_cm = lane.lane_width_cm;
        } else {
            // Camera con song nhung mat 2 vanh: giu huong lai, hao toan trong
            // thoi gian ngan, qua doan thi dung han.
            two_lanes_ = false;
            width_cm = 0.0f;

            if (lane_lost_since_ == std::chrono::steady_clock::time_point{}) {
                lane_lost_since_ = t_now;
            }

            const auto lost = std::chrono::duration_cast<std::chrono::milliseconds>(
                t_now - lane_lost_since_
            ).count();

            if (lost >= lane_lost_stop_ms_) {
                last_dev_px_ = 0;
                emg = true;
            } else {
                dev = last_dev_px_;
                speed = speed_hold_x10_;
            }
        }

        // 2 bien nay chi de bao cao, khong gui xuong ESP32 -> luu lai cho status_tick()
        last_dev_cm_ = dev_cm;
        last_width_cm_ = width_cm;

        age_ms_ = lane.age_ms;
        proc_ms_ = lane.proc_ms;
        frame_id_ = lane.frame_id;
        emg_ = emg;

        SerialCommand cmd;
        cmd.dev_final_px = static_cast<int16_t>(std::clamp(dev, -32768, 32767));
        cmd.speed_control = static_cast<uint8_t>(std::clamp(speed, 0, 255));
        cmd.emergency_stop = emg;

        // Chi de bao cao len GUI: lenh toc do da gui cho dong co (km/h x10)
        last_speed_ = cmd.speed_control;

        if (!serial_->send_command(cmd)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 3000,
                "Khong gui duoc lenh xuong ESP32 (cong serial da dong?)"
            );
        }
    }

    // =========================================================================
    // VONG VE ANH 10 Hz
    // =========================================================================

    void viz_tick() {
        if (!vis_pub_) {
            return;
        }

        CameraLane::LaneOutput lane;
        camera_->get_latest(lane, true);

        if (lane.vis.empty()) {
            return;
        }

        sensor_msgs::msg::Image msg;
        msg.header.stamp = now();
        msg.height = static_cast<uint32_t>(lane.vis.rows);
        msg.width = static_cast<uint32_t>(lane.vis.cols);
        msg.encoding = "bgr8";
        msg.is_bigendian = 0;
        msg.step = static_cast<uint32_t>(lane.vis.cols * 3);
        msg.data.resize(static_cast<size_t>(msg.step) * msg.height);

        for (int y = 0; y < lane.vis.rows; ++y) {
            std::memcpy(
                msg.data.data() + static_cast<size_t>(y) * msg.step,
                lane.vis.ptr(y),
                msg.step
            );
        }

        vis_pub_->publish(msg);
    }

    // =========================================================================
    // TRANG THAI + LOG 5 Hz
    // =========================================================================

    void status_tick() {
        const auto t_now = std::chrono::steady_clock::now();

        double fps = 0.0;
        if (last_status_at_ != std::chrono::steady_clock::time_point{}) {
            const auto dt = std::chrono::duration<double>(
                t_now - last_status_at_
            ).count();
            if (dt > 0.0) {
                fps = static_cast<double>(frame_id_ - last_frame_id_) / dt;
            }
        }
        last_status_at_ = t_now;
        last_frame_id_ = frame_id_;

        const float front_cm = lidar_status_.front_min_cm.value_or(-1.0f);
        const bool lidar_alive = lidar_has_data_ && (
            std::chrono::duration_cast<std::chrono::milliseconds>(
                t_now - lidar_seen_at_
            ).count() < 1000
        );

        // age_ms_ = -1 (unsigned) khi chua co frame nao, dua ve -1 de doc de
        const long age_log = (age_ms_ == static_cast<unsigned long>(-1))
            ? -1L
            : static_cast<long>(age_ms_);

        // Telemetry dong co tu ESP32 (~50 Hz): van toc thuc te + tuoi goi RX
        const auto fb = serial_->get_latest_feedback();
        const unsigned long fb_age = serial_->feedback_age_ms();
        const long fbage_log = (fb_age == static_cast<unsigned long>(-1))
            ? -1L
            : static_cast<long>(fb_age);

        char buf[384];
        std::snprintf(
            buf, sizeof(buf),
            "two_lanes=%d dev=%d emg=%d age=%ldms proc=%.1fms fps=%.1f "
            "lidar=%s front=%.0fcm serial=%s w=%.0fcm devm=%.0fcm "
            "spd=%.1f kmh=%.2f fbage=%ld alert=%s",
            (two_lanes_ ? 1 : 0),
            last_dev_px_,
            (emg_ ? 1 : 0),
            age_log,
            proc_ms_,
            fps,
            (lidar_alive ? "ok" : "none"),
            (front_cm < 0.0f ? -1.0f : front_cm),
            (serial_->is_open() ? "open" : "closed"),
            // 0 = khong do duoc (mat 2 vanh, hoac qua gan chan troi)
            (last_width_cm_ > 0.0f ? last_width_cm_ : -1.0f),
            (two_lanes_ ? last_dev_cm_ : -1.0f),
            (static_cast<float>(last_speed_) / 10.0f),
            (fb.valid ? fb.velocity_kmh : -1.0f),
            fbage_log,
            lidar_status_.alert.c_str()
        );

        std_msgs::msg::String msg;
        msg.data = buf;
        status_pub_->publish(msg);

        static int tick_count = 0;
        if ((tick_count++ % 3) == 0) {
            RCLCPP_INFO(get_logger(), "%s", buf);
        }
    }

    // =========================================================================
    // THANH VIEN
    // =========================================================================

    std::unique_ptr<CameraLane> camera_;
    std::unique_ptr<SerialESP32> serial_;

    LidarModule lidar_;
    LidarStatus lidar_status_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    std::chrono::steady_clock::time_point lidar_seen_at_{};
    bool lidar_has_data_ = false;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr vis_pub_;

    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr viz_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    int speed_x10_ = 40;
    int speed_hold_x10_ = 20;
    int dev_sign_ = 1;

    int last_dev_px_ = 0;
    int lane_lost_stop_ms_ = 400;

    bool two_lanes_ = false;
    bool emg_ = false;

    // Gia tri do duoc o control_tick(), chi de status_tick() log
    float last_dev_cm_ = 0.0f;
    float last_width_cm_ = 0.0f;

    // Lenh toc do (km/h x10) da gui cho dong co o tick gan nhat, chi de bao cao
    uint8_t last_speed_ = 0;

    unsigned long age_ms_ = 0;
    double proc_ms_ = 0.0;

    long frame_id_ = 0;
    long last_frame_id_ = 0;
    std::chrono::steady_clock::time_point last_status_at_{};
    std::chrono::steady_clock::time_point lane_lost_since_{};
};

// ============================================================================
// MAIN
// ============================================================================

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FusionNode>());
    rclcpp::shutdown();
    return 0;
}