// ============================================================================
// FUSION NODE - node dieu khien duy nhat cua xe
// ----------------------------------------------------------------------------
// Chuoi: CameraLane (nhan 2 lan) -> SerialESP32 -> ESP32 qua UART
// (CP2102, /dev/ttyUSB0, 230400 baud).
//
// Luong (MultiThreadedExecutor, 2 nhom callback):
//   ctrl_group_: control_tick 100 Hz, status_tick 10 Hz, /scan, /autocar/run
//   viz_group_ : viz_tick (nen JPEG anh cho GUI)
// Nen anh chay o luong rieng nen KHONG lam tre vong dieu khien.
//
// CHAY / DUNG: xe chi chay khi GUI (nut SPACE) gui /autocar/run = true va
// van tiep tuc gui deu (heartbeat). Mat heartbeat qua start_timeout_ms (GUI
// tat, treo) -> xe dung. Dat require_start:=false de chay khong can GUI.
//
// Dieu kien dung QUAN TRONG: firmware kep PWM bang [95,180], nen speed = 0
// van bi day len 95 (~1.55 km/h). Tin hieu dung that su la emg = 1.
//
// LiDAR: subscribe /scan de quan sat va bao cao, KHONG dua vao quyet dinh lai.
// ============================================================================

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>

#include "camera_node.hpp"
#include "esp32s3_node.hpp"
#include "lidar_node.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using namespace std::chrono_literals;
using SteadyClock = std::chrono::steady_clock;

namespace {

std::chrono::nanoseconds period_from_hz(double hz) {
    const double safe_hz = (hz > 1.0) ? hz : 1.0;
    return std::chrono::nanoseconds(static_cast<int64_t>(1e9 / safe_hz));
}

long ms_since(SteadyClock::time_point t, SteadyClock::time_point now) {
    if (t == SteadyClock::time_point{}) {
        return -1;
    }
    return static_cast<long>(
        std::chrono::duration_cast<std::chrono::milliseconds>(now - t).count());
}

// JPEG cho anh mau, PNG cho mask nhi phan (lossless, khong nhieu vien)
bool encode_image(const cv::Mat& img, const std::string& ext,
                  sensor_msgs::msg::CompressedImage& out) {
    if (img.empty()) {
        return false;
    }
    std::vector<int> params;
    if (ext == ".jpg") {
        params = {cv::IMWRITE_JPEG_QUALITY, 80};
    } else {
        params = {cv::IMWRITE_PNG_COMPRESSION, 1};   // nhanh, mask nen da nho
    }
    std::vector<uchar> buf;
    if (!cv::imencode(ext, img, buf, params)) {
        return false;
    }
    out.format = (ext == ".jpg") ? "jpeg" : "png";
    out.data = std::move(buf);
    return true;
}

// ----------------------------------------------------------------------------
// Uoc luong goc servo tu dev, CUNG CONG THUC voi firmware
// (firmware/esp32s3/autonomous_vehicle/autonomous_vehicle.ino, calcSteer).
// ESP32 khong gui goc servo len nen GUI hien gia tri uoc luong nay.
// ----------------------------------------------------------------------------
constexpr float FW_STEER_CENTER = 90.0f;
constexpr float FW_STEER_RANGE = 30.0f;   // 60..120 do
constexpr float FW_CAM_DEADZONE = 10.0f;  // px anh tham chieu 640
constexpr float FW_CAM_MAX_DEV = 50.0f;
constexpr float FW_STEER_KP = 0.8f;

float estimate_servo_deg(int dev_px) {
    const float a = std::fabs(static_cast<float>(dev_px));
    const float off =
        std::clamp(a - FW_CAM_DEADZONE, 0.0f, FW_CAM_MAX_DEV - FW_CAM_DEADZONE) /
        (FW_CAM_MAX_DEV - FW_CAM_DEADZONE) * FW_STEER_RANGE * FW_STEER_KP;
    return dev_px < 0 ? FW_STEER_CENTER + off : FW_STEER_CENTER - off;
}

}   // namespace

class FusionNode : public rclcpp::Node {

public:

    FusionNode()
        : rclcpp::Node("fusion_node") {

        // ---- Tham so ----
        const auto serial_port = declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        const auto lidar_port = declare_parameter<std::string>("lidar_port", "/dev/ttyUSB1");
        const auto camera_index = declare_parameter<int>("camera_index", -1);
        const auto camera_fps = declare_parameter<int>("camera_fps", 30);
        const auto camera_width = declare_parameter<int>("camera_width", 1920);
        const auto camera_height = declare_parameter<int>("camera_height", 1080);
        // <= 0: phoi sang tu dong. > 0: phoi sang tay (100 us), vd 250 de giu 30 fps
        const auto camera_exposure = declare_parameter<int>("camera_exposure", -1);
        // Hinh hoc camera: chan troi (ti le chieu cao anh) + do cao cam (m)
        const auto horizon_frac = declare_parameter<double>(
            "horizon_frac", CameraProfile{}.horizon_frac);
        const auto camera_height_m = declare_parameter<double>(
            "camera_height_m", CameraProfile{}.height_m);
        const auto roi_top_frac = declare_parameter<double>(
            "roi_top_frac", CameraProfile{}.roi_top_frac);
        const auto speed_x10 = declare_parameter<int>("speed_x10", 30);
        const auto speed_hold_x10 = declare_parameter<int>("speed_hold_x10", 15);
        const auto speed_corner_x10 = declare_parameter<int>("speed_corner_x10", 15);
        const auto speed_ramp_x10 = declare_parameter<int>("speed_ramp_x10", 8);
        // Toc do nho nhat khi xe CHAY (km/h x10). Duoi muc nay ESC ra PWM 95,
        // dong co khong du luc thang ma sat tinh -> banh dung yen.
        const auto speed_min_x10 = declare_parameter<int>("speed_min_x10", 25);
        const auto dev_sign = declare_parameter<int>("dev_sign", 1);
        const auto lane_lost_stop_ms = declare_parameter<int>("lane_lost_stop_ms", 400);
        const auto control_hz = declare_parameter<int>("control_hz", 100);
        const auto viz_hz = declare_parameter<int>("viz_hz", 30);
        const auto status_hz = declare_parameter<int>("status_hz", 10);
        const auto enable_viz = declare_parameter<bool>("enable_viz", true);
        const auto enable_lidar = declare_parameter<bool>("enable_lidar", true);
        const auto lidar_mount_offset_deg = declare_parameter<double>(
            "lidar_mount_offset_deg", LidarModule::DEFAULT_MOUNT_OFFSET_DEG);
        const auto require_start = declare_parameter<bool>("require_start", true);
        const auto start_timeout_ms = declare_parameter<int>("start_timeout_ms", 600);
        const auto log_level = declare_parameter<std::string>("log_level", "info");

        speed_x10_ = static_cast<int>(speed_x10);
        speed_hold_x10_ = static_cast<int>(speed_hold_x10);
        speed_corner_x10_ = static_cast<int>(speed_corner_x10);
        speed_ramp_x10_ = static_cast<float>(speed_ramp_x10);
        speed_min_x10_ = static_cast<int>(speed_min_x10);
        // declare_parameter<int> tra ve int64_t -> ep kieu ro rang cho std::max
        dt_control_s_ = 1.0f / static_cast<float>(std::max<int64_t>(1, control_hz));
        lane_lost_stop_ms_ = static_cast<int>(lane_lost_stop_ms);
        dev_sign_ = (dev_sign < 0) ? -1 : 1;
        require_start_ = require_start;
        start_timeout_ms_ = static_cast<long>(start_timeout_ms);
        lidar_offset_deg_ = static_cast<float>(lidar_mount_offset_deg);

        if (log_level == "debug") {
            get_logger().set_level(rclcpp::Logger::Level::Debug);
        } else if (log_level == "warn") {
            get_logger().set_level(rclcpp::Logger::Level::Warn);
        }

        ctrl_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        viz_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions ctrl_opts;
        ctrl_opts.callback_group = ctrl_group_;

        // ---- Camera ----
        CameraProfile profile;
        profile.horizon_frac = static_cast<float>(horizon_frac);
        profile.height_m = static_cast<float>(camera_height_m);
        profile.roi_top_frac = static_cast<float>(roi_top_frac);
        camera_ = std::make_unique<CameraLane>(
            static_cast<int>(camera_index), static_cast<int>(camera_fps),
            static_cast<int>(camera_width), static_cast<int>(camera_height), profile);
        camera_->set_roi_top_frac(static_cast<float>(roi_top_frac));
        camera_->set_manual_exposure(static_cast<int>(camera_exposure));

        if (camera_->start()) {
            RCLCPP_INFO(get_logger(), "Camera started (%ldx%ld @ %ld fps requested)",
                        static_cast<long>(camera_width), static_cast<long>(camera_height),
                        static_cast<long>(camera_fps));
        } else {
            RCLCPP_ERROR(get_logger(),
                         "Chua mo duoc camera, dang tu thu lai moi 2 s (xe dung cho toi khi "
                         "co hinh). Kiem tra cap USB camera, /dev/video* va nhom video.");
        }

        // ---- Serial ESP32 (UART) ----
        serial_ = std::make_unique<SerialESP32>(serial_port, lidar_port);
        if (serial_->open()) {
            RCLCPP_INFO(get_logger(), "ESP32 serial open on %s", serial_->port().c_str());
        } else {
            RCLCPP_ERROR(get_logger(), "serial FAIL (%s), se thu mo lai moi giay",
                         serial_port.c_str());
        }
        last_reconnect_at_ = SteadyClock::now();

        // ---- Topic ----
        status_pub_ = create_publisher<std_msgs::msg::String>("/lane/status", 10);

        if (enable_viz) {
            vis_pub_ = create_publisher<sensor_msgs::msg::Image>("/lane/vis", 2);
            vis_jpg_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
                "/autocar/dbg/lane_vis/compressed", 1);
            raw_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
                "/autocar/dbg/cam_raw/compressed", 1);
            roi_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
                "/autocar/dbg/lane_roi/compressed", 1);
            bin_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
                "/autocar/dbg/lane_bin/compressed", 1);
        }

        if (enable_lidar) {
            lidar_.set_mount_offset_deg(lidar_offset_deg_);
            lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan", rclcpp::SensorDataQoS(),
                [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
                    lidar_.update(*msg, lidar_status_);
                    lidar_seen_at_ = SteadyClock::now();
                },
                ctrl_opts);
        }

        // Lenh chay/dung tu GUI (nut SPACE). GUI gui lai ~10 Hz lam heartbeat.
        run_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/autocar/run", 10,
            [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
                run_requested_ = msg->data;
                run_seen_at_ = SteadyClock::now();
            },
            ctrl_opts);

        // ---- Timer ----
        control_timer_ = create_wall_timer(
            period_from_hz(static_cast<double>(control_hz)),
            [this]() { control_tick(); }, ctrl_group_);

        if (enable_viz) {
            viz_timer_ = create_wall_timer(
                period_from_hz(static_cast<double>(viz_hz)),
                [this]() { viz_tick(); }, viz_group_);
        }

        status_timer_ = create_wall_timer(
            period_from_hz(static_cast<double>(status_hz)),
            [this]() { status_tick(); }, ctrl_group_);

        RCLCPP_INFO(get_logger(), "%s",
                    require_start_
                        ? "Xe DUNG cho lenh: bam SPACE tren GUI (hoac publish "
                          "/autocar/run true) de chay."
                        : "require_start:=false -> xe chay ngay khi thay lan.");
    }

    // Node tat: gui lenh EMG mot lan cuoi truoc khi dong serial
    ~FusionNode() override {
        control_timer_.reset();
        viz_timer_.reset();
        status_timer_.reset();

        if (serial_ && serial_->is_open()) {
            SerialCommand stop;
            stop.emergency_stop = true;
            for (int i = 0; i < 3; ++i) {
                serial_->send_command(stop);
                std::this_thread::sleep_for(5ms);
            }
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
        const auto t_now = SteadyClock::now();

        CameraLane::LaneOutput lane;
        camera_->get_latest(lane);

        const bool camera_ok = camera_->is_running() && !lane.stale;

        // ---- Lenh chay tu GUI ----
        const long run_age = ms_since(run_seen_at_, t_now);
        const bool armed = !require_start_ ||
                           (run_requested_ && run_age >= 0 && run_age <= start_timeout_ms_);
        if (armed && !armed_) {
            // Vua bam chay: tinh lai dong ho mat lan, toc do ramp tu 0
            lane_lost_since_ = t_now;
            RCLCPP_INFO(get_logger(), "RUN: xe bat dau chay");
        } else if (!armed && armed_) {
            RCLCPP_WARN(get_logger(), "STOP: %s",
                        run_requested_ ? "mat heartbeat tu GUI" : "nguoi dung bam dung");
        }
        armed_ = armed;

        int dev = 0;
        int target = 0;
        bool emg = false;
        float dev_cm = 0.0f;     // chi bao cao
        float width_cm = 0.0f;   // chi bao cao

        if (!camera_ok) {
            // Mat camera -> dung ngay
            last_dev_px_ = 0;
            lane_lost_since_ = {};
            track_ = 0;
            emg = true;
        } else if (lane.state == LaneState::TWO_LINES) {
            // Du 2 vach: duong thang speed_scale = 1 -> day du speed_x10 (ramp),
            // vao cua camera tu giam speed_scale (0.45..1.0 theo do cong)
            last_dev_px_ = dev_sign_ * lane.dev_px;
            lane_lost_since_ = t_now;
            track_ = 2;
            dev = last_dev_px_;
            const float scale = std::clamp(lane.speed_scale, 0.0f, 1.0f);
            target = static_cast<int>(static_cast<float>(speed_x10_) * scale + 0.5f);
            dev_cm = static_cast<float>(dev_sign_) * lane.dev_cm;
            width_cm = lane.lane_width_cm;
        } else if (lane.state == LaneState::ONE_LINE) {
            // Chi thay 1 vach (khuc cua): van lai theo vach, chay cham
            track_ = 1;
            lane_lost_since_ = t_now;
            last_dev_px_ = dev_sign_ * lane.dev_px;
            dev = last_dev_px_;
            target = std::min(speed_corner_x10_,
                              static_cast<int>(speed_x10_ * lane.speed_scale + 0.5f));
            dev_cm = static_cast<float>(dev_sign_) * lane.dev_cm;
        } else {
            // Mat ca 2 vach: giu huong lai, chay cham trong lane_lost_stop_ms
            track_ = 0;
            if (lane_lost_since_ == SteadyClock::time_point{}) {
                lane_lost_since_ = t_now;
            }
            if (ms_since(lane_lost_since_, t_now) >= lane_lost_stop_ms_) {
                last_dev_px_ = 0;
                emg = true;
            } else {
                dev = last_dev_px_;
                target = speed_hold_x10_;
            }
        }

        // Chua bam chay: dung xe, banh thang
        if (!armed_) {
            emg = true;
            dev = 0;
        }

        // Dang chay thi khong de lenh duoi speed_min_x10 (banh se khong quay)
        if (!emg && target > 0) {
            target = std::max(target, speed_min_x10_);
        }

        // Toc do: tang dan (speed_ramp_x10_ don vi x10 moi giay), giam ngay.
        // Bat dau tu speed_min_x10 chu khong tu 0: duoi muc do xe dung yen.
        if (!emg && target > 0 && speed_cur_x10_f_ < speed_min_x10_) {
            speed_cur_x10_f_ = static_cast<float>(speed_min_x10_);
        }
        if (emg) {
            speed_cur_x10_f_ = 0.0f;
        } else if (static_cast<float>(target) > speed_cur_x10_f_) {
            speed_cur_x10_f_ = std::min(static_cast<float>(target),
                                        speed_cur_x10_f_ + speed_ramp_x10_ * dt_control_s_);
        } else {
            speed_cur_x10_f_ = static_cast<float>(target);
        }
        const int speed = static_cast<int>(speed_cur_x10_f_ + 0.5f);

        SerialCommand cmd;
        cmd.dev_final_px = static_cast<int16_t>(std::clamp(dev, -32768, 32767));
        cmd.speed_control = static_cast<uint8_t>(std::clamp(speed, 0, 255));
        cmd.emergency_stop = emg;

        // ---- Luu de status_tick bao cao ----
        last_dev_cm_ = dev_cm;
        last_width_cm_ = width_cm;
        last_cmd_dev_ = cmd.dev_final_px;
        last_speed_ = cmd.speed_control;
        two_lanes_ = lane.two_lanes;
        gated_ = lane.gated;
        horizon_frac_ = lane.horizon_frac;
        curvature_ = lane.curvature;
        speed_scale_ = lane.speed_scale;
        age_ms_ = lane.age_ms;
        proc_ms_ = lane.proc_ms;
        frame_id_ = lane.frame_id;
        frame_w_ = lane.frame_w;
        frame_h_ = lane.frame_h;
        emg_ = emg;

        // ---- Gui xuong ESP32, tu mo lai cong khi mat ket noi ----
        const bool ok = serial_->is_open() && !serial_->is_broken() &&
                        serial_->send_command(cmd);
        if (!ok) {
            if (ms_since(last_reconnect_at_, t_now) >= 1000) {
                last_reconnect_at_ = t_now;
                serial_->close();
                if (serial_->open()) {
                    RCLCPP_INFO(get_logger(), "ESP32 serial reopened on %s",
                                serial_->port().c_str());
                }
            }
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                                 "Khong gui duoc lenh xuong ESP32 (cong serial dong?)");
        }
    }

    // =========================================================================
    // ANH QUAN SAT CHO GUI (luong rieng)
    // =========================================================================

    void viz_tick() {
        auto subs = [](const auto& pub) {
            return pub && pub->get_subscription_count() > 0;
        };
        const bool want_vis = subs(vis_pub_);
        const bool want_jpg = subs(vis_jpg_pub_);
        const bool want_raw = subs(raw_pub_);
        const bool want_roi = subs(roi_pub_);
        const bool want_bin = subs(bin_pub_);

        // Khong ai xem -> khong lay anh -> luong camera cung thoi ve overlay
        if (!(want_vis || want_jpg || want_raw || want_roi || want_bin)) {
            return;
        }

        CameraLane::LaneOutput lane;
        camera_->get_latest(lane, true);

        // Chi phat khi co frame MOI (tranh gui lap cung 1 anh)
        if (lane.vis.empty() || lane.vis_frame_id == last_viz_frame_) {
            return;
        }
        last_viz_frame_ = lane.vis_frame_id;

        std_msgs::msg::Header header;
        header.stamp = now();
        header.frame_id = "camera";

        if (want_vis) {
            sensor_msgs::msg::Image msg;
            msg.header = header;
            msg.height = static_cast<uint32_t>(lane.vis.rows);
            msg.width = static_cast<uint32_t>(lane.vis.cols);
            msg.encoding = "bgr8";
            msg.is_bigendian = 0;
            msg.step = static_cast<uint32_t>(lane.vis.cols * 3);
            msg.data.resize(static_cast<size_t>(msg.step) * msg.height);
            for (int y = 0; y < lane.vis.rows; ++y) {
                std::memcpy(msg.data.data() + static_cast<size_t>(y) * msg.step,
                            lane.vis.ptr(y), msg.step);
            }
            vis_pub_->publish(std::move(msg));
        }

        auto publish = [&](const auto& pub, const cv::Mat& img, const char* ext) {
            sensor_msgs::msg::CompressedImage msg;
            msg.header = header;
            if (encode_image(img, ext, msg)) {
                pub->publish(std::move(msg));
            }
        };

        if (want_jpg) {
            publish(vis_jpg_pub_, lane.vis, ".jpg");
        }
        if (want_raw) {
            publish(raw_pub_, lane.raw, ".jpg");
        }
        if (want_roi) {
            publish(roi_pub_, lane.roi, ".jpg");
        }
        if (want_bin && !lane.bin.empty()) {
            const cv::Mat bin255 = lane.bin * 255;
            publish(bin_pub_, bin255, ".png");
        }
    }

    // =========================================================================
    // TRANG THAI 10 Hz (GUI doc chuoi key=value nay)
    // =========================================================================

    void status_tick() {
        const auto t_now = SteadyClock::now();

        // FPS camera tinh tren cua so ~1 s cho on dinh
        const double dt = std::chrono::duration<double>(t_now - fps_window_at_).count();
        if (fps_window_at_ == SteadyClock::time_point{}) {
            fps_window_at_ = t_now;
            fps_window_frame_ = frame_id_;
        } else if (dt >= 1.0) {
            fps_ = static_cast<double>(frame_id_ - fps_window_frame_) / dt;
            fps_window_at_ = t_now;
            fps_window_frame_ = frame_id_;
        }

        const long lidar_age = ms_since(lidar_seen_at_, t_now);
        const bool lidar_alive = lidar_status_.has_data && lidar_age >= 0 && lidar_age < 1000;
        auto cm = [&](const std::optional<float>& v) {
            return (lidar_alive && v) ? *v : -1.0f;
        };

        const long age_log = (age_ms_ == static_cast<unsigned long>(-1))
            ? -1L : static_cast<long>(age_ms_);

        const auto fb = serial_->get_latest_feedback();
        const unsigned long fb_age = serial_->feedback_age_ms();
        const long fbage_log = (fb_age == static_cast<unsigned long>(-1))
            ? -1L : static_cast<long>(fb_age);

        const char* kTrack[] = {"lost", "one", "two"};
        char buf[720];
        std::snprintf(
            buf, sizeof(buf),
            "two_lanes=%d track=%s dev=%d emg=%d run=%d age=%ldms proc=%.1fms "
            "fps=%.1f cam=%dx%d gate=%d lidar=%s front=%.0fcm left=%.0fcm "
            "right=%.0fcm rear=%.0fcm alert=%s lofs=%.1f serial=%s w=%.0fcm "
            "devm=%.1fcm curv=%.3f scale=%.2f spd=%.1f kmh=%.2f fbage=%ld "
            "servo=%.1f hz=%.2f esc=%d fwemg=%d fwwd=%d fwcmd=%d",
            two_lanes_ ? 1 : 0,
            kTrack[(track_ >= 0 && track_ <= 2) ? track_ : 0],
            last_dev_px_,
            emg_ ? 1 : 0,
            armed_ ? 1 : 0,
            age_log,
            proc_ms_,
            fps_,
            frame_w_, frame_h_,
            gated_ ? 1 : 0,
            lidar_alive ? "ok" : "none",
            cm(lidar_status_.front_min_cm),
            cm(lidar_status_.left_min_cm),
            cm(lidar_status_.right_min_cm),
            cm(lidar_status_.rear_bypass_min_cm),
            lidar_alive ? lidar_status_.alert.c_str() : "NONE",
            lidar_offset_deg_,
            (serial_->is_open() && !serial_->is_broken()) ? "open" : "closed",
            last_width_cm_ > 0.0f ? last_width_cm_ : -1.0f,
            track_ >= 1 ? last_dev_cm_ : 0.0f,
            curvature_,
            speed_scale_,
            static_cast<float>(last_speed_) / 10.0f,
            fb.valid ? fb.velocity_kmh : -1.0f,
            fbage_log,
            // Co telemetry v2: goc servo THAT ESP32 dang ghi; khong co: uoc luong
            (fb.has_v2 && fb_age < 300) ? static_cast<float>(fb.steer_deg)
                                        : estimate_servo_deg(emg_ ? 0 : last_cmd_dev_),
            horizon_frac_,
            // Muc xung ESC ESP32 dang phat (90 = neutral), -1 = firmware cu / mat lien lac
            (fb.has_v2 && fb_age < 300) ? fb.esc_deg : -1,
            (fb.has_v2 && fb.fw_emg) ? 1 : 0,
            (fb.has_v2 && fb.fw_watchdog) ? 1 : 0,
            (fb.has_v2 && fb.fw_got_cmd) ? 1 : 0);

        std_msgs::msg::String msg;
        msg.data = buf;
        status_pub_->publish(msg);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "%s", buf);
    }

    // =========================================================================
    // THANH VIEN
    // =========================================================================

    std::unique_ptr<CameraLane> camera_;
    std::unique_ptr<SerialESP32> serial_;

    rclcpp::CallbackGroup::SharedPtr ctrl_group_;
    rclcpp::CallbackGroup::SharedPtr viz_group_;

    LidarModule lidar_;
    LidarStatus lidar_status_;
    float lidar_offset_deg_ = LidarModule::DEFAULT_MOUNT_OFFSET_DEG;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    SteadyClock::time_point lidar_seen_at_{};

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr run_sub_;
    bool run_requested_ = false;
    SteadyClock::time_point run_seen_at_{};
    bool require_start_ = true;
    long start_timeout_ms_ = 600;
    bool armed_ = false;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr vis_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr vis_jpg_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr raw_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr roi_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr bin_pub_;
    long last_viz_frame_ = -1;   // chi dung trong viz_tick

    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr viz_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    int speed_x10_ = 30;
    int speed_hold_x10_ = 15;
    int speed_corner_x10_ = 15;
    float speed_ramp_x10_ = 8.0f;    // don vi x10 moi giay (8 = 0.8 km/h/s)
    int speed_min_x10_ = 25;         // toc do nho nhat khi dang chay
    float speed_cur_x10_f_ = 0.0f;   // lenh hien tai, tang dan khi ramp
    float dt_control_s_ = 0.01f;     // 1 / control_hz
    int dev_sign_ = 1;
    int lane_lost_stop_ms_ = 400;
    SteadyClock::time_point lane_lost_since_{};
    SteadyClock::time_point last_reconnect_at_{};

    // ---- Gia tri control_tick ghi, status_tick doc (cung nhom callback) ----
    int track_ = 0;              // 0 = mat vach, 1 = 1 vach, 2 = du 2 vach
    int last_dev_px_ = 0;
    int last_cmd_dev_ = 0;
    bool two_lanes_ = false;
    bool gated_ = false;
    float horizon_frac_ = 0.0f;
    bool emg_ = true;
    float last_dev_cm_ = 0.0f;
    float last_width_cm_ = 0.0f;
    float curvature_ = 0.0f;
    float speed_scale_ = 1.0f;
    uint8_t last_speed_ = 0;
    unsigned long age_ms_ = 0;
    double proc_ms_ = 0.0;
    long frame_id_ = 0;
    int frame_w_ = 0;
    int frame_h_ = 0;

    double fps_ = 0.0;
    long fps_window_frame_ = 0;
    SteadyClock::time_point fps_window_at_{};
};

// ============================================================================
// MAIN
// ============================================================================

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    {
        auto node = std::make_shared<FusionNode>();
        // 2 luong: 1 cho dieu khien + trang thai, 1 cho nen anh
        rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
        exec.add_node(node);
        exec.spin();
    }   // node huy o day -> gui EMG truoc khi dong serial
    rclcpp::shutdown();
    return 0;
}
