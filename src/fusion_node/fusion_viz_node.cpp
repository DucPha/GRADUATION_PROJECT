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
#include "path_tracker.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <ctime>
#include <memory>
#include <mutex>
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
// 55..125 do (= firmware STEER_MIN/MAX). 10/10: ve lai +-35 nhu lan chay
// 09/10 15:47 (2 vong dau gan nhu khong de vach); +-30 xe kich lai van khong
// om kip cua gap (R nho nhat ~1.15 m). Do doc dev -> goc giu 30 do / 41 px.
constexpr float FW_STEER_RANGE = 35.0f;
constexpr float FW_CAM_DEADZONE = 4.0f;   // px anh tham chieu 640
constexpr float FW_CAM_MAX_DEV = FW_CAM_DEADZONE + 41.0f * FW_STEER_RANGE / 30.0f;
constexpr float FW_STEER_KP = 1.0f;

float estimate_servo_deg(int dev_px) {
    const float a = std::fabs(static_cast<float>(dev_px));
    const float off =
        std::clamp(a - FW_CAM_DEADZONE, 0.0f, FW_CAM_MAX_DEV - FW_CAM_DEADZONE) /
        (FW_CAM_MAX_DEV - FW_CAM_DEADZONE) * FW_STEER_RANGE * FW_STEER_KP;
    return dev_px < 0 ? FW_STEER_CENTER + off : FW_STEER_CENTER - off;
}

// Nguoc cua estimate_servo_deg: goc BANH mong muon (> 0 = phai) -> dev de
// firmware ra dung goc servo do. steer_ratio = do banh / do servo.
int dev_from_wheel_deg(double wheel_deg, double steer_ratio) {
    const double servo_off = std::fabs(wheel_deg) / std::max(0.1, steer_ratio);
    if (servo_off < 0.2) {
        return 0;
    }
    const double full = FW_STEER_RANGE * FW_STEER_KP;   // goc servo luc bao hoa
    const double d = FW_CAM_DEADZONE +
                     std::min(servo_off, full) / full * (FW_CAM_MAX_DEV - FW_CAM_DEADZONE);
    return (wheel_deg > 0.0 ? 1 : -1) * static_cast<int>(std::lround(d));
}

// Ban do nho nhin tu tren (goc phai tren anh GUI): xe, duong tam da nho, vach
// that/ao, diem ngam. Toa do xe: goc truc sau, X phai, Z truoc.
void draw_bev(cv::Mat& img, const PathTracker::Output& t, double wheelbase_m,
              double cam_to_rear_m, double near_z_m) {
    const int W = 150, H = 190, ppm = 100;
    const double z_min = -0.35;
    const int ox = img.cols - W - 6, oy = 30;
    if (ox < 0 || oy + H > img.rows) {
        return;
    }
    cv::Mat roi = img(cv::Rect(ox, oy, W, H));
    roi.convertTo(roi, -1, 0.35, 0.0);
    auto P = [&](const cv::Point2f& q) {
        return cv::Point(ox + W / 2 + cvRound(q.x * ppm),
                         oy + H - cvRound((q.y - z_min) * ppm));
    };
    auto poly = [&](const std::vector<cv::Point2f>& v, const cv::Scalar& c, int th) {
        std::vector<cv::Point> p;
        for (const auto& q : v) {
            p.push_back(P(q));
        }
        if (p.size() > 1) {
            cv::polylines(img, p, false, c, th, cv::LINE_AA);
        }
    };
    // Mep gan nhat camera thay duoc (truoc chan camera near_z_m)
    const float z_vis = static_cast<float>(cam_to_rear_m + near_z_m);
    cv::line(img, P({-0.75f, z_vis}), P({0.75f, z_vis}), cv::Scalar(90, 90, 90), 1);
    cv::rectangle(img, cv::Rect(ox, oy, W, H), cv::Scalar(120, 120, 120), 1);
    poly(t.left_v, t.virt_left ? cv::Scalar(255, 0, 255) : cv::Scalar(0, 230, 0), 2);
    poly(t.right_v, t.virt_right ? cv::Scalar(255, 0, 255) : cv::Scalar(255, 120, 0), 2);
    poly(t.centre_v, cv::Scalar(0, 230, 255), 1);
    // Xe: tu sau truc sau 0.08 m toi truoc truc truoc 0.08 m, rong 0.2 m
    const float L = static_cast<float>(wheelbase_m);
    cv::rectangle(img, P({-0.10f, L + 0.08f}), P({0.10f, -0.08f}),
                  cv::Scalar(235, 235, 235), 1);
    if (t.valid) {
        cv::circle(img, P(t.target_v), 4, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
        cv::line(img, P({0.0f, 0.0f}), P(t.target_v), cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    }
    const char* turn = t.turn > 0 ? "CUA PHAI" : (t.turn < 0 ? "CUA TRAI" : "THANG");
    cv::putText(img, turn, cv::Point(ox + 4, oy + 14), cv::FONT_HERSHEY_SIMPLEX, 0.4,
                cv::Scalar(235, 235, 235), 1, cv::LINE_AA);
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
        // 0: do roi khoa phoi sang (mac dinh). > 0: co dinh (100 us). < 0: camera tu dong
        const auto camera_exposure = declare_parameter<int>("camera_exposure", 0);
        // Hinh hoc camera: do cao (m), goc cui (do), goc nhin doc (do), be rong lan (m)
        const CameraProfile cam_def;
        const auto camera_height_m = declare_parameter<double>(
            "camera_height_m", cam_def.height_m);
        const auto camera_pitch_deg = declare_parameter<double>(
            "camera_pitch_deg", cam_def.pitch_deg);
        const auto camera_vfov_deg = declare_parameter<double>(
            "camera_vfov_deg", cam_def.vfov_deg);
        const auto roi_top_frac = declare_parameter<double>(
            "roi_top_frac", cam_def.roi_top_frac);
        // Toc do LENH (km/h x10). Firmware ghi ESC theo us: 1543 + (v - 3.58) /
        // 2.98 * 19.5 us, san 1550 us (+3 us khi het lai): 52 -> ~1554 us,
        // 65 -> ~1562 us. Toc do THAT do tu anh 09/10 chi ~0.42 m/s o lenh 48
        // (x0.32) va giam dan khi pin yeu -> du cham cho xu ly anh (1.7 cm /
        // frame); san thap hon thi BLDC keu / dung.
        // speed_x10: du 2 vach + duong thang; vao cua 2 vach noi suy ve
        // speed_corner_x10 theo speed_scale cua camera.
        const auto speed_x10 = declare_parameter<int>("speed_x10", 70);
        // Chi thay 1 vach tren duong thang + xe giua lan (vach kia mo / loa);
        // cua hoac xe lech -> ve speed_corner_x10
        const auto speed_one_x10 = declare_parameter<int>("speed_one_x10", 60);
        const auto speed_hold_x10 = declare_parameter<int>("speed_hold_x10", 58);
        const auto speed_corner_x10 = declare_parameter<int>("speed_corner_x10", 58);
        // Tang toc khi het cua: x10 km/h moi giay (12: 58 -> 70 deu trong ~1 s)
        const auto speed_ramp_x10 = declare_parameter<int>("speed_ramp_x10", 12);
        // Giam toc vao cua: x10 km/h moi giay (20: 65 -> 58 trong ~0.35 s, deu)
        speed_down_x10_ = static_cast<float>(declare_parameter<int>("speed_down_x10", 20));
        // Toc do nho nhat khi xe CHAY (km/h x10). Xung thap (1543 us) BLDC keu
        // cot ket; 10/10 cua gat 52 (~1554 us) van keu ket -> 58 (~1557.5 us).
        const auto speed_min_x10 = declare_parameter<int>("speed_min_x10", 58);
        const auto dev_sign = declare_parameter<int>("dev_sign", 1);
        // Mat ca 2 vach bay lau moi dung (Python: bo cham giu goc lai, chi dung
        // khi ra khoi duong ~3 s). Ban truoc 400 ms: 1 frame toi / loa den la
        // xe dung giua duong.
        const auto lane_lost_stop_ms = declare_parameter<int>("lane_lost_stop_ms", 2500);
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
        // Ghi anh camera khi xe CHAY (de chinh xu ly anh bang anh duong dua
        // that): ~/lane_logs/run_YYYYmmdd_HHMMSS/*.jpg + frames.csv
        const auto record_frames = declare_parameter<bool>("record_frames", true);
        const auto record_root = declare_parameter<std::string>("record_dir", "auto");

        // ---- Lai theo duong da nho (PathTracker): pure pursuit tu truc sau
        // tren duong nho + odometry. Camera mat duong nho -> lai theo dev_px.
        VehicleParams vp;
        vp.wheelbase_m = declare_parameter<double>("wheelbase_m", vp.wheelbase_m);
        vp.cam_to_rear_m = declare_parameter<double>("cam_to_rear_axle_m", vp.cam_to_rear_m);
        vp.lookahead_min_m = declare_parameter<double>("lookahead_min_m", vp.lookahead_min_m);
        vp.lookahead_max_m = declare_parameter<double>("lookahead_max_m", vp.lookahead_max_m);
        vp.lookahead_gain_s = declare_parameter<double>("lookahead_gain_s", vp.lookahead_gain_s);
        vp.lookahead_corner_scale = declare_parameter<double>("lookahead_corner_scale", vp.lookahead_corner_scale);
        vp.camera_latency_s = declare_parameter<double>("camera_latency_s", vp.camera_latency_s);
        vp.actuator_latency_s = declare_parameter<double>("actuator_latency_s", vp.actuator_latency_s);
        vp.car_half_width_m = declare_parameter<double>("car_half_width_m", vp.car_half_width_m);
        vp.tape_half_m = declare_parameter<double>("tape_half_m", vp.tape_half_m);
        vp.line_margin_m = declare_parameter<double>("line_margin_m", vp.line_margin_m);
        // Danh lai manh hon trong cua (nhan phan pure pursuit), 1 = tat
        vp.corner_gain = declare_parameter<double>("corner_gain", vp.corner_gain);
        // Goc gap: chi be lai khi truc sau con cach dinh R * tan(goc / 2)
        vp.kink_radius_m = declare_parameter<double>("kink_radius_m", vp.kink_radius_m);
        // Phan hoi lech ngang (P + I) tai chan camera, xem path_tracker.hpp
        vp.xte_gain = declare_parameter<double>("xte_gain", vp.xte_gain);
        vp.xte_ki = declare_parameter<double>("xte_ki", vp.xte_ki);
        vp.xte_i_max_deg = declare_parameter<double>("xte_i_max_deg", vp.xte_i_max_deg);
        vp.xte_far_m = declare_parameter<double>("xte_far_m", vp.xte_far_m);
        vp.xte_far_gain = declare_parameter<double>("xte_far_gain", vp.xte_far_gain);
        // PID duong thang (kp * lech + kd * goc huong + I), xem path_tracker.hpp
        vp.pid_kp_deg_per_m = declare_parameter<double>("pid_kp", vp.pid_kp_deg_per_m);
        vp.pid_kd = declare_parameter<double>("pid_kd", vp.pid_kd);
        vp.pid_ki = declare_parameter<double>("pid_ki", vp.pid_ki);
        vp.pid_i_max_deg = declare_parameter<double>("pid_i_max_deg", vp.pid_i_max_deg);
        vp.pid_max_deg = declare_parameter<double>("pid_max_deg", vp.pid_max_deg);
        vp.pid_filter_s = declare_parameter<double>("pid_filter_s", vp.pid_filter_s);
        // do banh / do servo: servo quay 30 do ma banh chi quay ~20 do -> 0.65.
        // Chay that 08/10 voi 0.8: xe bat ve phia ngoai cua, cua nho be lai
        // khong du (banh quay it hon khai bao). Thu vong kin: khai bao 0.65 ben
        // nhat khi ty so that 0.55-0.8 (khai bao cao hon that -> lech ra ngoai).
        // 08/10 voi 0.65 van be lai thieu o cua nhe -> 0.6
        steer_ratio_ = declare_parameter<double>("steer_ratio", 0.6);
        // Toc do THAT / toc do lenh, CHI cho odometry (bo nho duong, du doan
        // vach giua 2 frame, bu tre servo). Ld / phan hoi xte van theo toc do
        // lenh nhu cu. Do tu anh 10/10 18:05 (khoang cach toi goc gap giam
        // dan): lenh 5.8 km/h (1.61 m/s) -> that ~0.44 m/s (x0.27; 09/10:
        // x0.32). Truoc day 1.0: bo nho troi nhanh gap ~3.5 lan -> diem goc
        // gap trong vung mu toi "som" -> be lai SOM, lech ngang moi frame nhay
        // 8 cm luc camera chot lai, lai giat > 8 do/frame 11% frame (con 2%).
        odom_speed_scale_ = declare_parameter<double>("odom_speed_scale", 0.30);
        // GOC SERVO MA BANH DI THANG. 10/10 18:05 chay 2 chieu cung 1 duong:
        // goc banh TB chieu nguoc kim -10.4, chieu kim dong ho -0.2 (doi xung
        // phai la +-x) -> lenh 0 (servo 90) xe da re PHAI ~4.5-5 do banh. Chieu
        // kim dong ho (cua phai) xe be som, de vach trong 28% frame (chieu kia
        // 2%), duong thang lech phai 7 cm; khau I bao hoa 6 do de bu. Uoc luong
        // tu toc do quay dau tren duong thang cung ra ~4.5 do banh = 7.5 do
        // servo. NHUNG ke xe len, servo 90 banh van thang (nguoi dung kiem
        // tra 10/10) -> mac dinh 90 = KHONG bu, chi de thu.
        servo_straight_deg_ = declare_parameter<double>("servo_straight_deg", 90.0);
        vp.max_steer_deg = FW_STEER_RANGE * FW_STEER_KP * steer_ratio_;
        // GIAM TOC TRUOC CUA: ESC chi nha ga (khong phanh) nen xe troi cham
        // dan voi gia toc ~coast_decel_mps2. Thay cua cach d (m) phia truoc
        // (PathTracker::corner_dist_m) -> toc do cho phep
        //   v = sqrt(v_cua^2 + 2 * coast_decel * (d - corner_margin_m))
        // de toi diem cach cua corner_margin_m da xuong toc do cua. Gia toc
        // nay cung dung cho odometry khi giam toc (lenh giam ngay nhung xe con
        // troi). Xe van vot cua -> giam coast_decel_mps2 (giam toc som hon).
        coast_decel_mps2_ = declare_parameter<double>("coast_decel_mps2", 0.45);
        corner_margin_m_ = declare_parameter<double>("corner_margin_m", 0.20);
        // Mat ca 2 vach: van chay theo duong da nho toi da bay nhieu ms
        lost_memory_ms_ = static_cast<int>(declare_parameter<int>("lost_memory_ms", 1500));
        // He so lane keeping lay tu ban Python (car_config.py):
        //  steer_filter_s = STEER_FILTER_SEC: loc thong thap goc banh (s), 0 = tat
        //  lane_start_frames = LANE_START_FRAMES: bam chay xong, chi xuat phat
        //    (va TU chay lai sau khi mat lan dung xe) khi thay lan (2 vach hoac
        //    1 vach da bam on dinh) bay nhieu frame camera lien tiep
        steer_filter_s_ = declare_parameter<double>("steer_filter_s", 0.08);
        // Toc do danh lai toi da (do banh / s): duong thang cham -> khong
        // chao (log 10/10 19:27: banh -20 -> +6 -> -16 do moi ~1.5 s tren
        // thang, 150-270 do/s), trong cua nhanh vua. 0 = khong gioi han
        steer_rate_dps_ = declare_parameter<double>("steer_rate_dps", 15.0);
        steer_rate_corner_dps_ = declare_parameter<double>("steer_rate_corner_dps", 45.0);
        lane_start_frames_ = static_cast<int>(declare_parameter<int>("lane_start_frames", 3));
        // Chi thay 1 vach: doi tam bam ve phia vach mat toi da bay nhieu m,
        // toc do doi (m/s)
        single_search_m_ = declare_parameter<double>("single_search_m", 0.08);
        single_search_rate_ = declare_parameter<double>("single_search_rate", 0.30);
        // Vao cua: giu mep xe cach mep bang keo vach NGOAI bay nhieu m (doi
        // duong bam ra ngoai toi da corner_outer_max_m), < 0 = tat
        corner_outer_gap_m_ = declare_parameter<double>("corner_outer_gap_m", 0.08);
        corner_outer_max_m_ = declare_parameter<double>("corner_outer_max_m", 0.15);
        // Cua gat: |do cong phia truoc| (1/m) tu muc nay moi bam vach ngoai.
        // Log 16:43: cua lon (1.5-7 s, banh het lai) giu > 1.1; cua nhe < 1.0
        corner_sharp_k_ = declare_parameter<double>("corner_sharp_k", 1.0);
        tracker_ = PathTracker(vp);

        speed_x10_ = static_cast<int>(speed_x10);
        speed_one_x10_ = static_cast<int>(speed_one_x10);
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
        profile.height_m = static_cast<float>(camera_height_m);
        profile.pitch_deg = static_cast<float>(camera_pitch_deg);
        profile.vfov_deg = static_cast<float>(camera_vfov_deg);
        profile.cam_to_rear_m = static_cast<float>(vp.cam_to_rear_m);
        profile.wheelbase_m = static_cast<float>(vp.wheelbase_m);
        profile.roi_top_frac = static_cast<float>(roi_top_frac);
        camera_ = std::make_unique<CameraLane>(
            static_cast<int>(camera_index), static_cast<int>(camera_fps),
            static_cast<int>(camera_width), static_cast<int>(camera_height), profile);
        camera_->set_roi_top_frac(static_cast<float>(roi_top_frac));
        camera_->set_manual_exposure(static_cast<int>(camera_exposure));
        if (record_frames) {
            std::string root = record_root;
            if (root.empty() || root == "auto") {
                const char* home = std::getenv("HOME");
                root = std::string(home ? home : "/tmp") + "/lane_logs";
            }
            char stamp[32];
            const std::time_t now = std::time(nullptr);
            std::strftime(stamp, sizeof(stamp), "run_%Y%m%d_%H%M%S", std::localtime(&now));
            camera_->set_record_dir(root + "/" + stamp);
        }

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

    // Khoang cach (m) tu diem tren duong tam cach goc toa do Ld (truc sau) toi
    // vach `line`; < 0 khi khong co hoac vach khong phu toi ngang diem do
    static double dist_at_lookahead(const std::vector<cv::Point2f>& centre, double Ld,
                                    const std::vector<cv::Point2f>& line) {
        if (centre.size() < 2 || line.size() < 2) {
            return -1.0;
        }
        const cv::Point2f* p = nullptr;
        for (const auto& q : centre) {
            if (q.y > 0.05f && std::hypot(q.x, q.y) >= Ld) {
                p = &q;
                break;
            }
        }
        if (p == nullptr) {
            return -1.0;
        }
        double best = 1e9;
        bool inside = false;
        for (size_t i = 1; i < line.size(); ++i) {
            const cv::Point2f ab = line[i] - line[i - 1];
            const float u = (*p - line[i - 1]).dot(ab) / std::max(1e-9f, ab.dot(ab));
            const float t = std::clamp(u, 0.0f, 1.0f);
            const cv::Point2f e = *p - (line[i - 1] + ab * t);
            const double d = std::hypot(e.x, e.y);
            if (d < best) {
                best = d;
                // Hinh chieu roi ra ngoai 2 dau vach -> vach khong phu toi day
                inside = !((i == 1 && u < 0.0f) || (i + 1 == line.size() && u > 1.0f));
            }
        }
        return inside ? best : -1.0;
    }

    // Toc do muc tieu khi thay lan: noi suy speed_corner_x10 -> v_max theo
    //  - speed_scale camera (do cong / goc gap doan nhin thay),
    //  - do cong duong da nho phia truoc (ke ca doan da vao vung mu),
    //  - xe co dang GIUA LAN khong: lech <= 3 cm -> 1, >= 7 cm -> 0 (xe 25 cm
    //    trong lan 0.42 m chi con ~5 cm moi ben, lech thi cham lai cho kip ve),
    //  - khoang cach toi cua phia truoc (corner_speed_x10): giam toc tu xa.
    int lane_speed(int v_max, const CameraLane::LaneOutput& lane,
                   const PathTracker::Output& trk, bool use_trk) const {
        const int v = lane_speed_scaled(v_max, lane, trk, use_trk);
        return use_trk ? std::min(v, corner_speed_x10(v_max, trk)) : v;
    }

    // Goc banh THAT mong muon (> 0 phai) -> dev gui firmware, bu servo lech
    // tam (servo_straight_deg: goc servo ma banh di thang)
    int dev_for_wheel(double wheel_deg) const {
        const double trim = (servo_straight_deg_ - FW_STEER_CENTER) * steer_ratio_;
        return dev_sign_ * dev_from_wheel_deg(wheel_deg - dev_sign_ * trim, steer_ratio_);
    }

    // dev cua camera (tinh nhu servo 90 la di thang) -> dev da bu lech tam
    int camera_dev(int dev_px) const {
        const int d = dev_sign_ * dev_px;
        return dev_for_wheel((FW_STEER_CENTER - estimate_servo_deg(d)) * steer_ratio_ * dev_sign_);
    }

    // Toc do cho phep de truot tu do (gia toc coast_decel_mps2) xuong toc do
    // cua truoc khi toi cach cua corner_margin_m. Chua thay cua -> v_max.
    int corner_speed_x10(int v_max, const PathTracker::Output& trk) const {
        if (trk.corner_dist_m < 0.0 || coast_decel_mps2_ <= 0.0) {
            return v_max;
        }
        const double vc = std::min(speed_corner_x10_, v_max) / 36.0;   // m/s
        const double d = std::max(0.0, trk.corner_dist_m - corner_margin_m_);
        const double v = std::sqrt(vc * vc + 2.0 * coast_decel_mps2_ * d);
        return std::min(v_max, static_cast<int>(v * 36.0));
    }

    int lane_speed_scaled(int v_max, const CameraLane::LaneOutput& lane,
                          const PathTracker::Output& trk, bool use_trk) const {
        float scale = lane.speed_scale;
        if (use_trk) {
            const float curv = static_cast<float>(std::fabs(trk.curv_ahead));
            const float centred = 1.0f - std::clamp(
                (static_cast<float>(std::fabs(trk.xte_m)) - 0.03f) / 0.04f, 0.0f, 1.0f);
            scale = std::min({scale, 1.0f - std::clamp((curv - 0.4f) / 1.1f, 0.0f, 1.0f), centred});
        }
        scale = std::clamp(scale, 0.0f, 1.0f);
        const float corner = static_cast<float>(std::min(speed_corner_x10_, v_max));
        return static_cast<int>(corner + (static_cast<float>(v_max) - corner) * scale + 0.5f);
    }

    void control_tick() {
        const auto t_now = SteadyClock::now();

        CameraLane::LaneOutput lane;
        camera_->get_latest(lane);

        const bool camera_ok = camera_->is_running() && !lane.stale;

        // ---- Odometry + bo nho duong ----
        // Goc banh dang ap dung: lay goc servo THAT tu telemetry neu co, khong
        // thi lay goc da ra lenh chu ky truoc
        {
            const auto fb = serial_->get_latest_feedback();
            const unsigned long fb_age = serial_->feedback_age_ms();
            // Goc banh THAT: tinh tu goc servo ma banh di thang
            double wheel = wheel_cmd_deg_;
            if (fb.has_v2 && fb.steer_deg >= 0 && fb_age < 100) {
                wheel = (servo_straight_deg_ - fb.steer_deg) * steer_ratio_ * dev_sign_;
            }
            // Toc do THAT uoc luong: lenh tang -> theo lenh (da ramp); lenh
            // giam -> xe troi cham dan (coast_decel_mps2), dung xe -> firmware
            // phanh (nhanh gap 3)
            {
                const double a = coast_decel_mps2_ > 0.0 ? coast_decel_mps2_ : 1e3;
                const double dec = (speed_cur_x10_f_ <= 0.0f ? 3.0 : 1.0) * a * 36.0 *
                                   static_cast<double>(dt_control_s_);
                const double cmd = speed_cur_x10_f_;
                v_odom_x10_ = cmd >= v_odom_x10_ ? cmd : std::max(cmd, v_odom_x10_ - dec);
            }
            const double v_mps = v_odom_x10_ / 36.0 * odom_speed_scale_;
            wheel_now_deg_ = wheel;
            tracker_.predict(v_mps, wheel, t_now);
            // Camera du doan vi tri vach giua 2 frame theo cung odometry
            camera_->set_motion(static_cast<float>(v_mps), static_cast<float>(wheel));
            if (camera_ok && lane.frame_id != last_obs_frame_) {
                last_obs_frame_ = lane.frame_id;
                if (lane.state != LaneState::LOST) {
                    tracker_.add_observation(lane.centre_g, lane.left_g, lane.right_g,
                                             lane.stamp);
                }
            }
        }
        // Ld / phan hoi xte tinh theo toc do LENH (da chinh tren xe theo so
        // nay); odometry o tren dung toc do that (odom_speed_scale)
        const double v_now = v_odom_x10_ / 36.0;
        // Chi thay 1 vach: doi dan tam bam ve phia vach bi mat (toi da
        // single_search_m) -> xe lai vao trong tim lai vach kia, giu xe giua 2
        // vach va om cua khi vach trong ra khoi khung (SINGLE_LINE_SEARCH_PX
        // cua Python). Thay lai du 2 vach -> tra dan ve 0.
        {
            double want = 0.0;
            if (!camera_ok) {
                want = 0.0;
            } else if (lane.state == LaneState::ONE_LINE) {
                // Chi tren duong THANG: trong cua vach trong ra khoi khung la
                // binh thuong, doi tam ve phia do = ep xe de len vach trong
                // (chay that 09/10: bam vach ngoai nhung de vach trong). Tat
                // dan theo do cong phia truoc 0.3 -> 0.7 1/m.
                const double straight =
                    1.0 - std::clamp((std::fabs(last_curv_ahead_) - 0.3) / 0.4, 0.0, 1.0);
                want = (lane.left_g.empty() ? -1.0 : 1.0) * single_search_m_ * straight;
            } else if (lane.state == LaneState::LOST) {
                want = search_bias_m_;
            }
            const double step = single_search_rate_ * static_cast<double>(dt_control_s_);
            search_bias_m_ += std::clamp(want - search_bias_m_, -step, step);
        }
        // CUA GAT: GIU KHOANG CACH TOI VACH NGOAI. Camera chi thay phia truoc;
        // trong cua gap khuc thuong chi thay vach ngoai, vach trong ra khoi
        // khung -> duong tam (suy tu vach ngoai + be rong lan uoc luong) co the
        // lech. Nen o cua gat bam THANG theo vach ngoai: diem ngam cach vach
        // ngoai sao cho mep xe cach mep bang keo corner_outer_gap_m (du xa de
        // kip xu ly, khong de banh len vach ngoai; phia vach trong con rong).
        // Bat: |do cong phia truoc| >= corner_sharp_k VA banh da be cung chieu
        // >= CORNER_SHARP_IN_DEG, lien tuc CORNER_SHARP_HOLD_S (cua lon, xe da
        // vao cua; cua nhe / moi thay cua phia truoc khong bat). Truoc day bat
        // tu do cong 0.3 -> xe dat ra ngoai TRUOC khi vao cua, vot cham vach
        // ngoai (chay that 16:43). Giu: suot luc banh con be cung chieu >=
        // CORNER_SHARP_OUT_DEG. Khong dua vao do cong de giu: giua cua gap
        // khuc doan thang ke tiep nam cheo phia truoc -> do cong ~0 du banh
        // het lai (log 16:43: 2-5 s het lai, do cong 0-0.3). Tat: banh ve
        // duoi nguong CORNER_SHARP_EXIT_S, hoac duong cong nguoc chieu.
        {
            const double k = last_curv_ahead_;
            const double dt = static_cast<double>(dt_control_s_);
            if (sharp_turn_ == 0) {
                const int sk = k < 0.0 ? -1 : 1;
                const bool in = std::fabs(k) >= corner_sharp_k_ &&
                                wheel_now_deg_ * sk >= CORNER_SHARP_IN_DEG;
                sharp_t_s_ = in ? sharp_t_s_ + dt : 0.0;
                if (sharp_t_s_ >= CORNER_SHARP_HOLD_S) {
                    sharp_turn_ = sk;
                    sharp_t_s_ = 0.0;
                }
            } else {
                sharp_t_s_ = wheel_now_deg_ * sharp_turn_ < CORNER_SHARP_OUT_DEG ? sharp_t_s_ + dt : 0.0;
                if (sharp_t_s_ >= CORNER_SHARP_EXIT_S || k * sharp_turn_ <= -CORNER_SHARP_REV) {
                    sharp_turn_ = 0;
                    sharp_t_s_ = 0.0;
                }
            }
            double want = 0.0;
            if (camera_ok && corner_outer_gap_m_ >= 0.0 && sharp_turn_ != 0) {
                const int dir = -sharp_turn_;   // re trai -> vach ngoai ben PHAI (+)
                const double keep = tracker_.params().car_half_width_m +
                                    tracker_.params().tape_half_m + corner_outer_gap_m_;
                const double out_now = corner_bias_m_ * dir;
                // Diem ngam -> vach ngoai do o chu ky truoc (da gom doi hien
                // tai); khong do duoc -> coi duong tam o giua lan
                const double d = outer_d_m_ > 0.0 ? outer_d_m_ : 0.5 * lane_w_m_ - out_now;
                want = dir * std::clamp(out_now + d - keep, -CORNER_IN_MAX_M, corner_outer_max_m_);
            }
            const double step = corner_outer_rate_ * static_cast<double>(dt_control_s_);
            corner_bias_m_ += std::clamp(want - corner_bias_m_, -step, step);
        }
        const PathTracker::Output trk = tracker_.compute(v_now, search_bias_m_ + corner_bias_m_);
        if (trk.valid) {
            last_curv_ahead_ = trk.curv_ahead;
        }
        // Khoang cach diem ngam (tren duong tam, cach truc sau lookahead) toi
        // vach ngoai cua cua gat (loc nhe), cho chu ky sau
        {
            double d = -1.0;
            if (trk.valid && sharp_turn_ != 0) {
                d = dist_at_lookahead(trk.centre_v, trk.lookahead_m,
                                      sharp_turn_ < 0 ? trk.right_v : trk.left_v);
            }
            outer_d_m_ = d <= 0.0 ? -1.0 : outer_d_m_ <= 0.0 ? d : outer_d_m_ + 0.3 * (d - outer_d_m_);
        }
        const bool use_trk = trk.valid;
        // Loc goc banh (STEER_FILTER_SEC cua Python): nhieu vi tri vach vai cm
        // moi frame khong lam servo giat
        if (use_trk) {
            if (!steer_f_primed_) {
                steer_f_ = trk.steer_deg;
                steer_f_primed_ = true;
            } else {
                const double a = steer_filter_s_ > 0.0
                    ? std::min(1.0, static_cast<double>(dt_control_s_) / steer_filter_s_)
                    : 1.0;
                steer_f_ += a * (trk.steer_deg - steer_f_);
            }
        } else {
            steer_f_primed_ = false;
        }
        const int trk_dev = dev_for_wheel(steer_f_);

        // Dem so frame camera LIEN TIEP thay lan (dieu kien xuat phat). Ca 1
        // vach: camera chi bao ONE_LINE khi vach do bam on dinh vai frame. Truoc
        // day doi DU 2 vach -> xe dung o cho chi thay 1 vach la dung mai.
        if (!camera_ok) {
            lane_streak_ = 0;
        } else if (lane.frame_id != last_start_frame_) {
            last_start_frame_ = lane.frame_id;
            lane_streak_ = lane.state != LaneState::LOST ? lane_streak_ + 1 : 0;
        }
        {
            std::lock_guard<std::mutex> lock(trk_mtx_);
            trk_snapshot_ = trk;
        }

        // ---- Lenh chay tu GUI ----
        const long run_age = ms_since(run_seen_at_, t_now);
        const bool armed = !require_start_ ||
                           (run_requested_ && run_age >= 0 && run_age <= start_timeout_ms_);
        if (armed && !armed_) {
            // Vua bam chay: tinh lai dong ho mat lan, toc do ramp tu 0, cho
            // thay du 2 vach roi moi xuat phat
            lane_lost_since_ = t_now;
            lane_acquired_ = false;
            RCLCPP_INFO(get_logger(), "RUN: xe bat dau chay");
        } else if (!armed && armed_) {
            RCLCPP_WARN(get_logger(), "STOP: %s",
                        run_requested_ ? "mat heartbeat tu GUI" : "nguoi dung bam dung");
        }
        armed_ = armed;
        camera_->set_recording(armed_);

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
            // Du 2 vach: duong thang (speed_scale = 1) -> speed_x10 (ramp len),
            // vao cua noi suy ve speed_corner_x10 theo do cong (giam ngay)
            last_dev_px_ = use_trk ? trk_dev : camera_dev(lane.dev_px);
            lane_lost_since_ = t_now;
            track_ = 2;
            dev = last_dev_px_;
            target = lane_speed(speed_x10_, lane, trk, use_trk);
            dev_cm = static_cast<float>(dev_sign_) * lane.dev_cm;
            width_cm = lane.lane_width_cm;
            if (lane.lane_width_cm > 20.0f) {
                lane_w_m_ += 0.3 * (lane.lane_width_cm / 100.0 - lane_w_m_);
            }
        } else if (lane.state == LaneState::ONE_LINE) {
            // Chi thay 1 vach: lai theo duong tam suy tu vach do. Duong thang
            // + xe giua lan (vach kia mo / loa) -> toi speed_one_x10; cua hoac
            // xe lech -> ve speed_corner_x10.
            track_ = 1;
            lane_lost_since_ = t_now;
            last_dev_px_ = use_trk ? trk_dev : camera_dev(lane.dev_px);
            dev = last_dev_px_;
            target = lane_speed(speed_one_x10_, lane, trk, use_trk);
            dev_cm = static_cast<float>(dev_sign_) * lane.dev_cm;
        } else {
            // Mat ca 2 vach: giu huong lai, chay cham trong lane_lost_stop_ms
            track_ = 0;
            if (lane_lost_since_ == SteadyClock::time_point{}) {
                lane_lost_since_ = t_now;
            }
            const long lost_ms = ms_since(lane_lost_since_, t_now);
            if (use_trk && trk.ahead_m > 0.25 && lost_ms < lost_memory_ms_) {
                // Con duong da nho phia truoc (vd dang o giua cua gat, vach
                // ra khoi khung): chay tiep theo duong nho, toc do giu huong
                last_dev_px_ = trk_dev;
                dev = last_dev_px_;
                target = speed_hold_x10_;
            } else if (lost_ms >= lane_lost_stop_ms_) {
                last_dev_px_ = 0;
                emg = true;
                if (lane_acquired_) {
                    lane_acquired_ = false;
                    RCLCPP_WARN(get_logger(), "Mat lan %ld ms -> DUNG, thay lai lan %d frame se tu chay",
                                lost_ms, lane_start_frames_);
                }
            } else {
                dev = last_dev_px_;
                target = speed_hold_x10_;
            }
        }

        // Da bam chay nhung chua thay lan on dinh: dung cho
        if (armed_ && !lane_acquired_ && camera_ok) {
            if (lane_streak_ >= lane_start_frames_) {
                lane_acquired_ = true;
                lane_lost_since_ = t_now;
                RCLCPP_INFO(get_logger(), "Thay lan %d frame -> xe chay", lane_streak_);
            } else {
                emg = true;
                dev = 0;
            }
        }

        // Chua bam chay: dung xe, banh thang
        if (!armed_) {
            emg = true;
            dev = 0;
        }

        // Dang o cua gat: toc do cua (= toc do nho nhat); ra khoi cua moi tang
        // lai theo speed_ramp_x10
        if (sharp_turn_ != 0 && target > speed_corner_x10_) {
            target = speed_corner_x10_;
        }
        // Dang chay thi khong de lenh duoi speed_min_x10 (banh se khong quay)
        if (!emg && target > 0) {
            target = std::max(target, speed_min_x10_);
        }

        // Toc do: tang dan (speed_ramp_x10_ don vi x10 moi giay) khi het cua,
        // giam nhanh nhung DEU (speed_down_x10_ moi giay) khi vao cua -> khong
        // giat (truoc day giam tuc thi).
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
            speed_cur_x10_f_ = std::max(static_cast<float>(target),
                                        speed_cur_x10_f_ - speed_down_x10_ * dt_control_s_);
        }
        const int speed = static_cast<int>(speed_cur_x10_f_ + 0.5f);

        // Gioi han toc do danh lai tren LENH CUOI (moi nhanh: tracker, dev
        // camera, giu huong). Chay 10/10 19:37: gioi han dat sau bo loc tracker
        // bi bo qua moi khi tracker mat 1 nhip / dung dev camera -> banh van
        // nhay 10-15 do trong 1 frame. Cua = do cong phia truoc 0.4 -> 1.0 1/m
        // (nhu lookahead/corner_gain), hoac thay goc gap phia truoc, dang o
        // che do cua gat, rao chan vach dang day -> toc do cua. Xe dung (emg)
        // -> khong gioi han (banh ve thang ngay).
        {
            const double dt = steer_lim_at_ == SteadyClock::time_point{}
                ? static_cast<double>(dt_control_s_)
                : std::clamp(std::chrono::duration<double>(t_now - steer_lim_at_).count(), 0.0, 0.05);
            steer_lim_at_ = t_now;
            const double want = (servo_straight_deg_ - estimate_servo_deg(dev)) * steer_ratio_ * dev_sign_;
            if (emg || steer_rate_dps_ <= 0.0) {
                steer_lim_deg_ = want;
            } else {
                double rc = std::clamp((std::fabs(last_curv_ahead_) - 0.4) / 0.6, 0.0, 1.0);
                if (use_trk && (sharp_turn_ != 0 || trk.kink_dist_m >= 0.0 || trk.guard_deg != 0.0)) {
                    rc = 1.0;
                }
                // PID duong thang dang lai (pid_w = 1) moi la "thang"; giua cua
                // gap do cong ~0 nhung pid_w = 0 -> toc do cua
                if (use_trk && tracker_.params().pid_kp_deg_per_m > 0.0) {
                    rc = std::max(rc, 1.0 - trk.pid_w);
                }
                const double rate = steer_rate_dps_ +
                                    (std::max(steer_rate_corner_dps_, steer_rate_dps_) - steer_rate_dps_) * rc;
                const double step = rate * dt;
                steer_lim_deg_ += std::clamp(want - steer_lim_deg_, -step, step);
                dev = dev_for_wheel(steer_lim_deg_);
            }
        }

        // Goc banh ung voi lenh nay (cho odometry chu ky sau)
        wheel_cmd_deg_ = emg && !armed_ ? 0.0
            : (servo_straight_deg_ - estimate_servo_deg(dev)) * steer_ratio_ * dev_sign_;

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
        pitch_deg_ = lane.pitch_deg;
        near_z_m_ = lane.near_z_m;
        curvature_ = lane.curvature;
        speed_scale_ = lane.speed_scale;
        age_ms_ = lane.age_ms;
        proc_ms_ = lane.proc_ms;
        frame_id_ = lane.frame_id;
        frame_w_ = lane.frame_w;
        frame_h_ = lane.frame_h;
        emg_ = emg;
        trk_valid_ = use_trk;
        trk_turn_ = trk.turn;
        trk_ahead_m_ = trk.ahead_m;
        trk_la_m_ = trk.lookahead_m;
        trk_wheel_deg_ = trk.steer_deg;
        trk_guard_deg_ = trk.guard_deg;
        trk_xte_m_ = trk.xte_m;
        trk_xte_i_deg_ = trk.xte_i_deg;
        trk_pid_w_ = trk.pid_w;
        trk_pid_e_m_ = trk.pid_e_m;
        trk_pid_psi_deg_ = trk.pid_psi_deg;
        trk_pid_i_deg_ = trk.pid_i_deg;
        trk_pid_deg_ = trk.pid_deg;
        trk_corner_m_ = trk.corner_dist_m;
        trk_kink_m_ = trk.kink_dist_m;
        trk_kink_deg_ = trk.kink_deg;
        trk_kink_hold_ = trk.kink_hold;

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

        // Ve ban do nho len BAN SAO (lane.vis dung chung voi luong camera)
        if (want_vis || want_jpg) {
            PathTracker::Output t;
            {
                std::lock_guard<std::mutex> lock(trk_mtx_);
                t = trk_snapshot_;
            }
            lane.vis = lane.vis.clone();
            draw_bev(lane.vis, t, tracker_.params().wheelbase_m,
                     tracker_.params().cam_to_rear_m, near_z_m_);
        }

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
        char buf[960];
        std::snprintf(
            buf, sizeof(buf),
            "two_lanes=%d track=%s dev=%d emg=%d run=%d age=%ldms proc=%.1fms "
            "fps=%.1f cam=%dx%d gate=%d lidar=%s front=%.0fcm left=%.0fcm "
            "right=%.0fcm rear=%.0fcm alert=%s lofs=%.1f serial=%s w=%.0fcm "
            "devm=%.1fcm curv=%.3f scale=%.2f spd=%.1f kmh=%.2f fbage=%ld "
            "servo=%.1f hz=%.2f pitch=%.1f esc=%d fwemg=%d fwwd=%d fwcmd=%d "
            "mode=%s turn=%c mem=%.2f la=%.2f wheel=%.1f guard=%.1f xte=%.1fcm xi=%.1f corner=%.2f vest=%.1f cob=%.1fcm sharp=%c od=%.0fcm kink=%.2f/%.0f%s "
            "pidw=%.2f pide=%.1fcm pidpsi=%.1f pidi=%.1f pid=%.1f",
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
            pitch_deg_,
            // Muc xung ESC ESP32 dang phat (90 = neutral), -1 = firmware cu / mat lien lac
            (fb.has_v2 && fb_age < 300) ? fb.esc_deg : -1,
            (fb.has_v2 && fb.fw_emg) ? 1 : 0,
            (fb.has_v2 && fb.fw_watchdog) ? 1 : 0,
            (fb.has_v2 && fb.fw_got_cmd) ? 1 : 0,
            trk_valid_ ? "track" : "camera",
            trk_turn_ > 0 ? 'R' : (trk_turn_ < 0 ? 'L' : 'S'),
            trk_ahead_m_, trk_la_m_, trk_wheel_deg_, trk_guard_deg_,
            trk_xte_m_ * 100.0, trk_xte_i_deg_, trk_corner_m_, v_odom_x10_ / 10.0, corner_bias_m_ * 100.0,
            sharp_turn_ < 0 ? 'L' : sharp_turn_ > 0 ? 'R' : '-', outer_d_m_ * 100.0,
            trk_kink_m_, trk_kink_deg_, trk_kink_hold_ ? "H" : "",
            trk_pid_w_, trk_pid_e_m_ * 100.0, trk_pid_psi_deg_, trk_pid_i_deg_, trk_pid_deg_);

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

    int speed_x10_ = 70;
    int speed_hold_x10_ = 58;
    int speed_corner_x10_ = 58;
    double coast_decel_mps2_ = 0.45; // gia toc giam khi nha ga (m/s^2)
    double corner_margin_m_ = 0.20;  // xuong toc do cua truoc cua bay nhieu m
    double v_odom_x10_ = 0.0;        // toc do that uoc luong (km/h x10)
    int speed_one_x10_ = 60;
    float speed_ramp_x10_ = 12.0f;   // don vi x10 moi giay (12 = 1.2 km/h/s)
    float speed_down_x10_ = 20.0f;   // giam toc vao cua, don vi x10 moi giay
    int speed_min_x10_ = 58;         // toc do nho nhat khi dang chay
    float speed_cur_x10_f_ = 0.0f;   // lenh hien tai, tang dan khi ramp
    float dt_control_s_ = 0.01f;     // 1 / control_hz
    double steer_filter_s_ = 0.08;   // loc goc banh (s)
    double steer_f_ = 0.0;           // goc banh da loc (do)
    bool steer_f_primed_ = false;
    double steer_lim_deg_ = 0.0;     // goc banh lenh sau gioi han toc do (do)
    SteadyClock::time_point steer_lim_at_{};
    double steer_rate_dps_ = 15.0;          // toc do danh lai duong thang (do banh / s)
    double steer_rate_corner_dps_ = 45.0;   // toc do danh lai trong cua
    int lane_start_frames_ = 3;      // so frame thay lan lien tiep de xuat phat
    int lane_streak_ = 0;
    double single_search_m_ = 0.08;
    double single_search_rate_ = 0.15;
    double search_bias_m_ = 0.0;
    double corner_outer_gap_m_ = 0.08; // mep xe -> mep vach ngoai trong cua gat (m), < 0 = tat
    double corner_outer_max_m_ = 0.15; // doi ra ngoai toi da (m)
    double corner_outer_rate_ = 0.40;  // toc do doi (m/s)
    double corner_sharp_k_ = 1.0;      // do cong vao trang thai cua gat (1/m)
    static constexpr double CORNER_SHARP_HOLD_S = 0.15; // giu tren nguong bay lau moi bat
    static constexpr double CORNER_SHARP_IN_DEG = 8.0;  // goc banh cung chieu de bat (do)
    static constexpr double CORNER_SHARP_OUT_DEG = 6.0; // banh duoi muc nay ...
    static constexpr double CORNER_SHARP_EXIT_S = 0.2;  // ... bay lau thi tat
    static constexpr double CORNER_SHARP_REV = 0.4;     // duong cong nguoc chieu (1/m) -> tat
    static constexpr double CORNER_IN_MAX_M = 0.05;     // doi VAO trong toi da khi qua sat vach ngoai
    int sharp_turn_ = 0;               // dang o cua gat: -1 trai, +1 phai
    double sharp_t_s_ = 0.0;           // dem thoi gian dieu kien bat / tat
    double wheel_now_deg_ = 0.0;       // goc banh dang ap dung (do, > 0 = phai)
    double outer_d_m_ = -1.0;          // diem ngam -> vach ngoai (m), < 0 = chua do
    double corner_bias_m_ = 0.0;       // dang doi duong bam (m, > 0 = sang phai)
    double lane_w_m_ = 0.55;           // be rong lan do gan nhat (m, EMA)
    double last_curv_ahead_ = 0.0;   // do cong phia truoc chu ky truoc (1/m)
    long last_start_frame_ = -1;
    bool lane_acquired_ = false;     // da thay du 2 vach tu luc bam chay
    int dev_sign_ = 1;
    int lane_lost_stop_ms_ = 2500;
    SteadyClock::time_point lane_lost_since_{};
    SteadyClock::time_point last_reconnect_at_{};

    // ---- Gia tri control_tick ghi, status_tick doc (cung nhom callback) ----
    int track_ = 0;              // 0 = mat vach, 1 = 1 vach, 2 = du 2 vach
    int last_dev_px_ = 0;
    int last_cmd_dev_ = 0;
    bool two_lanes_ = false;
    bool gated_ = false;
    float horizon_frac_ = 0.0f;
    float pitch_deg_ = 0.0f;
    float near_z_m_ = 0.15f;
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

    // ---- PathTracker ----
    PathTracker tracker_;
    double steer_ratio_ = 0.6;
    double odom_speed_scale_ = 0.30;
    double servo_straight_deg_ = 90.0;   // goc servo ma banh di thang
    int lost_memory_ms_ = 1500;
    double wheel_cmd_deg_ = 0.0;   // goc banh cua lenh vua gui (> 0 phai)
    long last_obs_frame_ = -1;
    std::mutex trk_mtx_;           // trk_snapshot_: control ghi, viz doc
    PathTracker::Output trk_snapshot_;
    bool trk_valid_ = false;
    int trk_turn_ = 0;
    double trk_ahead_m_ = 0.0;
    double trk_la_m_ = 0.0;
    double trk_wheel_deg_ = 0.0;
    double trk_guard_deg_ = 0.0;
    double trk_xte_m_ = 0.0;
    double trk_xte_i_deg_ = 0.0;
    double trk_pid_w_ = 0.0;
    double trk_pid_e_m_ = 0.0;
    double trk_pid_psi_deg_ = 0.0;
    double trk_pid_i_deg_ = 0.0;
    double trk_pid_deg_ = 0.0;
    double trk_corner_m_ = -1.0;
    double trk_kink_m_ = -1.0;
    double trk_kink_deg_ = 0.0;
    bool trk_kink_hold_ = false;

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
