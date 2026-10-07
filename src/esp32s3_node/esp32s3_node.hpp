#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

// ============================================================================
// GIAO TIEP MINI PC <-> ESP32 QUA UART (SERIAL)
// ----------------------------------------------------------------------------
// ESP32 noi voi Mini PC qua chip USB-UART CP2102 tren board -> Linux thay
// /dev/ttyUSB* (KHONG dung USB-OTG / CDC nen khong co /dev/ttyACM*).
// Mac dinh tren xe: ESP32 = /dev/ttyUSB0, LiDAR = /dev/ttyUSB1.
//
// LiDAR RPLIDAR A1 cung dung CP2102 va cung so serial "0001" nen ten trong
// /dev/serial/by-id KHONG phan biet duoc 2 thiet bi. Vi vay khi phai tu do,
// driver mo tung cong va nghe goi telemetry 0xDC 0xBA cua ESP32: cong nao
// gui dung goi la ESP32 (LiDAR im lang cho toi khi duoc ra lenh quet).
//
// Khung truyen 230400 baud, 8N1, khong flow control.
// ============================================================================

// ============================================================================
// MINI PC -> ESP32 : 11 byte
//   AB CD | DEV_H DEV_L | SPEED | EMG | 00 00 00 00 | XOR(byte 2..9)
// ============================================================================

struct SerialCommand {
    // Do lech tam lan, px anh tham chieu 640 (xem camera_node.hpp)
    int16_t dev_final_px = 0;

    // Toc do dat = km/h x 10 (vd 5.5 km/h -> 55)
    uint8_t speed_control = 0;

    // 1 = dung xe (firmware ghi ESC neutral). KHONG dung speed = 0 de dung.
    bool emergency_stop = false;
};

// ============================================================================
// ESP32 -> MINI PC, ~50 Hz. Nhan ca 2 dang:
//   v1, 7 byte : DC BA | float32 velocity_kmh | XOR(byte 2..5)
//   v2, 10 byte: DC BB | float32 velocity_kmh | ESC_DEG | STEER_DEG | FLAGS |
//                XOR(byte 2..8)
// v2 cho biet muc xung ESC/servo ESP32 THAT SU dang phat -> biet lenh chay co
// toi ESC khong.
// ============================================================================

struct ESP32Feedback {
    float velocity_kmh = 0.0f;
    bool valid = false;

    // Chi co khi firmware gui v2 (has_v2 = true)
    bool has_v2 = false;
    int esc_deg = -1;      // goc Servo ghi cho ESC: 90 neutral, 95..180 tien
    int steer_deg = -1;    // goc servo lai
    bool fw_emg = false;   // ESP32 dang o che do dung
    bool fw_watchdog = false;  // ESP32 mat lenh > 500 ms
    bool fw_got_cmd = false;   // ESP32 da nhan it nhat 1 goi lenh hop le
};

// Mo cong `path`, nghe toi da timeout_ms xem co goi telemetry hop le cua
// ESP32 khong. Khong gui gi xuong ESP32.
bool probe_esp32(const std::string& path, int timeout_ms = 700);

// Tim cong ESP32 trong /dev/ttyUSB* va /dev/ttyACM* bang probe_esp32().
// exclude: bo qua cong da biet la cua thiet bi khac (vd LiDAR).
// Tra ve chuoi rong neu khong tim thay.
std::string autodetect_port(const std::string& exclude = "");

// ============================================================================
// SERIAL ESP32
// ============================================================================

class SerialESP32 {

public:

    static constexpr int BAUDRATE = 230400;

    static constexpr uint8_t TX_HEADER_1 = 0xAB;   // MiniPC -> ESP32
    static constexpr uint8_t TX_HEADER_2 = 0xCD;
    static constexpr uint8_t RX_HEADER_1 = 0xDC;   // ESP32 -> MiniPC
    static constexpr uint8_t RX_HEADER_2 = 0xBA;

    static constexpr size_t TX_PACKET_LEN = 11;
    static constexpr uint8_t RX_HEADER_2_V2 = 0xBB;
    static constexpr size_t RX_PACKET_LEN = 7;
    static constexpr size_t RX_PACKET_LEN_V2 = 10;

    // port: duong dan cong. Rong hoac cong khong ton tai -> tu do.
    // exclude: cong khong duoc chon khi tu do (cong LiDAR).
    explicit SerialESP32(const std::string& port = "",
                         const std::string& exclude = "");

    ~SerialESP32();

    bool open();
    void close();
    bool is_open() const;

    // Cong da bi loi I/O (rut cap, mat nguon) -> nen close() + open() lai
    bool is_broken() const { return broken_.load(); }

    const std::string& port() const { return port_; }

    // Gui 1 lenh. Khong bao gio chan (fd o che do non-blocking).
    bool send_command(const SerialCommand& cmd);

    ESP32Feedback get_latest_feedback() const;

    // Tuoi telemetry gan nhat (ms), -1 (unsigned) neu chua co. ESP32 gui
    // ~50 Hz nen > 200 ms nghia la mat lien lac.
    unsigned long feedback_age_ms() const;

private:

    std::string port_;
    std::string requested_port_;
    std::string exclude_;

    int fd_ = -1;

    // So lan open() that bai lien tiep: chi in loi o lan dau, tranh ngap log
    // khi node thu mo lai moi giay trong luc cap dang rut
    int fail_streak_ = 0;

    // Bao ve fd_, send_command(), open()/close(). Luong RX KHONG giu mutex
    // nay trong luc cho du lieu.
    mutable std::mutex mtx_;

    std::thread feedback_thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> broken_{false};

    mutable std::mutex feedback_mtx_;
    ESP32Feedback latest_feedback_{};
    std::chrono::steady_clock::time_point last_feedback_time_{};

    // ---- Bo phan tich goi RX ----
    enum class RxState : uint8_t { WAIT_H1, WAIT_H2, READ_PAYLOAD };

    RxState rx_state_ = RxState::WAIT_H1;
    uint8_t rx_packet_[RX_PACKET_LEN_V2] = {};
    uint8_t rx_idx_ = 0;
    uint8_t rx_len_ = RX_PACKET_LEN;   // do dai goi dang doc (7 hoac 10)

    void feedback_loop();
    void process_rx_byte(uint8_t b);
};
