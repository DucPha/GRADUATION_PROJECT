#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include <termios.h>

// ============================================================================
// MINI PC -> ESP32
// 11 bytes
//
// AB CD | DEV_H DEV_L | SPEED | EMG | 00 00 00 00 | XOR
// ============================================================================

struct SerialCommand {
    int16_t dev_final_px = 0;

    // Desired speed = km/h × 10
    // Example:
    // 5.5 km/h -> 55
    uint8_t speed_control = 0;

    bool emergency_stop = false;
};

// ============================================================================
// ESP32 -> MINI PC
// 7 bytes
//
// DC BA | FLOAT0 FLOAT1 FLOAT2 FLOAT3 | XOR
// ============================================================================

struct ESP32Feedback {
    float velocity_kmh = 0.0f;
    uint64_t timestamp_us = 0;  // Microseconds since epoch when received

    bool valid = false;
    
    // Check if feedback is stale (older than max_age_ms)
    bool is_stale(uint32_t max_age_ms = 100) const {
        if (!valid) return true;
        auto now = std::chrono::steady_clock::now();
        auto age_us = std::chrono::duration_cast<std::chrono::microseconds>(
            now.time_since_epoch()).count() - timestamp_us;
        return age_us > static_cast<int64_t>(max_age_ms * 1000);
    }
};

// ============================================================================
// SERIAL ESP32
// ============================================================================

class SerialESP32 {

public:

    // Firmware ESP32 khóa cứng 230400 (xem Autonomous_Vehicle.ino). Giá trị
    // này chỉ để tương thích: đổi ở PC mà không đổi firmware sẽ mất liên lạc.
    static constexpr int BAUDRATE = 230400;

    // MiniPC -> ESP32
    static constexpr uint8_t TX_HEADER_1 = 0xAB;
    static constexpr uint8_t TX_HEADER_2 = 0xCD;

    // ESP32 -> MiniPC
    static constexpr uint8_t RX_HEADER_1 = 0xDC;
    static constexpr uint8_t RX_HEADER_2 = 0xBA;

    static constexpr size_t TX_PACKET_LEN = 11;
    static constexpr size_t RX_PACKET_LEN = 7;

    explicit SerialESP32(
        const std::string& port = "/dev/ttyESP32"
    );

    ~SerialESP32();

    bool open();

    void close();

    bool is_open() const;

    // Baudrate gửi xuống ESP32 (thông số ROS `baudrate`). Firmware đang khóa
    // cứng Serial.begin(230400); tham số này chỉ hữu ích khi firmware cũng
    // đổi theo.
    void set_baudrate(int baudrate) { baudrate_ = baudrate; }
    int baudrate() const { return baudrate_; }

    // ------------------------------------------------------------------------
    // MiniPC -> ESP32
    // ------------------------------------------------------------------------

    bool send_command(
        const SerialCommand& cmd
    );

    // ------------------------------------------------------------------------
    // Latest valid telemetry
    // ------------------------------------------------------------------------

    ESP32Feedback
    get_latest_feedback() const;

    // true nếu luồng đọc gặp lỗi (EIO/EPIPE) => nhiều khả năng ESP32 đã bị
    // rút hoặc driver USB serial reset. Node điều khiển dùng cờ này để phanh
    // và thử mở lại cổng.
    bool link_down() const { return link_down_.load(); }

    // Xoá cờ lỗi sau khi mở lại cổng thành công.
    void clear_link_down() {
        link_down_.store(false);
        read_error_count_.store(0);
    }

private:

    // ------------------------------------------------------------------------
    // Serial resource
    // ------------------------------------------------------------------------

    std::string port_;

    int baudrate_ = BAUDRATE;

    int fd_ = -1;

    // Số lần liên tiếp không đọc được byte nào trên UART. ESP32 bị rút, hoặc
    // driver reset, sẽ làm read() trả lỗi EIO; nếu chỉ im lặng thì xe vẫn
    // nhận lệnh (không ai nhận phản hồi) mà HUD vẫn báo "OK".
    std::atomic<int> read_error_count_{0};
    std::atomic<bool> link_down_{false};

    // Protect:
    // - fd_
    // - send_command()
    // - open()/close()
    //
    // RX thread does NOT hold this while waiting for data.
    mutable std::mutex mtx_;

    // ------------------------------------------------------------------------
    // Background RX thread
    // ------------------------------------------------------------------------

    std::thread feedback_thread_;

    std::atomic<bool>
        running_{false};

    // ------------------------------------------------------------------------
    // Latest feedback
    // ------------------------------------------------------------------------

    mutable std::mutex
        feedback_mtx_;

    ESP32Feedback
        latest_feedback_{};

    // ------------------------------------------------------------------------
    // RX parser
    // ------------------------------------------------------------------------

    enum class RxState : uint8_t {
        WAIT_H1,
        WAIT_H2,
        READ_PAYLOAD
    };

    RxState
        rx_state_ =
            RxState::WAIT_H1;

    uint8_t
        rx_packet_[RX_PACKET_LEN] = {};

    uint8_t
        rx_idx_ = 0;

    // ------------------------------------------------------------------------
    // Internal functions
    // ------------------------------------------------------------------------

    uint8_t calculate_checksum(
        const uint8_t* data,
        size_t len
    ) const;

    bool configure_port();

    void feedback_loop();

    void process_rx_byte(
        uint8_t b
    );
};