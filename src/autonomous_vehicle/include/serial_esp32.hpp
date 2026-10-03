#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
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

    bool valid = false;
};

// ============================================================================
// SERIAL ESP32
// ============================================================================

class SerialESP32 {

public:

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
        const std::string& port =
            "/dev/ttyUSB0"
    );

    ~SerialESP32();

    bool open();

    void close();

    bool is_open() const;

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

    // Tuổi của telemetry gần nhất (ms). ESP32 gửi ở ~50 Hz nên giá trị > 200
    // nghĩa là đã mất liên lạc -> node điều khiển phải dừng xe.
    unsigned long feedback_age_ms() const;

private:

    // ------------------------------------------------------------------------
    // Serial resource
    // ------------------------------------------------------------------------

    std::string port_;

    int fd_ = -1;

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

    // std::chrono::steady_clock::time_point của gói telemetry hợp lệ cuối
    std::chrono::steady_clock::time_point
        last_feedback_time_{};

    // Nguồn thời gian cho feedback_age_ms()
    static std::chrono::steady_clock::time_point steady_now();

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