#include "serial_esp32.hpp"

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <string>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

// ============================================================================
// CONSTRUCTOR / DESTRUCTOR
// ============================================================================
SerialESP32::SerialESP32(const std::string& port)
    : port_(port) {}

SerialESP32::~SerialESP32() {
    close();
}

bool SerialESP32::is_open() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return fd_ >= 0;
}

// ============================================================================
// OPEN & CLOSE
// ============================================================================
bool SerialESP32::open() {
    std::lock_guard<std::mutex> lock(mtx_);
    if (fd_ >= 0) return true;

    // Mở cổng Serial: Đọc/Ghi, Không chiếm quyền Terminal, Đồng bộ
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
        std::cerr << "[SerialESP32] Error opening " << port_ << ": " << strerror(errno) << "\n";
        return false;
    }

    if (!configure_port()) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    // Bật cờ chạy và khởi tạo luồng đọc ngầm
    running_ = true;
    feedback_thread_ = std::thread(&SerialESP32::feedback_loop, this);

    std::cout << "[SerialESP32] Opened " << port_ << " successfully (fd=" << fd_ << ")\n";
    return true;
}

void SerialESP32::close() {
    // Tắt cờ trước để luồng ngầm tự thoát
    running_ = false;
    
    if (feedback_thread_.joinable()) {
        feedback_thread_.join();
    }

    // Sau khi luồng ngầm dừng hẳn mới đóng File Descriptor
    std::lock_guard<std::mutex> lock(mtx_);
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
        std::cout << "[SerialESP32] Closed gracefully.\n";
    }
}

// ============================================================================
// HÀM CẤU HÌNH CỔNG (OS LEVEL)
// ============================================================================
bool SerialESP32::configure_port() {
    struct termios tty;
    if (tcgetattr(fd_, &tty) != 0) {
        std::cerr << "[SerialESP32] tcgetattr error: " << strerror(errno) << "\n";
        return false;
    }

    cfsetospeed(&tty, BAUDRATE);
    cfsetispeed(&tty, BAUDRATE);

    // 8N1
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS; // No hardware flow control
    tty.c_cflag |= CREAD | CLOCAL;

    // Tắt các tính năng can thiệp ký tự của Linux
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    // Dùng Non-blocking mode cho read (select sẽ lo việc block an toàn)
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        std::cerr << "[SerialESP32] tcsetattr error: " << strerror(errno) << "\n";
        return false;
    }

    // Tắt DTR / RTS để tránh mạch ESP32 bị Auto-Reset khi mở cổng
    int status;
    if (ioctl(fd_, TIOCMGET, &status) == 0) {
        status &= ~TIOCM_DTR;
        status &= ~TIOCM_RTS;
        ioctl(fd_, TIOCMSET, &status);
    }

    tcflush(fd_, TCIOFLUSH);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    return true;
}

// ============================================================================
// GỬI LỆNH XUỐNG ESP32 (TX)
// ============================================================================
bool SerialESP32::send_command(const SerialCommand& cmd) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (fd_ < 0) return false;

    uint8_t packet[TX_PACKET_LEN];
    
    // Header
    packet[0] = TX_HEADER_1; // 0xAB
    packet[1] = TX_HEADER_2; // 0xCD
    
    // Payload (Bytes 2 to 9)
    packet[2] = (cmd.dev_final_px >> 8) & 0xFF; // MSB
    packet[3] = cmd.dev_final_px & 0xFF;        // LSB
    packet[4] = cmd.speed_control;              // Vận tốc (km/h * 10)
    packet[5] = cmd.emergency_stop ? 1 : 0;     // Phanh khẩn cấp
    packet[6] = 0x00; // Padding
    packet[7] = 0x00; // Padding
    packet[8] = 0x00; // Padding
    packet[9] = 0x00; // Padding
    
    // Checksum (XOR từ byte 2 đến byte 9)
    packet[10] = calculate_checksum(&packet[2], 8);

    ssize_t written = ::write(fd_, packet, TX_PACKET_LEN);
    return (written == TX_PACKET_LEN);
}

// ============================================================================
// LẤY TRẠNG THÁI (AI LUỒNG CHÍNH GỌI)
// ============================================================================
ESP32Feedback SerialESP32::get_latest_feedback() const {
    std::lock_guard<std::mutex> lock(feedback_mtx_);
    return latest_feedback_;
}

std::chrono::steady_clock::time_point SerialESP32::steady_now() {
    return std::chrono::steady_clock::now();
}

unsigned long SerialESP32::feedback_age_ms() const {
    std::lock_guard<std::mutex> lock(feedback_mtx_);
    if (!latest_feedback_.valid) {
        return static_cast<unsigned long>(-1);
    }
    const auto age = std::chrono::steady_clock::now() - last_feedback_time_;
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(age).count();
    return ms < 0 ? 0ul : static_cast<unsigned long>(ms);
}

// ============================================================================
// LUỒNG NGẦM ĐỌC DATA (RX THREAD)
// ============================================================================
void SerialESP32::feedback_loop() {
    fd_set read_fds;
    struct timeval timeout;

    while (running_) {
        // Lấy fd_ an toàn. Không giữ mtx_ trong lúc chờ io (Đúng chuẩn thiết kế)
        int current_fd = -1;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            current_fd = fd_;
        }

        if (current_fd < 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        FD_ZERO(&read_fds);
        FD_SET(current_fd, &read_fds);

        // Chờ 10ms (Không ăn CPU)
        timeout.tv_sec = 0;
        timeout.tv_usec = 10000; 

        int ret = select(current_fd + 1, &read_fds, nullptr, nullptr, &timeout);
        
        if (ret > 0 && FD_ISSET(current_fd, &read_fds)) {
            uint8_t buf[64];
            ssize_t bytes_read = ::read(current_fd, buf, sizeof(buf));
            
            if (bytes_read > 0) {
                // Nhồi byte nhận được vào State Machine
                for (ssize_t i = 0; i < bytes_read; ++i) {
                    process_rx_byte(buf[i]);
                }
            }
        }
    }
}

// ============================================================================
// RX STATE MACHINE (MÁY TRẠNG THÁI)
// ============================================================================
void SerialESP32::process_rx_byte(uint8_t b) {
    switch (rx_state_) {
        case RxState::WAIT_H1: // Đợi 0xDC
            if (b == RX_HEADER_1) {
                rx_packet_[0] = b;
                rx_idx_ = 1;
                rx_state_ = RxState::WAIT_H2;
            }
            break;

        case RxState::WAIT_H2: // Đợi 0xBA
            if (b == RX_HEADER_2) {
                rx_packet_[1] = b;
                rx_idx_ = 2;
                rx_state_ = RxState::READ_PAYLOAD;
            } else if (b == RX_HEADER_1) {
                // Rơi nhầm byte, nhưng lại trùng header 1
                rx_packet_[0] = b;
                rx_idx_ = 1;
            } else {
                rx_state_ = RxState::WAIT_H1;
            }
            break;

        case RxState::READ_PAYLOAD: // Đọc tiếp 5 bytes
            rx_packet_[rx_idx_++] = b;

            if (rx_idx_ == RX_PACKET_LEN) { // Đã nhận đủ 7 bytes
                // Tính Checksum XOR từ 4 bytes float (Vị trí 2 -> 5)
                uint8_t calc_csum = calculate_checksum(&rx_packet_[2], 4);
                
                // Vị trí 6 là Checksum do ESP32 gửi lên
                if (calc_csum == rx_packet_[6]) {
                    ESP32Feedback fb;
                    // Ép 4 bytes vào kiểu Float
                    std::memcpy(&fb.velocity_kmh, &rx_packet_[2], sizeof(float));
                    fb.valid = std::isfinite(fb.velocity_kmh);

                    // Gói có checksum đúng nhưng float NaN/inf -> coi như hỏng,
                    // không ghi đè telemetry hợp lệ đang có.
                    if (fb.valid) {
                        std::lock_guard<std::mutex> lock(feedback_mtx_);
                        latest_feedback_ = fb;
                        last_feedback_time_ = steady_now();
                    }
                }
                
                // Quay lại đợi gói tiếp theo
                rx_state_ = RxState::WAIT_H1;
            }
            break;
    }
}

// ============================================================================
// UTILS
// ============================================================================
uint8_t SerialESP32::calculate_checksum(const uint8_t* data, size_t len) const {
    uint8_t checksum = 0;
    for (size_t i = 0; i < len; ++i) {
        checksum ^= data[i];
    }
    return checksum;
}