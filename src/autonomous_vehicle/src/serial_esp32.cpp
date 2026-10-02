#include "serial_esp32.hpp"

#include <iostream>
#include <chrono>
#include <cmath>
#include <cstring>
#include <sys/ioctl.h>
#include <poll.h>
#include <cerrno>
#include <fcntl.h>
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

    // O_NONBLOCK: Không chặn luồng I/O
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        std::cerr << "[SerialESP32] Error opening " << port_ << ": " << strerror(errno) << "\n";
        return false;
    }

    if (!configure_port()) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    running_ = true;
    feedback_thread_ = std::thread(&SerialESP32::feedback_loop, this);

    std::cout << "[SerialESP32] Opened " << port_ << " successfully (fd=" << fd_ << ")\n";
    return true;
}

void SerialESP32::close() {
    running_ = false;

    if (feedback_thread_.joinable()) {
        feedback_thread_.join();
    }

    std::lock_guard<std::mutex> lock(mtx_);
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
        std::cout << "[SerialESP32] Closed gracefully.\n";
    }
}

// ============================================================================
// CẤU HÌNH CỔNG SERIAL (OS LEVEL - TỐI ƯU REAL-TIME)
// ============================================================================
bool SerialESP32::configure_port() {
    struct termios tty;
    if (tcgetattr(fd_, &tty) != 0) {
        std::cerr << "[SerialESP32] tcgetattr error: " << strerror(errno) << "\n";
        return false;
    }

    // Set Baudrate chuẩn 230400 (khớp với firmware ESP32)
    cfsetospeed(&tty, B230400);
    cfsetispeed(&tty, B230400);

    // 8N1 - 8 data bits, no parity, 1 stop bit
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS; // Không dùng flow control phần cứng
    tty.c_cflag |= CREAD | CLOCAL;

    // Chuyển sang chế độ RAW THUẦN TÚY (Tắt toàn bộ xử lý ký tự của Linux kernel)
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    // Non-blocking thuần: read() trả về ngay lập tức nếu không có dữ liệu
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        std::cerr << "[SerialESP32] tcsetattr error: " << strerror(errno) << "\n";
        return false;
    }

    // Tắt DTR và RTS để chống kích hoạt mạch Auto-Reset (DTR/RTS dập chân EN/RST của ESP32)
    int status;
    if (ioctl(fd_, TIOCMGET, &status) == 0) {
        status &= ~TIOCM_DTR;
        status &= ~TIOCM_RTS;
        ioctl(fd_, TIOCMSET, &status);
    }

    // Xóa sạch rác trong buffer phần cứng trước khi chạy
    tcflush(fd_, TCIOFLUSH);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    return true;
}

// ============================================================================
// GỬI LỆNH XUỐNG ESP32 (TX - ZERO LATENCY)
// ============================================================================
bool SerialESP32::send_command(const SerialCommand& cmd) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (fd_ < 0) return false;

    uint8_t packet[TX_PACKET_LEN];

    // Header: AB CD
    packet[0] = TX_HEADER_1;
    packet[1] = TX_HEADER_2;

    // Payload (Bytes 2 đến 9)
    packet[2] = (cmd.dev_final_px >> 8) & 0xFF; // DEV_H
    packet[3] = cmd.dev_final_px & 0xFF;        // DEV_L
    packet[4] = cmd.speed_control;              // SPEED (km/h * 10)
    packet[5] = cmd.emergency_stop ? 1 : 0;     // EMG
    packet[6] = 0x00;                           // Padding
    packet[7] = 0x00;                           // Padding
    packet[8] = 0x00;                           // Padding
    packet[9] = 0x00;                           // Padding

    // Checksum: XOR từ byte 2 đến byte 9
    packet[10] = calculate_checksum(&packet[2], 8);

    // Ghi trực tiếp xuống UART không đệm trễ
    ssize_t written = ::write(fd_, packet, TX_PACKET_LEN);

    if (written < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
            std::cerr << "[SerialESP32] write() failed: " << strerror(errno) << "\n";
            read_error_count_.fetch_add(1);
            link_down_.store(true);
        }
        return false;
    }

    // Ghi ngắn (written < TX_PACKET_LEN) nghĩa là dòng không nuốt hết hoặc bị
    // ngắt giữa chừng. Frame bị cắt cụt đều là frame hỏng -> coi như mất liên
    // kết, không phải chỉ "lệnh chưa gửi xong".
    if (written < static_cast<ssize_t>(TX_PACKET_LEN)) {
        std::cerr << "[SerialESP32] short write: " << written << "/"
                  << TX_PACKET_LEN << " bytes\n";
        read_error_count_.fetch_add(1);
        link_down_.store(true);
        return false;
    }

    return true;
}

// ============================================================================
// LẤY TRẠNG THÁI (CHO ROS NODE ĐỌC)
// ============================================================================
ESP32Feedback SerialESP32::get_latest_feedback() const {
    std::lock_guard<std::mutex> lock(feedback_mtx_);
    return latest_feedback_;
}

// ============================================================================
// LUỒNG NGẦM ĐỌC DATA (RX THREAD) - EVENT-DRIVEN, VÉT CẠN BUFFER, NO SLEEP
// ============================================================================
void SerialESP32::feedback_loop() {
    struct pollfd pfd;
    uint8_t buf[256];

    while (running_) {
        int current_fd = -1;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            current_fd = fd_;
        }

        if (current_fd < 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        pfd.fd = current_fd;
        pfd.events = POLLIN;
        pfd.revents = 0;

        // poll() cho toi da 20 ms (tuong ung chu ky 50 Hz cua ESP32). Khi co
        // byte toi no tra ve ngay, khong can them mot sleep_for(co dieu kien)
        // nhu ban select() + sleep_for(5 ms) truoc day -- von tre them 5 ms
        // moi vong ke ca khi da co du lieu.
        int ret = ::poll(&pfd, 1, 20);

        if (ret < 0) {
            if (errno == EINTR) continue;   // bi ngat, thu lai
            // Loi poll (fd hong) = mat lien lac that.
            std::cerr << "[SerialESP32] poll failed: " << strerror(errno) << "\n";
            read_error_count_.fetch_add(1);
            link_down_.store(true);
            break;
        }

        if (ret > 0 && (pfd.revents & POLLIN)) {
            // Vet can toan bo byte dang cho trong kernel buffer, nen telemetry
            // nhan duoc luon la mau moi nhat, khong phai mau cu trong buffer.
            while (true) {
                ssize_t n = ::read(current_fd, buf, sizeof(buf));
                if (n > 0) {
                    for (ssize_t i = 0; i < n; ++i) {
                        process_rx_byte(buf[i]);
                    }
                    continue;
                }
                if (n == 0) break;                                  // het du lieu
                if (errno == EINTR) continue;                       // doc bi ngat
                if (errno == EAGAIN || errno == EWOULDBLOCK) break;  // buffer rong

                // EIO/EBADF: ESP32 bi rut hoac driver USB reset. Gan co de
                // node dong cong va phanh, thay vi giu trang thai "OK" gia.
                std::cerr << "[SerialESP32] read() failed: " << strerror(errno) << "\n";
                read_error_count_.fetch_add(1);
                link_down_.store(true);
                break;
            }
        } else if (ret > 0 && (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))) {
            // Cong chet nhung read() van tra EAGAIN (thuong gap voi USB CDC
            // sau khi rut cap), nen phai kiem tra revents rieng.
            std::cerr << "[SerialESP32] poll error/hangup on fd " << current_fd
                      << ", marking link down\n";
            read_error_count_.fetch_add(1);
            link_down_.store(true);
        }
    }
}

// ============================================================================
// RX STATE MACHINE (FRAME TIMEOUT 10MS ĐỒNG BỘ VỚI FIRMWARE ESP32)
// ============================================================================
void SerialESP32::process_rx_byte(uint8_t b) {
    static auto last_rx_time = std::chrono::steady_clock::now();
    static constexpr int64_t FRAME_TIMEOUT_US = 10000; // 10ms Frame Timeout

    auto now = std::chrono::steady_clock::now();

    // 1. Chống kẹt byte: Nếu gói tin bị đứt đoạn quá 10ms -> tự reset về đầu
    if (rx_state_ != RxState::WAIT_H1) {
        auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(
            now - last_rx_time).count();
        if (elapsed_us > FRAME_TIMEOUT_US) {
            rx_state_ = RxState::WAIT_H1;
            rx_idx_ = 0;
        }
    }
    last_rx_time = now;

    // 2. Máy trạng thái phân tích gói tin
    switch (rx_state_) {
        case RxState::WAIT_H1: // Chờ 0xDC
            if (b == RX_HEADER_1) {
                rx_packet_[0] = b;
                rx_idx_ = 1;
                rx_state_ = RxState::WAIT_H2;
            }
            break;

        case RxState::WAIT_H2: // Chờ 0xBA
            if (b == RX_HEADER_2) {
                rx_packet_[1] = b;
                rx_idx_ = 2;
                rx_state_ = RxState::READ_PAYLOAD;
            } else if (b == RX_HEADER_1) {
                // Rơi nhầm byte nhưng lại trùng H1 -> giữ lại H1
                rx_packet_[0] = b;
                rx_idx_ = 1;
            } else {
                rx_state_ = RxState::WAIT_H1;
                rx_idx_ = 0;
            }
            break;

        case RxState::READ_PAYLOAD: // Đọc tiếp 5 bytes (4 byte float speed + 1 byte checksum)
            if (rx_idx_ < RX_PACKET_LEN) {
                rx_packet_[rx_idx_++] = b;
            }

            if (rx_idx_ >= RX_PACKET_LEN) {
                // Tính Checksum XOR từ byte 2 đến byte 5 (4 bytes float)
                uint8_t calc_csum = calculate_checksum(&rx_packet_[2], 4);

                // Vị trí 6 là checksum từ ESP32
                if (calc_csum == rx_packet_[6]) {
                    ESP32Feedback fb;
                    std::memcpy(&fb.velocity_kmh, &rx_packet_[2], sizeof(float));

                    fb.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
                        now.time_since_epoch()).count();

                    // Lọc dữ liệu hợp lý
                    if (std::isfinite(fb.velocity_kmh) &&
                        fb.velocity_kmh >= 0.0f &&
                        fb.velocity_kmh <= 100.0f) {
                        fb.valid = true;

                        std::lock_guard<std::mutex> lock(feedback_mtx_);
                        latest_feedback_ = fb;
                    }
                }

                // Luôn reset về đầu để đón frame tiếp theo
                rx_state_ = RxState::WAIT_H1;
                rx_idx_ = 0;
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