#include "esp32s3_node.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include <dirent.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

namespace {

// XOR tat ca byte trong [data, data + len)
uint8_t xor_checksum(const uint8_t* data, size_t len) {
    uint8_t c = 0;
    for (size_t i = 0; i < len; ++i) {
        c ^= data[i];
    }
    return c;
}

bool path_exists(const std::string& path) {
    struct stat st {};
    return !path.empty() && ::stat(path.c_str(), &st) == 0;
}

// Cung 1 thiet bi? (so sanh duong dan that sau khi giai symlink by-id)
bool same_device(const std::string& a, const std::string& b) {
    if (a.empty() || b.empty()) {
        return false;
    }
    char ra[PATH_MAX];
    char rb[PATH_MAX];
    const bool oka = ::realpath(a.c_str(), ra) != nullptr;
    const bool okb = ::realpath(b.c_str(), rb) != nullptr;
    return (oka && okb) ? std::strcmp(ra, rb) == 0 : a == b;
}

std::vector<std::string> list_devices(const std::string& prefix) {
    std::vector<std::string> names;
    DIR* handle = ::opendir("/dev");
    if (!handle) {
        return names;
    }
    while (const dirent* entry = ::readdir(handle)) {
        const std::string name = entry->d_name;
        if (name.rfind(prefix, 0) == 0) {
            names.push_back("/dev/" + name);
        }
    }
    ::closedir(handle);
    // Sap xep de thu tu do on dinh giua cac lan khoi dong
    std::sort(names.begin(), names.end());
    return names;
}

// Mo cong o che do raw 8N1 230400, non-blocking. Tra ve fd hoac -1.
int open_raw_port(const std::string& path, bool verbose) {
    const int fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        if (verbose) {
            std::cerr << "[SerialESP32] Cannot open " << path << ": "
                      << std::strerror(errno) << "\n";
        }
        return -1;
    }

    struct termios tty {};
    if (tcgetattr(fd, &tty) != 0) {
        if (verbose) {
            std::cerr << "[SerialESP32] tcgetattr " << path << ": "
                      << std::strerror(errno) << "\n";
        }
        ::close(fd);
        return -1;
    }

    // cfmakeraw: tat moi xu ly ky tu cua Linux (echo, CR/LF, XON/XOFF...)
    cfmakeraw(&tty);
    cfsetospeed(&tty, B230400);
    cfsetispeed(&tty, B230400);
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);  // 8N1, khong flow control
    tty.c_cflag |= CS8 | CREAD | CLOCAL;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        if (verbose) {
            std::cerr << "[SerialESP32] tcsetattr " << path << ": "
                      << std::strerror(errno) << "\n";
        }
        ::close(fd);
        return -1;
    }

    // Ha DTR + RTS CUNG LUC: mach auto-reset cua board ESP32 (2 transistor
    // noi DTR/RTS vao EN/IO0) khong bi kich -> ESP32 khong khoi dong lai.
    int status = 0;
    if (ioctl(fd, TIOCMGET, &status) == 0) {
        status &= ~(TIOCM_DTR | TIOCM_RTS);
        ioctl(fd, TIOCMSET, &status);
    }

    tcflush(fd, TCIOFLUSH);
    return fd;
}

}   // namespace

// ============================================================================
// DO CONG ESP32
// ============================================================================

bool probe_esp32(const std::string& path, int timeout_ms) {
    const int fd = open_raw_port(path, false);
    if (fd < 0) {
        return false;
    }

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);
    uint8_t pkt[SerialESP32::RX_PACKET_LEN_V2];
    size_t idx = 0;
    size_t len = SerialESP32::RX_PACKET_LEN;
    bool found = false;

    while (!found && std::chrono::steady_clock::now() < deadline) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        struct timeval tv {0, 20000};
        if (select(fd + 1, &rfds, nullptr, nullptr, &tv) <= 0) {
            continue;
        }
        uint8_t buf[256];
        const ssize_t n = ::read(fd, buf, sizeof(buf));
        for (ssize_t i = 0; i < n && !found; ++i) {
            const uint8_t b = buf[i];
            if (idx == 0) {
                idx = (b == SerialESP32::RX_HEADER_1) ? 1 : 0;
                pkt[0] = b;
            } else if (idx == 1) {
                if (b == SerialESP32::RX_HEADER_2 || b == SerialESP32::RX_HEADER_2_V2) {
                    pkt[1] = b;
                    idx = 2;
                    len = (b == SerialESP32::RX_HEADER_2_V2) ? SerialESP32::RX_PACKET_LEN_V2
                                                             : SerialESP32::RX_PACKET_LEN;
                } else {
                    idx = (b == SerialESP32::RX_HEADER_1) ? 1 : 0;
                }
            } else {
                pkt[idx++] = b;
                if (idx == len) {
                    found = xor_checksum(&pkt[2], len - 3) == pkt[len - 1];
                    idx = 0;
                }
            }
        }
    }

    ::close(fd);
    return found;
}

std::string autodetect_port(const std::string& exclude) {
    for (const char* prefix : {"ttyUSB", "ttyACM"}) {
        for (const auto& path : list_devices(prefix)) {
            if (same_device(path, exclude)) {
                continue;
            }
            if (probe_esp32(path)) {
                return path;
            }
        }
    }
    return "";
}

// ============================================================================
// CONSTRUCTOR / DESTRUCTOR
// ============================================================================

SerialESP32::SerialESP32(const std::string& port, const std::string& exclude)
    : port_(port), requested_port_(port), exclude_(exclude) {}

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
    if (fd_ >= 0) {
        return true;
    }

    const bool verbose = fail_streak_ == 0;
    ++fail_streak_;   // dat lai 0 khi mo thanh cong

    // Cong cau hinh khong ton tai (rut cap, doi thu tu ttyUSB) -> tu do
    port_ = requested_port_;
    if (!path_exists(port_)) {
        if (!port_.empty() && verbose) {
            std::cerr << "[SerialESP32] " << port_
                      << " does not exist, auto-detecting ESP32...\n";
        }
        port_ = autodetect_port(exclude_);
        if (port_.empty()) {
            if (!verbose) {
                return false;
            }
            std::cerr << "[SerialESP32] No ESP32 found on /dev/ttyUSB* or "
                         "/dev/ttyACM*. Check the USB cable or pass "
                         "serial_port:=<path>.\n";
            return false;
        }
        std::cout << "[SerialESP32] Auto-detected ESP32 on " << port_ << "\n";
    }

    fd_ = open_raw_port(port_, verbose);
    if (fd_ < 0) {
        return false;
    }
    fail_streak_ = 0;

    rx_state_ = RxState::WAIT_H1;
    rx_idx_ = 0;
    broken_ = false;
    running_ = true;
    feedback_thread_ = std::thread(&SerialESP32::feedback_loop, this);

    std::cout << "[SerialESP32] Opened " << port_ << " @ " << BAUDRATE
              << " baud (UART)\n";
    return true;
}

void SerialESP32::close() {
    // Tat co truoc de luong RX tu thoat, roi moi dong fd
    running_ = false;

    if (feedback_thread_.joinable()) {
        feedback_thread_.join();
    }

    std::lock_guard<std::mutex> lock(mtx_);
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
        std::cout << "[SerialESP32] Closed " << port_ << "\n";
    }
}

// ============================================================================
// GUI LENH XUONG ESP32 (TX)
// ============================================================================

bool SerialESP32::send_command(const SerialCommand& cmd) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (fd_ < 0) {
        return false;
    }

    uint8_t packet[TX_PACKET_LEN];
    packet[0] = TX_HEADER_1;
    packet[1] = TX_HEADER_2;
    packet[2] = static_cast<uint8_t>((cmd.dev_final_px >> 8) & 0xFF);  // MSB
    packet[3] = static_cast<uint8_t>(cmd.dev_final_px & 0xFF);         // LSB
    packet[4] = cmd.speed_control;                                     // km/h x10
    packet[5] = cmd.emergency_stop ? 1 : 0;
    packet[6] = packet[7] = packet[8] = packet[9] = 0x00;              // du phong
    packet[10] = xor_checksum(&packet[2], 8);

    const ssize_t written = ::write(fd_, packet, TX_PACKET_LEN);
    if (written < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        // EIO / ENXIO / ENODEV: thiet bi da bien mat
        broken_ = true;
    }
    return written == static_cast<ssize_t>(TX_PACKET_LEN);
}

// ============================================================================
// TELEMETRY
// ============================================================================

ESP32Feedback SerialESP32::get_latest_feedback() const {
    std::lock_guard<std::mutex> lock(feedback_mtx_);
    return latest_feedback_;
}

unsigned long SerialESP32::feedback_age_ms() const {
    std::lock_guard<std::mutex> lock(feedback_mtx_);
    if (!latest_feedback_.valid) {
        return static_cast<unsigned long>(-1);
    }
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - last_feedback_time_).count();
    return ms < 0 ? 0ul : static_cast<unsigned long>(ms);
}

// ============================================================================
// LUONG DOC NGAM (RX)
// ============================================================================

void SerialESP32::feedback_loop() {
    int fd = -1;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        fd = fd_;
    }
    // fd_ chi bi dong SAU khi luong nay da join, nen dung ban sao fd an toan

    while (running_) {
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(fd, &read_fds);
        struct timeval timeout {0, 10000};   // 10 ms, khong an CPU

        const int ret = select(fd + 1, &read_fds, nullptr, nullptr, &timeout);
        if (ret < 0) {
            if (errno != EINTR) {
                broken_ = true;
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            continue;
        }
        if (ret == 0) {
            continue;
        }

        uint8_t buf[128];
        const ssize_t n = ::read(fd, buf, sizeof(buf));
        if (n > 0) {
            for (ssize_t i = 0; i < n; ++i) {
                process_rx_byte(buf[i]);
            }
        } else if (n == 0 || (errno != EAGAIN && errno != EINTR)) {
            // select bao co du lieu ma read tra 0 = cong da bi rut
            broken_ = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }
}

// ============================================================================
// MAY TRANG THAI GOI RX
// ============================================================================

void SerialESP32::process_rx_byte(uint8_t b) {
    switch (rx_state_) {
        case RxState::WAIT_H1:
            if (b == RX_HEADER_1) {
                rx_packet_[0] = b;
                rx_idx_ = 1;
                rx_state_ = RxState::WAIT_H2;
            }
            break;

        case RxState::WAIT_H2:
            if (b == RX_HEADER_2 || b == RX_HEADER_2_V2) {
                rx_packet_[1] = b;
                rx_idx_ = 2;
                rx_len_ = (b == RX_HEADER_2_V2) ? RX_PACKET_LEN_V2 : RX_PACKET_LEN;
                rx_state_ = RxState::READ_PAYLOAD;
            } else if (b != RX_HEADER_1) {
                // byte 0xDC lap lai thi van giu WAIT_H2
                rx_state_ = RxState::WAIT_H1;
            }
            break;

        case RxState::READ_PAYLOAD:
            rx_packet_[rx_idx_++] = b;
            if (rx_idx_ == rx_len_) {
                const size_t csum_at = rx_len_ - 1;   // byte cuoi = XOR byte 2..csum_at-1
                if (xor_checksum(&rx_packet_[2], csum_at - 2) == rx_packet_[csum_at]) {
                    ESP32Feedback fb;
                    std::memcpy(&fb.velocity_kmh, &rx_packet_[2], sizeof(float));
                    fb.valid = std::isfinite(fb.velocity_kmh);
                    if (rx_len_ == RX_PACKET_LEN_V2) {
                        fb.has_v2 = true;
                        fb.esc_deg = rx_packet_[6];
                        fb.steer_deg = rx_packet_[7];
                        fb.fw_emg = (rx_packet_[8] & 0x01) != 0;
                        fb.fw_watchdog = (rx_packet_[8] & 0x02) != 0;
                        fb.fw_got_cmd = (rx_packet_[8] & 0x08) != 0;
                    }
                    // Checksum dung nhung float NaN/inf -> bo, khong ghi de
                    if (fb.valid) {
                        std::lock_guard<std::mutex> lock(feedback_mtx_);
                        latest_feedback_ = fb;
                        last_feedback_time_ = std::chrono::steady_clock::now();
                    }
                }
                rx_state_ = RxState::WAIT_H1;
            }
            break;
    }
}
