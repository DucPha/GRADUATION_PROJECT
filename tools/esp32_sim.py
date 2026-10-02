#!/usr/bin/env python3
"""ESP32 giả lập để chạy thử toàn hệ thống KHÔNG CẦN phần cứng.

Tạo một cổng serial ảo, đúng tên đường dẫn mà node fusion_viz_node có thể mở,
và nói chuyện đúng protocol nhị phân của firmware thật.

    PC -> ESP32 (11 byte):  AB CD | dev_hi dev_lo | speed | emg | 00 00 00 00 | XOR(2..9)
    ESP32 -> PC (7 byte):   DC BA | float32 velocity | XOR(2..5)

Script này CỐ TÌNH dùng đúng số byte, đúng thứ tự, đúng phép XOR như .ino.
Đổi lệch chỗ nào thì test ở đây bắt được, không phải lúc chạy xe thật.

Cách dùng:
    # cổng ảo tại /tmp/ttyESP32sim
    python3 tools/esp32_sim.py

    # chạy node với cổng ảo
    ros2 launch autonomous_vehicle bringup.launch.py serial_port:=/tmp/ttyESP32sim

Tắt: Ctrl-C. Xoá cổng ảo: rm -f /tmp/ttyESP32sim
"""

import argparse
import errno
import math
import os
import pty
import select
import signal
import struct
import sys
import termios
import time

# --------------------------------------------------------------------------
# Hằng số phải khớp firmware Autonomous_Vehicle.ino
# --------------------------------------------------------------------------
HDR_RX1, HDR_RX2 = 0xAB, 0xCD      # byte đồng bộ của frame MiniPC -> ESP32
HDR_TX1, HDR_TX2 = 0xDC, 0xBA      # byte đồng bộ của frame ESP32 -> MiniPC
RX_LEN, TX_LEN = 11, 7

IDX_LANE_H, IDX_LANE_L = 2, 3
IDX_SPEED, IDX_EMG, IDX_CSUM = 4, 5, 10

MAX_SPEED_KMH = 15.0        # firmware: giới hạn tốc độ mục tiêu
MAX_VALID_SPEED_KMH = 20.0  # firmware: ngưỡng chặn nhiễu tốc độ
WDOG_TIMEOUT_MS = 500       # firmware: mất gói lệnh quá lâu -> dừng xe
TELEM_DT_MS = 20            # firmware: 50 Hz telemetry
BAUD = 230400

# Bảng tra ESC firmware (pwm -> km/h). Dùng để tốc độ phản hồi bám lệnh.
ESC_LUT = [
    (90, 0.00), (94, 0.00), (95, 1.55), (100, 3.58), (102, 6.56),
    (105, 8.34), (110, 9.50), (115, 11.00), (120, 12.37), (125, 12.79),
    (130, 13.41), (135, 13.59), (140, 13.71), (145, 14.01), (150, 14.16),
    (155, 14.19), (160, 14.28), (165, 14.31), (170, 14.34), (175, 14.46),
    (180, 14.75),
]


def xor_bytes(data):
    c = 0
    for b in data:
        c ^= b
    return c


def esc_pwm_for_speed(kmh):
    """Ngược bảng ESC: trả PWM tương ứng tốc độ mong muốn."""
    if kmh <= ESC_LUT[2][1]:
        return 95
    for (p0, s0), (p1, s1) in zip(ESC_LUT, ESC_LUT[1:]):
        if s0 <= kmh <= s1:
            if s1 == s0:
                return int(p1)
            return int(round(p0 + (p1 - p0) * (kmh - s0) / (s1 - s0)))
    return int(ESC_LUT[-1][0])


class RxParser:
    """Máy trạng thái nhận frame 11 byte, y hệt firmware."""

    def __init__(self):
        self.state = 0          # 0=WAIT_H1 1=WAIT_H2 2=PAYLOAD
        self.buf = bytearray(RX_LEN)
        self.idx = 0
        self.good = 0
        self.bad = 0

    def feed(self, b):
        if self.state == 0:
            if b == HDR_RX1:
                self.buf[0] = b
                self.idx = 1
                self.state = 1
        elif self.state == 1:
            if b == HDR_RX2:
                self.buf[1] = b
                self.idx = 2
                self.state = 2
            elif b == HDR_RX1:
                self.buf[0] = b
                self.idx = 1
            else:
                self.state = 0
                self.idx = 0
        else:
            if self.idx < RX_LEN:
                self.buf[self.idx] = b
                self.idx += 1
            if self.idx >= RX_LEN:
                if xor_bytes(self.buf[2:10]) == self.buf[IDX_CSUM]:
                    self.good += 1
                    self._dispatch()
                else:
                    self.bad += 1
                self.state = 0
                self.idx = 0

    def _dispatch(self):
        dev = struct.unpack(">h", bytes(self.buf[IDX_LANE_H:IDX_LANE_L + 1]))[0]
        self.last_dev = dev
        self.last_speed = min(self.buf[IDX_SPEED] * 0.1, MAX_SPEED_KMH)
        self.last_emg = self.buf[IDX_EMG] != 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--link", default="/tmp/ttyESP32sim",
                    help="đường dẫn symlink tới cổng serial ảo")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    # Log phải ra ngay khi chạy nền và ghi ra file: mặc định print bị
    # block-buffer nên nhìn như treo máy.
    sys.stdout.reconfigure(line_buffering=True)

    master_fd, slave_fd = pty.openpty()

    # Đặt cổng ảo ở chế độ raw: nếu không, byte 0x1D/0x0A trong protocol sẽ bị
    # kernel dịch thành 0x0A/0x0A và checksum sai mọi frame.
    attrs = termios.tcgetattr(slave_fd)
    attrs[0] = 0                      # iflag: không xử lý ký tự
    attrs[1] = 0                      # oflag: không xử lý ký tự
    attrs[3] = 0                      # lflag: raw
    attrs[2] = (attrs[2] & ~termios.CSIZE) | termios.CS8
    attrs[2] &= ~(termios.PARENB | termios.PARODD | termios.CSTOPB)
    attrs[2] &= ~termios.CRTSCTS
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(slave_fd, termios.TCSANOW, attrs)

    if os.path.lexists(args.link):
        os.unlink(args.link)
    os.symlink(os.ttyname(slave_fd), args.link)

    running = {"go": True}

    def bye(*_):
        running["go"] = False

    signal.signal(signal.SIGINT, bye)
    signal.signal(signal.SIGTERM, bye)

    print(f"[sim] cổng serial ảo: {args.link} -> {os.ttyname(slave_fd)}")
    print(f"[sim] baud {BAUD}, nhận frame {RX_LEN} byte, gửi telemetry {TX_LEN} byte @ "
          f"{1000 // TELEM_DT_MS} Hz")
    print("[sim] Ctrl-C để dừng\n")

    rx = RxParser()
    rx.last_dev, rx.last_speed, rx.last_emg = 0, 0.0, False

    emg = False
    dev = 0
    target = 0.0
    cur_spd = 0.0
    last_packet = time.monotonic()
    last_tx = 0.0
    last_print = 0.0

    while running["go"]:
        now = time.monotonic()

        # --- nhận lệnh (non-blocking) ---
        try:
            r, _, _ = select.select([master_fd], [], [], 0.005)
        except (OSError, select.error) as e:
            if getattr(e, "errno", None) == errno.EINTR:
                continue
            break

        if r:
            try:
                data = os.read(master_fd, 256)
            except OSError as e:
                if e.errno == errno.EIO:      # đối phương đóng cổng
                    time.sleep(0.01)
                    continue
                raise
            for b in data:
                rx.feed(b)
            if rx.good != getattr(rx, "_seen", 0):
                rx._seen = rx.good
                last_packet = now
                dev = rx.last_dev
                emg = rx.last_emg
                target = rx.last_speed

        # --- watchdog giống firmware ---
        stale = (now - last_packet) * 1000.0 > WDOG_TIMEOUT_MS
        if stale:
            target = 0.0
            emg = True

        # --- mô phỏng động cơ: bám mục tiêu với gia tốc/phanh ---
        brake = emg and cur_spd > 0.3
        if brake:
            cur_spd = max(0.0, cur_spd - 30.0 * 0.01)   # phanh mạnh
        elif target > cur_spd:
            cur_spd = min(target, cur_spd + 4.0 * 0.01)
        elif target < cur_spd:
            cur_spd = max(target, cur_spd - 6.0 * 0.01)

        cur_spd = max(0.0, min(cur_spd, MAX_VALID_SPEED_KMH))

        # --- telemetry 50 Hz ---
        if (now - last_tx) * 1000.0 >= TELEM_DT_MS:
            last_tx = now
            frame = bytearray(TX_LEN)
            frame[0], frame[1] = HDR_TX1, HDR_TX2
            struct.pack_into("<f", frame, 2, cur_spd)
            frame[6] = xor_bytes(frame[2:6])
            try:
                os.write(master_fd, bytes(frame))
            except OSError:
                break

        # --- log 2 Hz để người dùng thấy xe đang phản ứi ---
        if not args.quiet and (now - last_print) >= 0.5:
            last_print = now
            pwm = esc_pwm_for_speed(target)
            flag = "STOP" if emg else "RUN "
            print(f"[sim] {flag} dev={dev:+4d}px  target={target:4.1f}km/h  "
                  f"cur={cur_spd:4.1f}km/h  pwm={pwm:3d}  "
                  f"frames={rx.good} bad={rx.bad}"
                  f"{'  STALE' if stale else ''}")

    try:
        os.close(master_fd)
        os.close(slave_fd)
    except OSError:
        pass
    if os.path.lexists(args.link):
        os.unlink(args.link)
    print(f"\n[sim] dừng. Nhận {rx.good} frame hợp lệ, {rx.bad} frame CRC sai.")


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        pass