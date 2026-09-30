"""Gửi webcam laptop sang Mini PC qua TCP (định dạng: len 4 bytes BE + JPEG).

Sửa so với bản cũ:
  * socket.connect() nằm NGOÀI try -> Pi chưa bật/đúng IP thì script chết
    ngay với traceback, không có thông báo nào hữu ích. Nay báo lỗi rõ
    ràng và thoát êm.
  * Không có timeout: connect() có thể treo vô hạn.
  * Không tự kết nối lại khi Pi khởi động lại.
  * Không kiểm tra camera mở được không.
"""

import argparse
import socket
import struct
import sys
import time

import cv2


def parse_args():
    p = argparse.ArgumentParser(description="Stream laptop webcam sang Mini PC")
    p.add_argument("--host", default="192.168.137.168",
                   help="IP Mini PC (mặc định: IP hotspot Windows)")
    p.add_argument("--port", type=int, default=8000)
    p.add_argument("--cam", type=int, default=0, help="index camera")
    p.add_argument("--width", type=int, default=320)
    p.add_argument("--height", type=int, default=240)
    p.add_argument("--quality", type=int, default=60, help="JPEG quality")
    p.add_argument("--timeout", type=float, default=5.0, help="giây chờ connect")
    return p.parse_args()


def connect(host, port, timeout):
    """Kết nối, thử lại mỗi 2 giây. Nhấn Ctrl-C để dừng."""
    delay = 2.0
    while True:
        try:
            s = socket.create_connection((host, port), timeout=timeout)
            s.settimeout(None)
            print(f"Da ket noi den {host}:{port}")
            return s
        except OSError as e:
            print(f"Ket noi that bai ({e}). Thu lai sau {delay:.0f}s... "
                  f"(Ctrl-C de thoat)", file=sys.stderr)
            time.sleep(delay)


def main():
    args = parse_args()

    cap = cv2.VideoCapture(args.cam)
    if not cap.isOpened():
        print(f"Khong mo duoc camera index {args.cam}", file=sys.stderr)
        return 1

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    sock = connect(args.host, args.port, args.timeout)
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), args.quality]
    sent = 0
    last_stat = time.monotonic()

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Camera mat frame, dung lai.", file=sys.stderr)
                break

            ok, encimg = cv2.imencode(".jpg", frame, encode_param)
            if not ok:
                continue
            data = encimg.tobytes()

            try:
                sock.sendall(struct.pack(">L", len(data)) + data)
            except OSError as e:
                # Pi đã tắt / mất mạng -> quay lại vòng chờ kết nối lại.
                print(f"Ngat ket noi ({e}), thu ket noi lai...", file=sys.stderr)
                sock.close()
                sock = connect(args.host, args.port, args.timeout)
                continue

            sent += 1
            now = time.monotonic()
            if now - last_stat >= 1.0:
                print(f"Da gui {sent} anh ({sent / (now - last_stat + 1e-9):.1f} fps)",
                      file=sys.stderr)
                sent = 0
                last_stat = now
    except KeyboardInterrupt:
        print("\nDung theo yeu cau.")
    finally:
        cap.release()
        sock.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
