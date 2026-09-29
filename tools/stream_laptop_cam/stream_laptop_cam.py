import cv2
import socket
import struct

# Điền IP mạng Hotspot của Raspberry Pi (thường là 192.168.4.1 hoặc 10.42.0.1)
PI_HOTSPOT_IP = '192.168.137.168'
STREAM_PORT = 8000

# Mở webcam laptop
cap = cv2.VideoCapture(0)
# Hạ độ phân giải xuống để Pi xử lý mượt và mạng truyền không bị trễ
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print(f"Dang ket noi den Pi tai {PI_HOTSPOT_IP}:{STREAM_PORT}...")
client_socket.connect((PI_HOTSPOT_IP, STREAM_PORT))
connection = client_socket.makefile('wb')
print("Da ket noi! Dang truyen hinh anh sang Pi...")

encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 60]  # Nén JPEG 60% để truyền tức thì

try:
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # Nén frame sang định dạng JPEG
        result, encimg = cv2.imencode('.jpg', frame, encode_param)
        data = encimg.tobytes()
        size = len(data)

        # Gửi độ dài gói tin (4 bytes) rồi gửi dữ liệu ảnh
        client_socket.sendall(struct.pack(">L", size) + data)

except Exception as e:
    print(f"Ngat ket noi: {e}")
finally:
    cap.release()
    client_socket.close()