#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AUTOCAR MONITOR HMI
===================
PySide6 cho giao diện realtime (ảnh vẽ bằng QPainter, không qua Matplotlib).
Matplotlib CHỈ dùng cho panel LiDAR, giữ đúng thiết kế của code RPLIDAR gốc.
OpenCV dùng để giải mã/resize ảnh nhận từ ROS 2.

Chạy (dữ liệu thật từ ROS 2): python3 gui.py
Nhịp vẽ 60 FPS               : python3 gui.py --fps 60
"""

import argparse
import sys
import threading
import time
from dataclasses import dataclass

import numpy as np

from PySide6.QtCore import Qt, QTimer, QRectF, QPointF
from PySide6.QtGui import QColor, QFont, QImage, QPainter, QPen
from PySide6.QtWidgets import (
    QApplication, QFrame, QGridLayout, QHBoxLayout, QLabel,
    QMainWindow, QSizePolicy, QVBoxLayout, QWidget,
)

# Matplotlib chỉ dành cho widget LiDAR.
import matplotlib.colors as mcolors
from matplotlib.figure import Figure
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas


# ============================================================================
# CONFIG
# ============================================================================

APP_TITLE = "AUTOCAR MONITOR"

DMAX = 12000
IMIN = 0
IMAX = 50

CAM_W, CAM_H = 640, 400
ROI_Y0, ROI_Y1 = 120, 400
ROI_H = ROI_Y1 - ROI_Y0

LANE_THR_PX = 10
LINK_TIMEOUT_S = 1.0
STATUS_PERIOD_S = 0.25

TOPIC_SCAN = "/scan"
TOPIC_RAW = "/autocar/dbg/cam_raw/compressed"
TOPIC_ROI = "/autocar/dbg/lane_roi/compressed"
TOPIC_BIN = "/autocar/dbg/lane_bin/compressed"
TOPIC_STATUS = "/lane/status"

NODES = ("CAMERA", "LIDAR", "ESP32-S3")


# ============================================================================
# THEME
# ============================================================================

C_BG = "#F3F4F6"
C_WHITE = "#FFFFFF"
C_HEAD = "#111827"
C_HEAD_2 = "#1F2937"

C_TEXT = "#1F2937"
C_TEXT_2 = "#4B5563"
C_MUTED = "#737D8B"
C_LINE = "#D9DEE5"
C_LINE_2 = "#E8EBEF"
C_PANEL_INNER = "#F8FAFC"

C_BLUE = "#2563EB"
C_BLUE_SOFT = "#EAF2FF"
C_GREEN = "#1F9D55"
C_GREEN_SOFT = "#EAF7EF"
C_ORANGE = "#D97706"
C_ORANGE_SOFT = "#FFF4DE"
C_RED = "#DC3B3B"
C_RED_SOFT = "#FDECEC"

FONT_UI = "Segoe UI"
FONT_MONO = "Consolas"

BADGE_STYLES = {
    "blue": (C_BLUE_SOFT, C_BLUE, "#D5E5FF"),
    "green": (C_GREEN_SOFT, C_GREEN, "#D2ECD9"),
    "orange": (C_ORANGE_SOFT, C_ORANGE, "#F4DEB2"),
    "red": (C_RED_SOFT, C_RED, "#F2C9C9"),
}
KIND_COLOR = {"blue": C_BLUE, "green": C_GREEN, "orange": C_ORANGE, "red": C_RED}

# Tổng thời gian vẽ (ms) các viewport ảnh, để hiển thị trên tiêu đề.
PERF = {"paint": 0.0}


# ============================================================================
# HELPERS
# ============================================================================

def perf_now():
    return time.perf_counter()


def format_uptime(seconds):
    s = max(0, int(seconds))
    return f"{s // 3600:02d}:{(s // 60) % 60:02d}:{s % 60:02d}"


def parse_num(text):
    """Đọc số từ giá trị key=value của /lane/status.

    Node C++ in cả đơn vị sau số: age=12ms, proc=2.1ms, front=142cm...
    '2.1ms' -> 2.1, '142cm' -> 142.0, 'ok' -> None.
    """
    s = str(text)
    i = 1 if s[:1] in ("+", "-") else 0
    dot = False
    while i < len(s) and (s[i].isdigit() or (s[i] == "." and not dot)):
        if s[i] == ".":
            dot = True
        i += 1
    body = s[:i]
    if not any(c.isdigit() for c in body):
        return None
    try:
        return float(body)
    except ValueError:
        return None


# ============================================================================
# DATA HUB: chỉ giữ dữ liệu MỚI NHẤT của mỗi nguồn (không hàng đợi -> không tràn)
# ============================================================================

@dataclass
class NodeStat:
    msgs: int = 0
    rx: int = 0
    tx: int = 0
    err: int = 0
    t_last: float = 0.0


class Hub:
    def __init__(self):
        self.lock = threading.Lock()
        self.scan = None
        self.scan_seq = 0
        self.cam = None
        self.cam_seq = 0
        self.stats = {name: NodeStat() for name in NODES}
        self.info = {
            "proc_ms": None, "tx": "--", "rx": "--",
            "speed": None, "steer": None,
            "mode": "AUTONOMOUS", "state": "FOLLOW LANE",
        }

    def _touch(self, node, rx=0, tx=0, err=None):
        s = self.stats[node]
        s.msgs += 1
        s.rx += int(rx)
        s.tx += int(tx)
        s.t_last = time.time()
        if err is not None:
            s.err = int(err)

    # Mảng đưa vào Hub phải là mảng MỚI mỗi lần (không sửa tại chỗ sau khi put).
    def put_cam(self, frame, roi, binary, lane_center, nbytes, proc_ms=None):
        with self.lock:
            self.cam = (frame, roi, binary, lane_center)
            self.cam_seq += 1
            self.info["proc_ms"] = proc_ms
            self._touch("CAMERA", rx=nbytes)

    def put_scan(self, scan, nbytes):
        with self.lock:
            self.scan = scan
            self.scan_seq += 1
            self._touch("LIDAR", rx=nbytes)

    def put_esp(self, tx_text, rx_text, tx_bytes, rx_bytes, err=0,
                speed=None, steer=None, mode=None, state=None):
        with self.lock:
            self.info["tx"] = tx_text
            self.info["rx"] = rx_text
            self.info["speed"] = speed
            self.info["steer"] = steer
            if mode is not None:
                self.info["mode"] = mode
            if state is not None:
                self.info["state"] = state
            self._touch("ESP32-S3", rx=rx_bytes, tx=tx_bytes, err=err)

    def set_info(self, **kw):
        with self.lock:
            self.info.update(kw)

    def snapshot(self):
        with self.lock:
            stats = {n: (s.msgs, s.rx, s.tx, s.err, s.t_last)
                     for n, s in self.stats.items()}
            return (self.scan, self.scan_seq, self.cam, self.cam_seq,
                    stats, dict(self.info))


# ============================================================================
# ROS 2 FEEDER
# ============================================================================

class RosFeeder(threading.Thread):
    def __init__(self, hub):
        super().__init__(daemon=True)
        self.hub = hub
        self.stop_event = threading.Event()

    def stop(self):
        self.stop_event.set()

    def run(self):
        try:
            import cv2
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
            from sensor_msgs.msg import CompressedImage, LaserScan
            from std_msgs.msg import String
        except ImportError as exc:
            print(f"[ROS 2] import error: {exc}")
            return

        hub = self.hub
        # depth=1 + best effort: ROS không xếp hàng, luôn lấy gói mới nhất
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        last = {"roi": None, "binary": None, "lane_center": None}
        proc = {"ms": None}

        def decode(msg, flag):
            return cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), flag)

        def fit(img, w, h, interp):
            # Chỉ resize khi kích thước khác
            if img.shape[1] == w and img.shape[0] == h:
                return img
            return cv2.resize(img, (w, h), interpolation=interp)

        def on_scan(msg):
            n = len(msg.ranges)
            if n == 0:
                return
            angles = np.degrees(msg.angle_min + np.arange(n, dtype=np.float32)
                                * msg.angle_increment) % 360.0
            dist = np.nan_to_num(np.asarray(msg.ranges, dtype=np.float32),
                                 nan=DMAX / 1000, posinf=DMAX / 1000, neginf=0) * 1000
            dist = np.clip(dist, 150, DMAX)
            if len(msg.intensities) == n:
                inten = np.asarray(msg.intensities, dtype=np.float32)
            else:
                inten = np.zeros(n, dtype=np.float32)
            hub.put_scan(np.column_stack((inten, angles, dist)), nbytes=n * 8)

        def on_roi(msg):
            img = decode(msg, cv2.IMREAD_COLOR)
            if img is not None:
                last["roi"] = fit(img, CAM_W, ROI_H, cv2.INTER_LINEAR)

        def on_bin(msg):
            img = decode(msg, cv2.IMREAD_GRAYSCALE)
            if img is None:
                return
            img = (fit(img, CAM_W, ROI_H, cv2.INTER_NEAREST) > 127).astype(np.uint8) * 255
            cols = np.flatnonzero(img[-6])
            last["binary"] = img
            last["lane_center"] = ((float(cols[0]) + float(cols[-1])) / 2
                                   if cols.size >= 2 else None)

        def on_raw(msg):
            img = decode(msg, cv2.IMREAD_COLOR)
            if img is None:
                return
            img = fit(img, CAM_W, CAM_H, cv2.INTER_LINEAR)
            roi = last["roi"] if last["roi"] is not None else img[ROI_Y0:ROI_Y1].copy()
            binary = (last["binary"] if last["binary"] is not None
                      else np.zeros((ROI_H, CAM_W), dtype=np.uint8))
            hub.put_cam(img, roi, binary, last["lane_center"],
                        nbytes=len(msg.data), proc_ms=proc["ms"])

        def on_status(msg):
            """ /lane/status (2 Hz, key=value, node C++ in kèm đơn vị):
            two_lanes=1 dev=-12 emg=0 age=12ms proc=2.1ms fps=29.4
            lidar=ok front=142cm serial=open w=50cm devm=-4cm
            spd=4.0 kmh=0.00 fbage=12 alert=CLEAR """
            d = {}
            for tok in str(msg.data).split():
                if "=" in tok:
                    key, val = tok.split("=", 1)
                    d[key] = val
            if not d:
                return

            proc["ms"] = parse_num(d.get("proc"))

            dev = parse_num(d.get("dev")) or 0.0
            emg = int(parse_num(d.get("emg")) or 0)
            two_lanes = int(parse_num(d.get("two_lanes")) or 0)

            # Góc servo mà firmware tính từ dev (calcSteerPID: deadzone 10 px,
            # max 50 px -> servo 60..120°, tâm 90°). ESP chỉ báo vận tốc nên
            # góc servo tính lại theo đúng công thức, không đọc được từ serial.
            off = min(max(abs(dev) - 10.0, 0.0), 40.0) / 40.0 * 30.0
            steer = 90.0 + off if dev < 0 else 90.0 - off

            # Vận tốc THỰC TẾ từ ESP32 (-1 = chưa có telemetry)
            kmh = parse_num(d.get("kmh"))
            speed = kmh if kmh is not None and kmh >= 0 else None

            if emg:
                state = "EMERGENCY STOP"
            elif two_lanes:
                state = "FOLLOW LANE"
            else:
                state = "LANE LOST"

            fbage = parse_num(d.get("fbage"))
            serial_ok = d.get("serial") == "open"
            err = 0 if serial_ok and fbage is not None and 0 <= fbage <= 200 else 1
            spd_cmd = parse_num(d.get("spd")) or 0.0

            rx_text = f"VEL {speed:.2f} km/h" if speed is not None else "VEL --"
            if fbage is not None and fbage >= 0:
                rx_text += f"  age={fbage:.0f} ms"

            hub.put_esp(
                f"CMD dev={dev:+.0f}px spd={spd_cmd:.1f} emg={emg}",
                rx_text,
                tx_bytes=11, rx_bytes=7, err=err,
                speed=speed, steer=steer, state=state)

        rclpy.init()
        node = Node("autocar_monitor_py")
        node.create_subscription(LaserScan, TOPIC_SCAN, on_scan, qos)
        node.create_subscription(CompressedImage, TOPIC_ROI, on_roi, qos)
        node.create_subscription(CompressedImage, TOPIC_BIN, on_bin, qos)
        node.create_subscription(CompressedImage, TOPIC_RAW, on_raw, qos)
        node.create_subscription(String, TOPIC_STATUS, on_status, qos)
        while rclpy.ok() and not self.stop_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.02)
        node.destroy_node()
        rclpy.shutdown()


# ============================================================================
# SMALL UI BUILDING BLOCKS
# ============================================================================

class PanelHeader(QFrame):
    def __init__(self, title, subtitle=""):
        super().__init__()
        self.setObjectName("panelHeader")
        self.setFixedHeight(34)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(11, 0, 10, 0)
        layout.setSpacing(8)

        self.title = QLabel(title.upper())
        self.title.setObjectName("panelTitle")
        self.subtitle = QLabel(subtitle.upper())
        self.subtitle.setObjectName("panelSubtitle")
        self.badge = QLabel("")
        self.badge.setObjectName("panelBadge")
        self._badge_text = ""
        self._badge_kind = None

        layout.addWidget(self.title)
        if subtitle:
            layout.addWidget(self.subtitle)
        layout.addStretch()
        layout.addWidget(self.badge)

    def set_badge(self, text, kind="blue"):
        # setStyleSheet chỉ gọi khi đổi màu; mỗi khung ảnh chỉ đổi chữ
        if text != self._badge_text:
            self._badge_text = text
            self.badge.setText(text)
        if not text or kind == self._badge_kind:
            return
        self._badge_kind = kind
        bg, fg, border = BADGE_STYLES.get(kind, BADGE_STYLES["blue"])
        self.badge.setStyleSheet(
            f"#panelBadge {{ background: {bg}; color: {fg};"
            f" border: 1px solid {border}; border-radius: 8px;"
            f" padding: 2px 7px; font-size: 7pt; font-weight: 700; }}")


class ImageViewport(QWidget):
    def __init__(self, raw=False, processed=False):
        super().__init__()
        self.raw = raw
        self.processed = processed
        self.image = QImage()
        self._buf = None                  # giữ mảng numpy sống cùng QImage
        self.lane_center = None
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

        self._bg = QColor("#F8FAFC")
        self._pen_border = QPen(QColor(C_LINE), 1)
        self._pen_roi = QPen(QColor(C_BLUE), 2)
        self._pen_center = QPen(QColor(C_BLUE), 1, Qt.PenStyle.DashLine)
        self._pen_lane = QPen(QColor(C_RED), 2)
        self._font_wait = QFont(FONT_UI, 10)
        self._font_chip = QFont(FONT_UI, 8)
        self._font_chip.setBold(True)

    def set_frame(self, arr, lane_center=None):
        """arr: uint8, HxWx3 (BGR như OpenCV) hoặc HxW (xám).
        Dùng trực tiếp bộ nhớ numpy, không đổi kênh màu, không sao chép."""
        arr = np.ascontiguousarray(arr)
        h, w = arr.shape[:2]
        fmt = (QImage.Format.Format_Grayscale8 if arr.ndim == 2
               else QImage.Format.Format_BGR888)
        self.image = QImage(arr.data, w, h, arr.strides[0], fmt)
        self._buf = arr
        self.lane_center = lane_center
        self.update()

    def paintEvent(self, event):
        t0 = perf_now()
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform, False)
        painter.fillRect(self.rect(), self._bg)

        if self.image.isNull():
            painter.setPen(QColor(C_MUTED))
            painter.setFont(self._font_wait)
            painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, "WAITING FOR IMAGE")
            painter.end()
            return

        iw, ih = self.image.width(), self.image.height()
        scale = min(self.width() / iw, self.height() / ih)
        dw, dh = max(1, int(iw * scale)), max(1, int(ih * scale))
        x = (self.width() - dw) // 2
        y = (self.height() - dh) // 2
        target = QRectF(x, y, dw, dh)

        painter.drawImage(target, self.image)
        painter.setPen(self._pen_border)
        painter.drawRect(target.adjusted(0, 0, -1, -1))

        # Khung ROI trên ảnh raw
        if self.raw and ih == CAM_H:
            top = y + ROI_Y0 * scale
            bottom = y + ROI_Y1 * scale
            painter.setPen(self._pen_roi)
            painter.drawRect(QRectF(x + 1, top, max(1, dw - 2), bottom - top))
            self._chip(painter, "ROI", x + 10, top + 6, C_BLUE)

        # Tâm làn và độ lệch trên ảnh kết quả
        if self.processed:
            cx = x + dw / 2
            painter.setPen(self._pen_center)
            painter.drawLine(QPointF(cx, y), QPointF(cx, y + dh))

            if self.lane_center is not None:
                lx = x + max(0, min(CAM_W - 1, float(self.lane_center))) * scale
                painter.setPen(self._pen_lane)
                painter.setBrush(QColor(C_RED))
                painter.drawEllipse(QPointF(lx, y + dh - 13), 5, 5)
                dev = self.lane_center - CAM_W / 2
                self._chip(painter, f"DEV {dev:+.0f}px", x + 9, y + dh - 31, C_HEAD)

        painter.end()
        PERF["paint"] += (perf_now() - t0) * 1000

    def _chip(self, painter, text, x, y, bg):
        painter.setFont(self._font_chip)
        m = painter.fontMetrics()
        rect = QRectF(x, y, m.horizontalAdvance(text) + 14, m.height() + 7)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QColor(bg))
        painter.drawRoundedRect(rect, 4, 4)
        painter.setPen(QColor("#FFFFFF"))
        painter.drawText(rect, Qt.AlignmentFlag.AlignCenter, text)


class ImagePanel(QFrame):
    def __init__(self, title, raw=False, processed=False):
        super().__init__()
        self.setObjectName("imagePanel")
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)
        self.header = PanelHeader(title)
        root.addWidget(self.header)
        self.view = ImageViewport(raw=raw, processed=processed)
        root.addWidget(self.view, 1)


# ============================================================================
# STATUS SIDE PANEL
# ============================================================================

class NodeRow(QFrame):
    def __init__(self, name):
        super().__init__()
        self.setObjectName("nodeRow")
        layout = QHBoxLayout(self)
        layout.setContentsMargins(8, 6, 8, 6)
        layout.setSpacing(8)

        self.dot = QLabel("●")
        self.dot.setObjectName("nodeDot")
        self.dot.setFixedWidth(12)
        self.name = QLabel(name)
        self.name.setObjectName("nodeName")
        self.rate = QLabel("--")
        self.rate.setObjectName("nodeRate")
        self.state = QLabel("WAITING")
        self.state.setObjectName("nodeState")
        self._connected = None

        layout.addWidget(self.dot)
        layout.addWidget(self.name)
        layout.addStretch()
        layout.addWidget(self.rate)
        layout.addWidget(self.state)

    def set_status(self, connected, rate, state):
        if connected != self._connected:       # chỉ đổi style khi trạng thái đổi
            self._connected = connected
            color = C_GREEN if connected else C_RED
            self.dot.setStyleSheet(f"#nodeDot {{ color: {color}; }}")
            self.state.setStyleSheet(
                f"#nodeState {{ color: {color}; font-weight: 700; }}")
        if self.rate.text() != rate:
            self.rate.setText(rate)
        if self.state.text() != state:
            self.state.setText(state)


class SystemPanel(QFrame):
    def __init__(self):
        super().__init__()
        self.setObjectName("systemPanel")
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)
        root.addWidget(PanelHeader("SYSTEM STATUS", "COMMUNICATION / TELEMETRY"))

        body = QWidget()
        layout = QVBoxLayout(body)
        layout.setContentsMargins(10, 9, 10, 10)
        layout.setSpacing(7)

        self.node_rows = {}
        for name in NODES:
            row = NodeRow(name)
            self.node_rows[name] = row
            layout.addWidget(row)

        separator = QFrame()
        separator.setFrameShape(QFrame.Shape.HLine)
        separator.setObjectName("separator")
        layout.addWidget(separator)

        section = QLabel("VEHICLE")
        section.setObjectName("miniSection")
        layout.addWidget(section)

        grid = QGridLayout()
        grid.setHorizontalSpacing(16)
        grid.setVerticalSpacing(5)

        self.speed = QLabel("--")
        self.steer = QLabel("--")
        self.proc = QLabel("--")

        for title, value, r, c in (("SPEED", self.speed, 0, 0),
                                   ("STEER", self.steer, 0, 1),
                                   ("PROC", self.proc, 1, 0)):
            box = QVBoxLayout()
            box.setContentsMargins(0, 0, 0, 0)
            box.setSpacing(0)
            lab = QLabel(title)
            lab.setObjectName("metricLabel")
            value.setObjectName("metricBigValue")
            box.addWidget(lab)
            box.addWidget(value)
            host = QWidget()
            host.setLayout(box)
            grid.addWidget(host, r, c)

        layout.addLayout(grid)
        root.addWidget(body, 1)


# ============================================================================
# LIDAR: GIỮ ĐÚNG THIẾT KẾ CODE RPLIDAR GỐC
# ============================================================================

class OldMatplotlibLidar(QFrame):
    """Phần Matplotlib giống hệt code gốc: figsize 4x4, polar, c=[0], s=15,
    ListedColormap + BoundaryNorm, rmax 12 m, nhãn 2m..12m, rlabel 22.5,
    grid, subplots_adjust(top .92, bottom .08, left .08, right .92) và công
    thức cỡ chữ max(6, fig_width * 1.3).

    Khác code gốc ở cách cập nhật: dùng blit (chỉ vẽ lại các điểm) thay cho
    vẽ lại cả hình, nên mỗi scan chỉ tốn vài ms."""

    def __init__(self):
        super().__init__()
        self.setObjectName("lidarPanel")
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)
        self.header = PanelHeader("LiDAR MAP", "ORIGINAL POLAR VIEW")
        root.addWidget(self.header)

        # ---- giống create/run() của code gốc ----
        self.figure = Figure(figsize=(4, 4))
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(111, projection="polar")

        cmap = mcolors.ListedColormap(["red", "orange", "yellow", "green"])
        norm = mcolors.BoundaryNorm([0, 2000, 4000, 8000, DMAX + 1], cmap.N)

        # Thêm c=[0] để "kích hoạt" engine tính màu theo mảng dữ liệu
        self.scatter = self.ax.scatter([0], [0], c=[0], s=15, cmap=cmap, norm=norm, lw=0)

        self.ax.set_rmax(DMAX)
        self.ax.set_yticks([2000, 4000, 6000, 8000, 10000, 12000])
        self.ax.set_yticklabels(["2m", "4m", "6m", "8m", "10m", "12m"], alpha=0.7)
        self.ax.set_rlabel_position(22.5)
        self.ax.grid(True)

        self.figure.subplots_adjust(top=0.92, bottom=0.08, left=0.08, right=0.92)

        self.canvas.mpl_connect("resize_event", self._on_resize)
        self._on_resize(None)
        # -----------------------------------------

        # Blit: nền (lưới, nhãn) lưu 1 lần, mỗi scan chỉ vẽ lại scatter
        self.scatter.set_animated(True)
        self._bg = None
        self.canvas.mpl_connect("draw_event", self._on_draw)

        root.addWidget(self.canvas, 1)

        footer = QFrame()
        footer.setObjectName("lidarFooter")
        fl = QHBoxLayout(footer)
        fl.setContentsMargins(10, 4, 10, 5)
        fl.setSpacing(15)

        self.nearest_label = QLabel("NEAREST  --")
        self.points_label = QLabel("POINTS  --")
        self.status_label = QLabel("STATUS  --")
        for lab in (self.nearest_label, self.points_label, self.status_label):
            lab.setObjectName("lidarMetric")
            fl.addWidget(lab)
        fl.addStretch()

        legend = QLabel("0–2m  RED   2–4m  ORANGE   4–8m  YELLOW   >8m  GREEN")
        legend.setObjectName("lidarLegend")
        fl.addWidget(legend)
        root.addWidget(footer)

        self._status_kind = None

    def _on_resize(self, event):
        new_size = max(6, self.figure.get_figwidth() * 1.3)
        self.ax.tick_params(axis="both", labelsize=new_size)
        self._bg = None                    # nền cũ không còn đúng kích thước

    def _on_draw(self, event):
        self._bg = self.canvas.copy_from_bbox(self.figure.bbox)
        self.figure.draw_artist(self.scatter)

    def set_scan(self, scan):
        if scan is None or len(scan) == 0:
            return

        # Cùng quy ước dữ liệu như update_line() gốc: theta = radian, r = mm
        self.scatter.set_offsets(np.column_stack((np.radians(scan[:, 1]), scan[:, 2])))
        self.scatter.set_array(scan[:, 2])

        if self._bg is not None:
            self.canvas.restore_region(self._bg)
            self.figure.draw_artist(self.scatter)
            self.canvas.blit(self.figure.bbox)
        else:
            self.canvas.draw_idle()

        dist = scan[:, 2]
        i = int(np.argmin(dist))
        nearest, angle = float(dist[i]), float(scan[i, 1])

        self.nearest_label.setText(f"NEAREST  {nearest / 1000:.2f} m  @  {angle:.0f}°")
        self.points_label.setText(f"POINTS  {len(scan):,}")

        if nearest < 2000:
            state, kind = "DANGER", "red"
        elif nearest < 4000:
            state, kind = "CAUTION", "orange"
        else:
            state, kind = "CLEAR", "green"
        self.status_label.setText(f"STATUS  {state}")

        if kind != self._status_kind:
            self._status_kind = kind
            self.status_label.setStyleSheet(
                f'#lidarMetric {{ color: {KIND_COLOR[kind]}; font-family: "{FONT_MONO}";'
                f" font-size: 7pt; font-weight: 700; }}")


# ============================================================================
# MAIN WINDOW
# ============================================================================

class AutoCarMonitor(QMainWindow):
    def __init__(self, hub, mode, fps=60, width=1440, height=900):
        super().__init__()
        self.hub = hub
        self.mode = mode
        self.target_fps = max(20, min(120, int(fps)))

        self.setWindowTitle(APP_TITLE)
        self.resize(width, height)
        self.setMinimumSize(1150, 720)

        self.start_time = perf_now()
        self.last_cam_seq = -1
        self.last_scan_seq = -1

        self.frames = 0
        self.fps_value = 0.0
        self.last_fps_time = perf_now()
        self.upd_ms = 0.0
        self.paint_ms = 0.0

        self.prev_stats = None
        self.prev_stats_t = None
        self._ready_kind = None

        self._apply_style()
        self._build_ui()

        self.timer = QTimer(self)
        self.timer.setTimerType(Qt.TimerType.PreciseTimer)
        self.timer.timeout.connect(self.update_gui)
        self.timer.start(max(5, round(1000 / self.target_fps)))

    # ------------------------------------------------------------------ style
    def _apply_style(self):
        self.setStyleSheet(f"""
            QMainWindow {{ background: {C_BG}; }}
            QWidget {{ font-family: "{FONT_UI}"; color: {C_TEXT}; }}

            #header {{ background: {C_HEAD}; border: none; }}
            #appTitle {{ color: white; font-size: 15pt; font-weight: 800; }}
            #appSubtitle {{ color: #A8B0BC; font-size: 8pt; }}
            #headerInfo {{ color: #DCE2EA; font-family: "{FONT_MONO}"; font-size: 7.5pt; }}

            #modeStrip {{ background: {C_WHITE}; border-bottom: 1px solid {C_LINE}; }}
            #modeBig {{ color: {C_HEAD}; font-family: "{FONT_MONO}"; font-size: 9pt; font-weight: 800; }}
            #stateBig {{ color: {C_BLUE}; font-size: 10pt; font-weight: 800; }}
            #stateMeta {{ color: {C_MUTED}; font-family: "{FONT_MONO}"; font-size: 7pt; }}
            #readyBadge {{ border-radius: 9px; padding: 4px 10px; font-size: 7.5pt; font-weight: 800; }}

            #imagePanel, #systemPanel, #lidarPanel {{
                background: {C_WHITE}; border: 1px solid {C_LINE}; border-radius: 6px; }}
            #panelHeader {{ background: #FBFCFD; border-bottom: 1px solid {C_LINE_2}; }}
            #panelTitle {{ color: {C_TEXT_2}; font-size: 8pt; font-weight: 800; }}
            #panelSubtitle {{ color: {C_MUTED}; font-size: 6.8pt; font-weight: 600; }}
            #panelBadge {{ font-size: 7pt; font-weight: 700; }}

            #nodeRow {{ background: {C_PANEL_INNER}; border: 1px solid {C_LINE_2}; border-radius: 4px; }}
            #nodeName {{ color: {C_TEXT}; font-size: 8.5pt; font-weight: 800; }}
            #nodeRate {{ color: {C_TEXT_2}; font-family: "{FONT_MONO}"; font-size: 7.5pt; font-weight: 700; }}
            #nodeState {{ font-size: 7pt; min-width: 68px; }}
            #nodeDot {{ color: {C_MUTED}; font-size: 9pt; }}

            #miniSection {{ color: {C_MUTED}; font-size: 7pt; font-weight: 800; padding-top: 1px; }}
            #metricLabel {{ color: {C_MUTED}; font-family: "{FONT_MONO}"; font-size: 6.5pt; font-weight: 700; }}
            #metricBigValue {{ color: {C_TEXT}; font-family: "{FONT_MONO}"; font-size: 10pt; font-weight: 800; }}

            #separator {{ color: {C_LINE}; background: {C_LINE}; max-height: 1px; }}
            #vseparator {{ color: {C_LINE}; background: {C_LINE}; max-width: 1px; }}

            #lidarPanel {{ background: white; }}
            #lidarFooter {{ background: #FBFCFD; border-top: 1px solid {C_LINE_2}; }}
            #lidarMetric {{ color: {C_MUTED}; font-family: "{FONT_MONO}"; font-size: 7pt; font-weight: 700; }}
            #lidarLegend {{ color: {C_MUTED}; font-family: "{FONT_MONO}"; font-size: 6.5pt; }}

            #bottomBar {{ background: {C_HEAD_2}; border: none; }}
            #bottomText {{ color: #AEB7C4; font-family: "{FONT_MONO}"; font-size: 6.8pt; }}
        """)

    # --------------------------------------------------------------------- UI
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)
        root.addWidget(self._build_header())
        root.addWidget(self._build_state_strip())
        root.addWidget(self._build_main_area(), 1)
        root.addWidget(self._build_bottom_bar())

    def _build_header(self):
        header = QFrame()
        header.setObjectName("header")
        header.setFixedHeight(55)
        layout = QHBoxLayout(header)
        layout.setContentsMargins(15, 0, 15, 0)
        layout.setSpacing(10)

        title = QLabel("AUTOCAR MONITOR")
        title.setObjectName("appTitle")
        subtitle = QLabel("AUTONOMOUS VEHICLE ENGINEERING DASHBOARD")
        subtitle.setObjectName("appSubtitle")
        self.header_info = QLabel("")
        self.header_info.setObjectName("headerInfo")
        self.header_info.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)

        layout.addWidget(title)
        layout.addWidget(subtitle)
        layout.addStretch()
        layout.addWidget(self.header_info)
        return header

    def _build_state_strip(self):
        frame = QFrame()
        frame.setObjectName("modeStrip")
        frame.setFixedHeight(43)
        layout = QHBoxLayout(frame)
        layout.setContentsMargins(15, 0, 15, 0)
        layout.setSpacing(13)

        self.mode_big = QLabel("AUTONOMOUS")
        self.mode_big.setObjectName("modeBig")

        divider = QFrame()
        divider.setFrameShape(QFrame.Shape.VLine)
        divider.setObjectName("vseparator")

        self.state_big = QLabel("FOLLOW LANE")
        self.state_big.setObjectName("stateBig")
        self.state_meta = QLabel("PERCEPTION → DECISION → CONTROL")
        self.state_meta.setObjectName("stateMeta")
        self.ready_badge = QLabel("● SYSTEM READY")
        self.ready_badge.setObjectName("readyBadge")
        self.set_ready_badge("ready")

        layout.addWidget(self.mode_big)
        layout.addWidget(divider)
        layout.addWidget(self.state_big)
        layout.addWidget(self.state_meta)
        layout.addStretch()
        layout.addWidget(self.ready_badge)
        return frame

    def _build_main_area(self):
        outer = QWidget()
        grid = QGridLayout(outer)
        grid.setContentsMargins(13, 11, 13, 9)
        grid.setHorizontalSpacing(11)
        grid.setVerticalSpacing(11)
        grid.setColumnStretch(0, 62)
        grid.setColumnStretch(1, 38)

        # Trái: raw camera + (ROI | kết quả)
        left = QWidget()
        ll = QVBoxLayout(left)
        ll.setContentsMargins(0, 0, 0, 0)
        ll.setSpacing(10)

        self.raw_panel = ImagePanel("RAW CAMERA", raw=True)

        bottom = QWidget()
        bp = QGridLayout(bottom)
        bp.setContentsMargins(0, 0, 0, 0)
        bp.setHorizontalSpacing(10)
        self.roi_panel = ImagePanel("ROI")
        self.result_panel = ImagePanel("PROCESSED RESULT", processed=True)
        bp.addWidget(self.roi_panel, 0, 0)
        bp.addWidget(self.result_panel, 0, 1)
        bp.setColumnStretch(0, 1)
        bp.setColumnStretch(1, 1)

        ll.addWidget(self.raw_panel, 4)
        ll.addWidget(bottom, 2)

        # Phải: trạng thái hệ thống + LiDAR
        right = QWidget()
        rl = QVBoxLayout(right)
        rl.setContentsMargins(0, 0, 0, 0)
        rl.setSpacing(10)
        self.system_panel = SystemPanel()
        self.lidar_panel = OldMatplotlibLidar()
        rl.addWidget(self.system_panel, 1)
        rl.addWidget(self.lidar_panel, 2)

        grid.addWidget(left, 0, 0)
        grid.addWidget(right, 0, 1)
        return outer

    def _build_bottom_bar(self):
        footer = QFrame()
        footer.setObjectName("bottomBar")
        footer.setFixedHeight(27)
        layout = QHBoxLayout(footer)
        layout.setContentsMargins(12, 0, 12, 0)

        self.bottom_left = QLabel("STATE  --")
        self.bottom_left.setObjectName("bottomText")
        self.bottom_right = QLabel("")
        self.bottom_right.setObjectName("bottomText")
        self.bottom_right.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)

        layout.addWidget(self.bottom_left)
        layout.addStretch()
        layout.addWidget(self.bottom_right)
        return footer

    def set_ready_badge(self, kind):
        if kind == self._ready_kind:
            return
        self._ready_kind = kind
        text, fg, bg, border = {
            "ready": ("● SYSTEM READY", C_GREEN, C_GREEN_SOFT, "#CDE9D7"),
            "partial": ("● PARTIAL LINK", C_ORANGE, C_ORANGE_SOFT, "#F0D9A6"),
        }.get(kind, ("● OFFLINE", C_RED, C_RED_SOFT, "#F0CACA"))
        self.ready_badge.setText(text)
        self.ready_badge.setStyleSheet(
            f"#readyBadge {{ color: {fg}; background: {bg}; border: 1px solid {border}; }}")

    # --------------------------------------------------------------- realtime
    def update_gui(self):
        t0 = perf_now()
        scan, scan_seq, cam, cam_seq, stats, info = self.hub.snapshot()

        # Ảnh chỉ cập nhật khi có khung MỚI
        if cam is not None and cam_seq != self.last_cam_seq:
            frame, roi, binary, lane_center = cam
            self.raw_panel.view.set_frame(frame, lane_center)
            self.roi_panel.view.set_frame(roi)
            self.result_panel.view.set_frame(binary, lane_center)

            proc = info.get("proc_ms")
            if proc is not None:
                self.raw_panel.header.set_badge(f"{proc:.1f} ms", "blue")

            self.last_cam_seq = cam_seq
            self.frames += 1                       # FPS = số khung camera đã hiển thị
            self.paint_ms += 0.1 * (PERF["paint"] - self.paint_ms)
            PERF["paint"] = 0.0

        # LiDAR: chỉ khi có scan mới
        if scan is not None and scan_seq != self.last_scan_seq:
            self.lidar_panel.set_scan(scan)
            self.last_scan_seq = scan_seq

        self.upd_ms += 0.1 * ((perf_now() - t0) * 1000 - self.upd_ms)

        t = perf_now()
        if t - self.last_fps_time >= STATUS_PERIOD_S:
            elapsed = t - self.last_fps_time
            self.fps_value = self.frames / elapsed if elapsed > 0 else 0.0
            self.frames = 0
            self.last_fps_time = t
            self._update_status(stats, info, cam, scan)

    def _update_status(self, stats, info, cam, scan):
        now = time.time()
        have_prev = self.prev_stats is not None and self.prev_stats_t is not None
        dt = max(1e-6, now - self.prev_stats_t) if have_prev else 0

        online = 0
        for name in NODES:
            msgs, rx, tx, err, t_last = stats[name]
            connected = t_last > 0 and (now - t_last) < LINK_TIMEOUT_S
            online += connected
            rate = (msgs - self.prev_stats[name][0]) / dt if have_prev and dt > 0 else 0
            state = ("CONNECTED" if connected
                     else "WAITING" if t_last == 0 else "DISCONNECTED")
            self.system_panel.node_rows[name].set_status(
                connected, f"{rate:.1f} Hz" if connected else "--", state)

        speed, steer = info.get("speed"), info.get("steer")
        proc = info.get("proc_ms")
        sp = self.system_panel
        sp.speed.setText(f"{float(speed):.2f} km/h" if speed is not None else "--")
        sp.steer.setText(f"{float(steer):.1f}°" if steer is not None else "--")
        sp.proc.setText(f"{float(proc):.1f} ms" if proc is not None else "--")

        mode = str(info.get("mode", "AUTONOMOUS"))
        state = str(info.get("state", "FOLLOW LANE"))
        self.mode_big.setText(mode.upper())
        self.state_big.setText(state.upper())

        n = len(NODES)
        self.set_ready_badge("ready" if online == n else "partial" if online > 0 else "offline")

        up = perf_now() - self.start_time
        self.header_info.setText(
            f"{self.mode}   |   GUI {self.fps_value:4.1f} FPS"
            f"   |   UPD {self.upd_ms:.1f} ms   PAINT {self.paint_ms:.1f} ms"
            f"   |   UP {format_uptime(up)}   |   {time.strftime('%H:%M:%S')}")

        lane_dev = (float(cam[3]) - CAM_W / 2) if cam is not None and cam[3] is not None else None
        if lane_dev is None:
            lane_state = "NO LANE"
        elif lane_dev > LANE_THR_PX:
            lane_state = "RIGHT"
        elif lane_dev < -LANE_THR_PX:
            lane_state = "LEFT"
        else:
            lane_state = "STRAIGHT"

        nearest = float(np.min(scan[:, 2])) if scan is not None and len(scan) else None
        nearest_text = f"{nearest / 1000:.2f} m" if nearest is not None else "--"
        dev_text = f"{lane_dev:+.0f}px" if lane_dev is not None else "--"
        proc_text = f"PROC {proc:.1f} ms" if proc is not None else "PROC --"

        self.bottom_left.setText(
            f"STATE  {state.upper()}   |   LANE {lane_state}   |   DEV {dev_text}")
        self.bottom_right.setText(
            f"{proc_text}   |   NEAREST {nearest_text}   |   {online}/{n} LINKS")

        self.prev_stats = stats
        self.prev_stats_t = now

    def closeEvent(self, event):
        self.timer.stop()
        event.accept()


# ============================================================================
# MAIN
# ============================================================================

def main():
    parser = argparse.ArgumentParser(description=APP_TITLE)
    parser.add_argument("--fps", type=int, default=60, help="Nhịp vẽ GUI")
    parser.add_argument("--width", type=int, default=1440)
    parser.add_argument("--height", type=int, default=900)
    args = parser.parse_args()

    app = QApplication(sys.argv)
    app.setApplicationName(APP_TITLE)

    hub = Hub()
    feeder, mode = RosFeeder(hub), "ROS 2 LIVE"
    feeder.start()

    window = AutoCarMonitor(hub, mode, fps=args.fps, width=args.width, height=args.height)
    window.show()

    code = app.exec()
    feeder.stop()
    feeder.join(timeout=0.5)
    sys.exit(code)


if __name__ == "__main__":
    main()