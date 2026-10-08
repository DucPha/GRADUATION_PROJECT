#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AUTOCAR MONITOR - dashboard realtime cho xe tu hanh
===================================================

Moi so lieu xu ly anh deu do node C++ (fusion_node) tinh. GUI KHONG tu xu ly
anh: chi giai ma va hien thi, nen nhung gi thay tren man hinh dung la nhung gi
bo dieu khien dang dung.

  /autocar/dbg/lane_vis/compressed  anh camera + overlay detector (C++ ve)
  /autocar/dbg/lane_roi/compressed  vung ROI cua anh camera
  /autocar/dbg/lane_bin/compressed  mask nhi phan sau morphology
  /lane/status                      trang thai key=value, 10 Hz
  /scan                             LaserScan tu driver RPLIDAR
  /autocar/run   (GUI phat)         std_msgs/Bool, 10 Hz: true = cho xe chay

Phim tat:  SPACE = chay / dung    ESC = dung ngay
Xe chi chay khi GUI dang mo va gui heartbeat; dong GUI -> xe dung.

Chay: python3 src/gui/gui.py [--fps 60] [--width 1280 --height 760]
Can: PySide6 (hoac PySide2), numpy, opencv (python3-opencv), rclpy.
"""

import argparse
import math
import sys
import threading
import time

import numpy as np

# ---- Qt: uu tien PySide6, khong co thi dung PySide2 (apt) ----
try:
    from PySide6.QtCore import Qt, QTimer, QRectF, QPointF
    from PySide6.QtGui import QColor, QFont, QImage, QPainter, QPen, QKeySequence, QShortcut
    from PySide6.QtWidgets import (
        QApplication, QFrame, QGridLayout, QHBoxLayout, QLabel, QMainWindow,
        QPushButton, QSizePolicy, QVBoxLayout, QWidget,
    )
    QT_BINDING = "PySide6"
except ImportError:  # pragma: no cover - chi dung khi may chua co PySide6
    from PySide2.QtCore import Qt, QTimer, QRectF, QPointF
    from PySide2.QtGui import QColor, QFont, QImage, QPainter, QPen, QKeySequence
    from PySide2.QtWidgets import (
        QApplication, QFrame, QGridLayout, QHBoxLayout, QLabel, QMainWindow,
        QPushButton, QShortcut, QSizePolicy, QVBoxLayout, QWidget,
    )
    QT_BINDING = "PySide2"



# ============================================================================
# CAU HINH
# ============================================================================

APP_TITLE = "AUTOCAR MONITOR"

TOPIC_VIS = "/autocar/dbg/lane_vis/compressed"
TOPIC_ROI = "/autocar/dbg/lane_roi/compressed"
TOPIC_BIN = "/autocar/dbg/lane_bin/compressed"
TOPIC_STATUS = "/lane/status"
TOPIC_SCAN = "/scan"
TOPIC_RUN = "/autocar/run"

RUN_HEARTBEAT_HZ = 10.0      # GUI gui /autocar/run deu dan (fusion: timeout 600 ms)
LINK_TIMEOUT_S = 1.0         # khong co du lieu qua lau -> mat ket noi
FUSION_LOST_STOP_S = 1.5     # mat /lane/status qua lau -> tu bo lenh chay
NO_SIGNAL_S = 1.0            # anh dung qua lau -> hien "NO SIGNAL"
STATUS_UI_PERIOD_S = 0.1     # cap nhat chu/so 10 Hz

# LiDAR: giu thiet ke ban do goc (polar, mm, 4 dai mau do/cam/vang/xanh).
# Xe 1/10 chi quan tam vat trong vai met -> mac dinh 3 m. O thang 12 m, tay
# dua lai gan (20-50 cm) chi cach tam vai pixel, bi bieu tuong xe che mat.
DMAX = 12000
LIDAR_RANGES_M = (0.5, 1, 2, 3, 4, 6, 8, 12)   # lan chuot de doi tam nhin
LIDAR_DEFAULT_RANGE_M = 3
# Dai mau theo DUNG nguong canh bao cua LidarModule C++ (mm):
#   do < 40 cm (DANGER), cam < 60 cm (WARNING), vang < 1.5 m, xanh >= 1.5 m
LIDAR_COLOR_BOUNDS_MM = (0, 400, 600, 1500, DMAX + 1)
# RPLIDAR A1 khong do duoc vat gan hon 15 cm (driver khong tra ve tia nao)
LIDAR_BLIND_MM = 150
# Mau diem: van bo do/cam/vang/xanh nhung DAM hon de noi ro tren nen trang
# (vang "yellow" cua matplotlib gan nhu bien mat tren nen trang).
LIDAR_COLORS = ("#DC2626", "#EA580C", "#CA8A04", "#15803D")
DEFAULT_LIDAR_OFFSET_DEG = 90.0           # = lidar_mount_offset_deg cua launch

LINKS = ("FUSION", "CAMERA", "LIDAR", "ESP32")


# ============================================================================
# GIAO DIEN: MAU + FONT
# ============================================================================

C_BG = "#EEF1F5"
C_CARD = "#FFFFFF"
C_HEAD = "#0F172A"
C_HEAD_2 = "#1E293B"
C_TEXT = "#0F172A"
C_TEXT_2 = "#475569"
C_MUTED = "#64748B"
C_LINE = "#DCE2EA"
C_LINE_2 = "#E9EDF2"
C_SOFT = "#F8FAFC"
C_VIEW_BG = "#0B1220"

C_BLUE = "#2563EB"
C_GREEN = "#16A34A"
C_ORANGE = "#D97706"
C_RED = "#DC2626"

KIND_COLOR = {"blue": C_BLUE, "green": C_GREEN, "orange": C_ORANGE,
              "red": C_RED, "muted": C_MUTED}
KIND_SOFT = {"blue": "#EAF1FF", "green": "#E8F7EE", "orange": "#FFF3DC",
             "red": "#FDECEC", "muted": "#EEF1F5"}

FONT_UI = '"Noto Sans", "DejaVu Sans", "Segoe UI", sans-serif'
FONT_MONO = '"DejaVu Sans Mono", "Liberation Mono", "Consolas", monospace'
FONT_UI_FAMILY = "Noto Sans"


# ============================================================================
# TIEN ICH
# ============================================================================

def now():
    return time.perf_counter()


def parse_num(text):
    """'12ms' -> 12.0, '-4.5cm' -> -4.5, 'ok' -> None, None -> None."""
    if text is None:
        return None
    s = str(text)
    i = 1 if s[:1] in ("+", "-") else 0
    dot = False
    while i < len(s) and (s[i].isdigit() or (s[i] == "." and not dot)):
        dot = dot or s[i] == "."
        i += 1
    body = s[:i]
    if not any(c.isdigit() for c in body):
        return None
    try:
        return float(body)
    except ValueError:
        return None


def parse_status(text):
    """'a=1 b=2ms ...' -> {'a': '1', 'b': '2ms'}"""
    d = {}
    for tok in str(text).split():
        if "=" in tok:
            k, v = tok.split("=", 1)
            d[k] = v
    return d


def fmt_uptime(seconds):
    s = max(0, int(seconds))
    return f"{s // 3600:02d}:{(s // 60) % 60:02d}:{s % 60:02d}"


class RateMeter:
    """Dem tan so nhan (Hz) tren cua so truot ~1 s."""

    def __init__(self):
        self.count = 0
        self.t0 = now()
        self.hz = 0.0
        self.t_last = 0.0

    def tick(self):
        self.count += 1
        self.t_last = now()

    def update(self):
        t = now()
        dt = t - self.t0
        if dt >= 1.0:
            self.hz = self.count / dt
            self.count = 0
            self.t0 = t
        return self.hz

    def alive(self):
        return self.t_last > 0 and now() - self.t_last < LINK_TIMEOUT_S


# ============================================================================
# DATA HUB: chi giu du lieu MOI NHAT cua moi nguon (khong hang doi, khong tre)
# ============================================================================

class Hub:
    def __init__(self, lidar_offset_deg):
        self.lock = threading.Lock()
        self.images = {"vis": None, "roi": None, "bin": None}
        self.image_seq = {"vis": 0, "roi": 0, "bin": 0}
        self.image_t = {"vis": 0.0, "roi": 0.0, "bin": 0.0}
        self.scan = None            # (theta_rad, dist_mm) khung XE
        self.scan_seq = 0
        self.status = {}
        self.status_seq = 0
        self.status_t = 0.0
        self.rates = {name: RateMeter() for name in ("FUSION", "CAMERA", "LIDAR")}
        self.lidar_offset_deg = lidar_offset_deg
        self.want_run = False       # lenh nguoi dung (SPACE)
        self.run_dirty = False      # can gui ngay, khong doi nhip heartbeat

    def put_image(self, key, img):
        with self.lock:
            self.images[key] = img
            self.image_seq[key] += 1
            self.image_t[key] = now()
            if key == "vis":
                self.rates["CAMERA"].tick()

    def put_scan(self, theta_rad, dist_mm):
        with self.lock:
            self.scan = (theta_rad, dist_mm)
            self.scan_seq += 1
            self.rates["LIDAR"].tick()

    def put_status(self, d):
        with self.lock:
            self.status = d
            self.status_seq += 1
            self.status_t = now()
            self.rates["FUSION"].tick()
            lofs = parse_num(d.get("lofs"))
            if lofs is not None:
                self.lidar_offset_deg = lofs

    def set_run(self, value):
        with self.lock:
            if self.want_run != value:
                self.want_run = value
                self.run_dirty = True

    def take_run(self):
        """(want_run, can_gui_ngay)"""
        with self.lock:
            dirty, self.run_dirty = self.run_dirty, False
            return self.want_run, dirty


# ============================================================================
# LUONG ROS 2
# ============================================================================

class RosWorker(threading.Thread):
    def __init__(self, hub):
        super().__init__(daemon=True)
        self.hub = hub
        self.stop_event = threading.Event()
        self.error = None

    def stop(self):
        self.stop_event.set()

    def run(self):
        try:
            import cv2
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
            from sensor_msgs.msg import CompressedImage, LaserScan
            from std_msgs.msg import Bool, String
        except ImportError as exc:
            self.error = f"ROS 2 import error: {exc} (da source /opt/ros/jazzy/setup.bash?)"
            print("[GUI]", self.error)
            return

        hub = self.hub
        # depth 1 + best effort: luon lay goi MOI NHAT, khong xep hang -> khong tre
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)

        def decode(msg, flag):
            return cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), flag)

        def on_vis(msg):
            img = decode(msg, cv2.IMREAD_COLOR)
            if img is not None:
                hub.put_image("vis", img)

        def on_roi(msg):
            img = decode(msg, cv2.IMREAD_COLOR)
            if img is not None:
                hub.put_image("roi", img)

        def on_bin(msg):
            img = decode(msg, cv2.IMREAD_GRAYSCALE)
            if img is not None:
                hub.put_image("bin", img)

        def on_scan(msg):
            # QUY LUAT DU LIEU LiDAR (giong het LidarModule C++):
            #  1. Tia i co goc ROS a_i = angle_min + i * angle_increment (rad),
            #     nguoc chieu kim dong ho, 0 = truc +X cua LiDAR.
            #  2. Chi giu tia co vat that: so huu han, > 0, trong
            #     [range_min, range_max] (inf = khong phan hoi, KHONG phai vat).
            #  3. Doi sang KHUNG XE: goc_xe = a_i + lidar_mount_offset_deg
            #     -> 0 = TRUOC, 90 = TRAI, 180 = SAU, 270 = PHAI (REP-103).
            #  Ban do ve 0 do o phia TREN, tang nguoc chieu kim dong ho.
            r = np.asarray(msg.ranges, dtype=np.float32)
            if r.size == 0:
                return
            ok = np.isfinite(r) & (r > 0.0)
            if msg.range_min > 0.0:
                ok &= r >= msg.range_min
            if msg.range_max > 0.0:
                ok &= r <= msg.range_max
            idx = np.flatnonzero(ok)
            if idx.size == 0:
                hub.put_scan(np.empty(0, np.float32), np.empty(0, np.float32))
                return
            ang = msg.angle_min + idx.astype(np.float64) * msg.angle_increment
            ang += math.radians(hub.lidar_offset_deg)
            theta = np.mod(ang, 2.0 * math.pi).astype(np.float32)
            hub.put_scan(theta, r[idx] * 1000.0)

        def on_status(msg):
            d = parse_status(msg.data)
            if d:
                hub.put_status(d)

        try:
            # Ctrl+C do Qt (main) xu ly; rclpy khong cai signal handler rieng,
            # neu khong luong nay se chet giua chung khi bam Ctrl+C
            from rclpy.signals import SignalHandlerOptions
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
            node = Node("autocar_monitor")
            node.create_subscription(CompressedImage, TOPIC_VIS, on_vis, qos)
            node.create_subscription(CompressedImage, TOPIC_ROI, on_roi, qos)
            node.create_subscription(CompressedImage, TOPIC_BIN, on_bin, qos)
            node.create_subscription(LaserScan, TOPIC_SCAN, on_scan, qos)
            node.create_subscription(String, TOPIC_STATUS, on_status, 10)
            run_pub = node.create_publisher(Bool, TOPIC_RUN, 10)
        except Exception as exc:  # noqa: BLE001
            self.error = f"ROS 2 init error: {exc}"
            print("[GUI]", self.error)
            return

        period = 1.0 / RUN_HEARTBEAT_HZ
        t_next = 0.0
        while rclpy.ok() and not self.stop_event.is_set():
            try:
                rclpy.spin_once(node, timeout_sec=0.005)
            except Exception as exc:  # noqa: BLE001 - context bi tat tu ngoai
                self.error = f"ROS 2 stopped: {exc}"
                break

            # An toan: mat lien lac voi fusion_node -> tu bo lenh chay, de khi
            # node song lai xe KHONG tu chay ma phai bam SPACE lan nua
            if hub.want_run and hub.status_t > 0 and now() - hub.status_t > FUSION_LOST_STOP_S:
                hub.set_run(False)

            want, dirty = hub.take_run()
            t = now()
            if dirty or t >= t_next:
                run_pub.publish(Bool(data=bool(want)))
                t_next = t + period

        # Dong GUI: bao dung xe vai lan cho chac roi moi thoat
        try:
            for _ in range(3):
                run_pub.publish(Bool(data=False))
                time.sleep(0.02)
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        rclpy.try_shutdown()


# ============================================================================
# KHOI GIAO DIEN NHO
# ============================================================================

class PanelHeader(QFrame):
    """Thanh tieu de panel: TIEU DE  phu de  ............  [badge] [badge]"""

    def __init__(self, title, subtitle="", n_badges=1):
        super().__init__()
        self.setObjectName("panelHeader")
        self.setFixedHeight(30)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(12, 0, 8, 0)
        lay.setSpacing(8)

        t = QLabel(title.upper())
        t.setObjectName("panelTitle")
        lay.addWidget(t)
        self.subtitle = QLabel(subtitle.upper())
        self.subtitle.setObjectName("panelSubtitle")
        lay.addWidget(self.subtitle)
        lay.addStretch()

        self.badges = []
        for _ in range(n_badges):
            b = Badge()
            lay.addWidget(b)
            self.badges.append(b)

    def set_subtitle(self, text):
        text = text.upper()
        if self.subtitle.text() != text:
            self.subtitle.setText(text)


class Badge(QLabel):
    """Nhan nho bo tron; chi doi stylesheet khi doi mau (setStyleSheet ton CPU)."""

    def __init__(self):
        super().__init__("")
        self._kind = None
        self.hide()

    def set(self, text, kind="blue"):
        if not text:
            self.hide()
            return
        if self.text() != text:
            self.setText(text)
        if kind != self._kind:
            self._kind = kind
            self.setStyleSheet(
                f"QLabel {{ background: {KIND_SOFT[kind]}; color: {KIND_COLOR[kind]};"
                f" border-radius: 9px; padding: 2px 8px; font-family: {FONT_MONO};"
                f" font-size: 8pt; font-weight: 700; }}")
        self.show()


class ImageView(QWidget):
    """Ve anh numpy bang QPainter: giu ti le, nen toi, khong copy du lieu."""

    def __init__(self, placeholder="WAITING FOR IMAGE"):
        super().__init__()
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setMinimumSize(80, 40)
        self.image = QImage()
        self._buf = None             # giu mang numpy song cung QImage
        self.placeholder = placeholder
        self.stale = True
        self._bg = QColor(C_VIEW_BG)
        self._font = QFont(FONT_UI_FAMILY, 9)
        self._font.setBold(True)

    def set_frame(self, arr):
        arr = np.ascontiguousarray(arr)
        h, w = arr.shape[:2]
        fmt = QImage.Format.Format_Grayscale8 if arr.ndim == 2 else QImage.Format.Format_BGR888
        self.image = QImage(arr.data, w, h, arr.strides[0], fmt)
        self._buf = arr
        self.stale = False
        self.update()

    def set_stale(self, stale):
        if stale != self.stale:
            self.stale = stale
            self.update()

    def set_placeholder(self, text):
        if text != self.placeholder:
            self.placeholder = text
            self.update()

    def paintEvent(self, _event):
        p = QPainter(self)
        p.fillRect(self.rect(), self._bg)
        if self.image.isNull():
            self._center_text(p, self.placeholder)
            p.end()
            return

        iw, ih = self.image.width(), self.image.height()
        scale = min(self.width() / iw, self.height() / ih)
        dw, dh = iw * scale, ih * scale
        target = QRectF((self.width() - dw) / 2, (self.height() - dh) / 2, dw, dh)
        p.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform, True)
        p.drawImage(target, self.image)

        if self.stale:
            p.fillRect(target, QColor(11, 18, 32, 170))
            self._center_text(p, self.placeholder)
        p.end()

    def _center_text(self, p, text):
        p.setPen(QColor("#94A3B8"))
        p.setFont(self._font)
        p.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, text)


class Card(QFrame):
    """Khung trang bo goc co header."""

    def __init__(self, title, subtitle="", n_badges=1):
        super().__init__()
        self.setObjectName("card")
        self.root = QVBoxLayout(self)
        self.root.setContentsMargins(1, 1, 1, 1)
        self.root.setSpacing(0)
        self.header = PanelHeader(title, subtitle, n_badges)
        self.root.addWidget(self.header)


class ImageCard(Card):
    def __init__(self, title, subtitle="", n_badges=1, placeholder="WAITING FOR IMAGE"):
        super().__init__(title, subtitle, n_badges)
        self.view = ImageView(placeholder)
        self.root.addWidget(self.view, 1)


class Metric(QFrame):
    """O so lieu: NHAN nho o tren, GIA TRI lon o duoi."""

    def __init__(self, label, unit=""):
        super().__init__()
        self.setObjectName("metric")
        self.setMinimumWidth(0)
        self.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(9, 6, 6, 6)
        lay.setSpacing(1)
        lab = QLabel(label)
        lab.setObjectName("metricLabel")
        row = QHBoxLayout()
        row.setSpacing(4)
        self.value = QLabel("--")
        self.value.setObjectName("metricValue")
        self.unit = QLabel(unit)
        self.unit.setObjectName("metricUnit")
        row.addWidget(self.value)
        row.addWidget(self.unit, 0, Qt.AlignmentFlag.AlignBottom)
        row.addStretch()
        lay.addWidget(lab)
        lay.addLayout(row)
        self._color = None

    def set(self, text, color=None):
        if self.value.text() != text:
            self.value.setText(text)
        color = color or C_TEXT
        if color != self._color:
            self._color = color
            self.value.setStyleSheet(f"color: {color};")


class LinkPill(QLabel):
    """Den ket noi tren header: ● CAMERA 30 Hz"""

    def __init__(self, name):
        super().__init__()
        self.name = name
        self._state = None
        self.set_state(False, "")

    def set_state(self, ok, extra):
        state = (ok, extra)
        if state == self._state:
            return
        self._state = state
        color = "#4ADE80" if ok else "#F87171"
        self.setText(f'<span style="color:{color}">●</span>&nbsp;{self.name}'
                     f'<span style="color:#94A3B8">&nbsp;{extra}</span>')


class RunButton(QPushButton):
    """Nut CHAY/DUNG. NoFocus de phim SPACE khong 'bam' nut 2 lan."""

    STYLES = {
        "start": (C_GREEN, "#15803D", "▶   CHẠY XE      [ SPACE ]"),
        "stop": (C_RED, "#B91C1C", "■   DỪNG XE      [ SPACE ]"),
        "wait": (C_ORANGE, "#B45309", "…   ĐANG CHỜ XE  [ SPACE ]"),
        "offline": ("#94A3B8", "#94A3B8", "MẤT KẾT NỐI FUSION NODE"),
    }

    def __init__(self):
        super().__init__()
        self.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setFixedHeight(48)
        self._mode = None
        self.set_mode("start")

    def set_mode(self, mode):
        if mode == self._mode:
            return
        self._mode = mode
        bg, hover, text = self.STYLES[mode]
        self.setText(text)
        self.setStyleSheet(
            f"QPushButton {{ background: {bg}; color: white; border: none;"
            f" border-radius: 8px; font-family: {FONT_UI}; font-size: 11pt;"
            f" font-weight: 800; letter-spacing: 1px; }}"
            f"QPushButton:hover {{ background: {hover}; }}"
            f"QPushButton:pressed {{ padding-top: 2px; }}")


# ============================================================================
# BAN DO LiDAR (ve bang QPainter, du 360 do)
# ============================================================================

class LidarView(QWidget):
    """Ban do quet 360 do cua RPLIDAR A1M8 quanh xe, ve truc tiep bang QPainter.

    Quy luat ve: du lieu da o KHUNG XE (xem RosWorker.on_scan, goc REP-103
    nguoc chieu kim dong ho). Xe o tam, mui xe huong LEN. Vach chia do ghi
    0..330 do THEO CHIEU KIM DONG HO tu mui xe (giong cach A1M8 / RoboStudio
    danh so goc), kem nhan TRUOC / PHAI / SAU / TRAI.

    Chong che chu: so do nam NGOAI vong tron (le danh rieng), nhan khoang
    cach co nen trang ve SAU cung (diem do khong de len chu), co chu tu nho
    lai theo kich thuoc khung. Vat xa hon tam nhin van hien la vach nho o
    mep vong -> luon thay du 360 do quanh xe."""

    def __init__(self):
        super().__init__()
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setMinimumSize(200, 200)
        self.range_idx = LIDAR_RANGES_M.index(LIDAR_DEFAULT_RANGE_M)
        self.theta = np.empty(0, np.float32)
        self.dist = np.empty(0, np.float32)
        self.on_range = None
        self._colors = [QColor(c) for c in LIDAR_COLORS]
        self._pens = [QPen(QColor("#1E293B"), 0.6) for _ in LIDAR_COLORS]

    def range_m(self):
        return LIDAR_RANGES_M[self.range_idx]

    def set_scan(self, theta, dist):
        self.theta, self.dist = theta, dist
        self.update()

    def wheelEvent(self, event):
        d = event.angleDelta().y()
        old = self.range_idx
        if d > 0:
            self.range_idx = max(0, self.range_idx - 1)
        elif d < 0:
            self.range_idx = min(len(LIDAR_RANGES_M) - 1, self.range_idx + 1)
        if self.range_idx != old:
            if self.on_range:
                self.on_range(self.range_m())
            self.update()

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        p.fillRect(self.rect(), QColor(C_CARD))
        w, h = self.width(), self.height()
        side = min(w, h)
        fs = max(6.5, min(9.0, side / 48.0))          # co chu theo khung
        font = QFont(FONT_UI_FAMILY)
        font.setPointSizeF(fs)
        p.setFont(font)
        fm = p.fontMetrics()
        margin = fm.horizontalAdvance("330°") + 8      # le cho so do ngoai vong
        R = max(40.0, side / 2.0 - margin)
        cx, cy = w / 2.0, h / 2.0
        r_mm = self.range_m() * 1000.0
        k = R / r_mm                                    # px / mm

        # ---- Luoi: vong khoang cach + tia 30 do ----
        step = {0.5: 0.1, 1: 0.2, 2: 0.5, 3: 0.5, 4: 1, 6: 1, 8: 2, 12: 2}[self.range_m()]
        rings = np.arange(step, self.range_m() + 1e-6, step)
        grid = QPen(QColor("#E2E8F0"), 1)
        p.setPen(grid)
        p.setBrush(Qt.BrushStyle.NoBrush)
        for rr in rings:
            r = rr * 1000.0 * k
            p.drawEllipse(QPointF(cx, cy), r, r)
        for a in range(0, 360, 30):
            t = math.radians(a)
            p.drawLine(QPointF(cx, cy), QPointF(cx + R * math.sin(t), cy - R * math.cos(t)))
        p.setPen(QPen(QColor("#94A3B8"), 1.2))
        p.drawEllipse(QPointF(cx, cy), R, R)

        # Vung mu 15 cm cua A1M8
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QColor(203, 213, 225, 140))
        rb = LIDAR_BLIND_MM * k
        p.drawEllipse(QPointF(cx, cy), rb, rb)

        # ---- Diem do ----
        if self.dist.size:
            th = self.theta.astype(np.float64)
            d = self.dist.astype(np.float64)
            inside = d <= r_mm
            x = cx - np.sin(th) * np.minimum(d, r_mm) * k   # 90 do (trai) -> sang trai
            y = cy - np.cos(th) * np.minimum(d, r_mm) * k   # 0 do (truoc) -> len tren
            band = np.searchsorted(np.asarray(LIDAR_COLOR_BOUNDS_MM[1:-1]), d, side="right")
            pr = max(1.8, side / 170.0)
            for b in range(len(LIDAR_COLORS)):
                sel = inside & (band == b)
                if not sel.any():
                    continue
                p.setPen(self._pens[b])
                p.setBrush(self._colors[b])
                for xi, yi in zip(x[sel].tolist(), y[sel].tolist()):
                    p.drawEllipse(QPointF(xi, yi), pr, pr)
            # Xa hon tam nhin: vach nho o mep vong (biet huong co vat)
            out = ~inside
            if out.any():
                p.setPen(QPen(QColor(21, 128, 61, 70), 1.2))
                for t in th[out].tolist():
                    s, c = math.sin(t), math.cos(t)
                    p.drawLine(QPointF(cx - s * (R - 4), cy - c * (R - 4)),
                               QPointF(cx - s * R, cy - c * R))

        # ---- Xe o tam, mui huong len ----
        car = max(6.0, side / 40.0)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QColor(C_TEXT))
        p.drawPolygon([QPointF(cx, cy - car), QPointF(cx - 0.7 * car, cy + 0.7 * car),
                       QPointF(cx + 0.7 * car, cy + 0.7 * car)])

        # ---- Chu (ve sau cung -> khong bi diem de len) ----
        # So do NGOAI vong, chieu kim dong ho tu mui xe
        p.setPen(QColor(C_TEXT_2))
        rl = R + 4 + fm.horizontalAdvance("330°") / 2.0
        for a in range(0, 360, 30):
            t = math.radians(a)
            tx, ty = cx + rl * math.sin(t), cy - rl * math.cos(t)
            txt = f"{a}°"
            tw = fm.horizontalAdvance(txt)
            p.drawText(QRectF(tx - tw / 2 - 2, ty - fm.height() / 2, tw + 4, fm.height()),
                       Qt.AlignmentFlag.AlignCenter, txt)
        # Huong xe ngay trong vong
        bold = QFont(font)
        bold.setBold(True)
        p.setFont(bold)
        bfm = p.fontMetrics()
        p.setPen(QColor(C_BLUE))
        for a, txt in ((0, "TRƯỚC"), (90, "PHẢI"), (180, "SAU"), (270, "TRÁI")):
            t = math.radians(a)
            rr = R - bfm.height() * (0.9 if a in (0, 180) else 1.0) - \
                (bfm.horizontalAdvance(txt) / 2.0 if a in (90, 270) else 0.0)
            tx, ty = cx + rr * math.sin(t), cy - rr * math.cos(t)
            self._label(p, bfm, tx, ty, txt, QColor(C_BLUE))
        # Nhan khoang cach tren tia 135 do (sau-phai), nen trang
        p.setFont(font)
        t = math.radians(135)
        for rr in rings[1::2] if len(rings) > 4 else rings:
            r = rr * 1000.0 * k
            self._label(p, fm, cx + r * math.sin(t), cy - r * math.cos(t), f"{rr:g}m",
                        QColor(C_MUTED))
        p.end()

    @staticmethod
    def _label(p, fm, x, y, txt, color):
        tw, th = fm.horizontalAdvance(txt), fm.height()
        box = QRectF(x - tw / 2 - 3, y - th / 2, tw + 6, th)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QColor(255, 255, 255, 215))
        p.drawRoundedRect(box, 3, 3)
        p.setPen(color)
        p.drawText(box, Qt.AlignmentFlag.AlignCenter, txt)


class LidarMap(Card):
    """The ban do LiDAR: LidarView + chan the (vat gan nhat, so diem, canh bao,
    chu thich mau). Lan chuot tren ban do de doi tam nhin 0.5..12 m."""

    def __init__(self):
        super().__init__("LiDAR MAP", "A1M8 · 360° · FRONT ↑", n_badges=1)
        self.view = LidarView()
        self.view.on_range = self._show_range
        self.root.addWidget(self.view, 1)

        footer = QFrame()
        footer.setObjectName("cardFooter")
        fl = QHBoxLayout(footer)
        fl.setContentsMargins(12, 4, 12, 5)
        fl.setSpacing(14)
        self.nearest_label = QLabel("NEAR --")
        self.points_label = QLabel("PTS --")
        self.status_label = QLabel("--")
        for lab in (self.nearest_label, self.points_label, self.status_label):
            lab.setObjectName("footMetric")
            fl.addWidget(lab)
        fl.addStretch()
        c = LIDAR_COLORS
        legend = QLabel(f'<span style="color:{c[0]}">●</span>.4 '
                        f'<span style="color:{c[1]}">●</span>.6 '
                        f'<span style="color:{c[2]}">●</span>1.5 '
                        f'<span style="color:{c[3]}">●</span>&gt; m')
        legend.setToolTip("Mau theo khoang cach: do < 0.4 m, cam < 0.6 m, vang < 1.5 m, "
                          "xanh xa hon. Vong xam o tam = vung mu 15 cm cua LiDAR. "
                          "Vach o mep vong = vat xa hon tam nhin.")
        legend.setObjectName("footLegend")
        fl.addWidget(legend)
        self.legend = legend
        self.root.addWidget(footer)
        self._status_kind = None
        self._show_range(self.view.range_m())

    def _show_range(self, r_m):
        self.header.badges[0].set(f"RANGE {r_m:g} m", "muted")

    def resizeEvent(self, event):
        # Khung hep: bo chu thich mau de cac so lieu khong bi cat
        self.legend.setVisible(self.width() >= 420)
        super().resizeEvent(event)

    def set_scan(self, theta, dist):
        self.view.set_scan(theta, dist)
        if dist.size:
            i = int(np.argmin(dist))
            # Goc theo chieu kim dong ho tu mui xe, 0..360 (nhu tren ban do)
            deg = (360.0 - math.degrees(float(theta[i]))) % 360.0
            self.nearest_label.setText(f"NEAR {dist[i] / 1000:.2f} m @ {deg:.0f}°")
        else:
            self.nearest_label.setText("NEAR --")
        self.points_label.setText(f"PTS {dist.size}")

    def set_alert(self, alert):
        """Muc canh bao lay tu C++ (LidarModule: <40 cm DANGER, <60 cm WARNING)."""
        kind = {"DANGER": "red", "WARNING": "orange", "CLEAR": "green"}.get(alert, "muted")
        text = alert or "--"
        if self.status_label.text() != text:
            self.status_label.setText(text)
        if kind != self._status_kind:
            self._status_kind = kind
            self.status_label.setStyleSheet(f"color: {KIND_COLOR[kind]};")


# ============================================================================
# KHUNG DIEU KHIEN: trang thai + nut SPACE + so lieu
# ============================================================================

class ControlCard(Card):
    def __init__(self, on_toggle):
        super().__init__("VEHICLE CONTROL", "", n_badges=1)
        body = QWidget()
        lay = QVBoxLayout(body)
        lay.setContentsMargins(12, 10, 12, 12)
        lay.setSpacing(10)

        top = QHBoxLayout()
        top.setSpacing(8)
        self.state = QLabel("WAITING")
        self.state.setObjectName("stateBig")
        self.state_detail = QLabel("")
        self.state_detail.setObjectName("stateDetail")
        top.addWidget(self.state)
        top.addStretch()
        top.addWidget(self.state_detail, 0, Qt.AlignmentFlag.AlignVCenter)
        lay.addLayout(top)

        self.button = RunButton()
        self.button.clicked.connect(on_toggle)
        lay.addWidget(self.button)

        grid = QGridLayout()
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(8)
        self.m = {
            "speed": Metric("SPEED CMD", "km/h"),
            "esc": Metric("ESC (ESP32)", "°"),
            "servo": Metric("SERVO", "°"),
            "dev": Metric("LANE DEV", "cm"),
            "width": Metric("LANE WIDTH", "cm"),
            "curv": Metric("CURVATURE", "1/m"),
            "fps": Metric("CAMERA", "fps"),
            "proc": Metric("DETECT", "ms"),
            "front": Metric("FRONT", "cm"),
        }
        order = ("speed", "esc", "servo", "dev", "width", "curv", "fps", "proc", "front")
        for k, key in enumerate(order):
            grid.addWidget(self.m[key], k // 3, k % 3)
        for c in range(3):
            grid.setColumnStretch(c, 1)
        lay.addLayout(grid)
        self.root.addWidget(body, 1)
        self._state_color = None

    def set_state(self, text, color, detail=""):
        if self.state.text() != text:
            self.state.setText(text)
        if color != self._state_color:
            self._state_color = color
            self.state.setStyleSheet(f"color: {color};")
        if self.state_detail.text() != detail:
            self.state_detail.setText(detail)


# ============================================================================
# CUA SO CHINH
# ============================================================================

class AutoCarMonitor(QMainWindow):
    def __init__(self, hub, worker, fps=60, width=1280, height=760):
        super().__init__()
        self.hub = hub
        self.worker = worker
        self.setWindowTitle(APP_TITLE)
        self.resize(width, height)
        self.setMinimumSize(1024, 640)

        self.t_start = now()
        self.seen = {"vis": -1, "roi": -1, "bin": -1, "scan": -1, "status": -1}
        self.t_status_ui = 0.0
        self.gui_rate = RateMeter()

        self._apply_style()
        self._build_ui()

        # Phim tat toan cua so: SPACE = chay/dung, ESC = dung ngay
        ctx = Qt.ShortcutContext.ApplicationShortcut
        self.sc_space = QShortcut(QKeySequence(Qt.Key.Key_Space), self)
        self.sc_space.setContext(ctx)
        self.sc_space.activated.connect(self.toggle_run)
        self.sc_esc = QShortcut(QKeySequence(Qt.Key.Key_Escape), self)
        self.sc_esc.setContext(ctx)
        self.sc_esc.activated.connect(lambda: self.hub.set_run(False))

        self.timer = QTimer(self)
        self.timer.setTimerType(Qt.TimerType.PreciseTimer)
        self.timer.timeout.connect(self.tick)
        self.timer.start(max(5, round(1000 / max(20, min(120, int(fps))))))

    # ------------------------------------------------------------------ style
    def _apply_style(self):
        self.setStyleSheet(f"""
            QMainWindow, #central {{ background: {C_BG}; }}
            QWidget {{ font-family: {FONT_UI}; color: {C_TEXT}; }}

            #header {{ background: {C_HEAD}; }}
            #appTitle {{ color: white; font-size: 13pt; font-weight: 800; letter-spacing: 1px; }}
            #appSubtitle {{ color: #94A3B8; font-size: 8pt; }}
            #headerInfo {{ color: #CBD5E1; font-family: {FONT_MONO}; font-size: 8.5pt; }}
            LinkPill, #linkPill {{ color: #E2E8F0; font-size: 8.5pt; font-weight: 700;
                background: {C_HEAD_2}; border-radius: 10px; padding: 3px 10px; }}

            #card {{ background: {C_CARD}; border: 1px solid {C_LINE}; border-radius: 8px; }}
            #panelHeader {{ background: transparent; border-bottom: 1px solid {C_LINE_2}; }}
            #panelTitle {{ color: {C_TEXT}; font-size: 8.5pt; font-weight: 800; letter-spacing: 1px; }}
            #panelSubtitle {{ color: {C_MUTED}; font-size: 7.5pt; font-weight: 600; }}

            #stateBig {{ font-size: 15pt; font-weight: 800; letter-spacing: 1px; }}
            #stateDetail {{ color: {C_MUTED}; font-family: {FONT_MONO}; font-size: 8.5pt; }}

            #metric {{ background: {C_SOFT}; border: 1px solid {C_LINE_2}; border-radius: 6px; }}
            #metricLabel {{ color: {C_MUTED}; font-size: 7pt; font-weight: 800; letter-spacing: 1px; }}
            #metricValue {{ font-family: {FONT_MONO}; font-size: 12pt; font-weight: 700; }}
            #metricUnit {{ color: {C_MUTED}; font-family: {FONT_MONO}; font-size: 7.5pt; padding-bottom: 2px; }}

            #cardFooter {{ border-top: 1px solid {C_LINE_2}; }}
            #footMetric {{ color: {C_TEXT_2}; font-family: {FONT_MONO}; font-size: 8pt; font-weight: 700; }}
            #footLegend {{ color: {C_MUTED}; font-family: {FONT_MONO}; font-size: 7.5pt; }}

            #bottomBar {{ background: {C_HEAD}; }}
            #bottomText {{ color: #94A3B8; font-family: {FONT_MONO}; font-size: 8pt; }}
            #keyHint {{ color: #E2E8F0; font-size: 8pt; font-weight: 700; }}
        """)

    # --------------------------------------------------------------------- UI
    def _build_ui(self):
        central = QWidget()
        central.setObjectName("central")
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)
        root.addWidget(self._build_header())
        root.addWidget(self._build_body(), 1)
        root.addWidget(self._build_footer())

    def _build_header(self):
        header = QFrame()
        header.setObjectName("header")
        header.setFixedHeight(46)
        lay = QHBoxLayout(header)
        lay.setContentsMargins(16, 0, 16, 0)
        lay.setSpacing(10)

        title = QLabel("AUTOCAR MONITOR")
        title.setObjectName("appTitle")
        subtitle = QLabel("Autonomous vehicle · lane following")
        subtitle.setObjectName("appSubtitle")
        self.app_subtitle = subtitle
        lay.addWidget(title)
        lay.addWidget(subtitle)
        lay.addStretch()

        self.pills = {}
        for name in LINKS:
            pill = LinkPill(name)
            pill.setObjectName("linkPill")
            self.pills[name] = pill
            lay.addWidget(pill)

        lay.addSpacing(10)
        self.header_info = QLabel("")
        self.header_info.setObjectName("headerInfo")
        lay.addWidget(self.header_info)
        return header

    def _build_body(self):
        body = QWidget()
        grid = QGridLayout(body)
        grid.setContentsMargins(12, 12, 12, 12)
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(12)

        # Trai: anh camera + overlay C++ (lon), duoi la ROI | mask
        self.cam_card = ImageCard("LANE DETECTION", "C++ overlay", n_badges=2,
                                  placeholder="WAITING FOR CAMERA")
        self.roi_card = ImageCard("ROI", "camera crop", n_badges=0)
        self.bin_card = ImageCard("BINARY MASK", "chỉ 2 vạch làn đang bám", n_badges=0)

        left = QWidget()
        ll = QGridLayout(left)
        ll.setContentsMargins(0, 0, 0, 0)
        ll.setHorizontalSpacing(12)
        ll.setVerticalSpacing(12)
        ll.addWidget(self.cam_card, 0, 0, 1, 2)
        ll.addWidget(self.roi_card, 1, 0)
        ll.addWidget(self.bin_card, 1, 1)
        ll.setRowStretch(0, 3)
        ll.setRowStretch(1, 1)
        ll.setColumnStretch(0, 1)
        ll.setColumnStretch(1, 1)

        # Phai: dieu khien + LiDAR
        self.control = ControlCard(self.toggle_run)
        self.lidar = LidarMap()
        right = QWidget()
        rl = QVBoxLayout(right)
        rl.setContentsMargins(0, 0, 0, 0)
        rl.setSpacing(12)
        rl.addWidget(self.control, 0)
        rl.addWidget(self.lidar, 1)

        # Ignored: be rong 2 cot chi theo ti le stretch, khong bi noi dung ep
        for w, min_w in ((left, 520), (right, 360)):
            w.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
            w.setMinimumWidth(min_w)
        grid.addWidget(left, 0, 0)
        grid.addWidget(right, 0, 1)
        # 66/34: o 1280x760 khung camera 16:9 vua khit, khong thua dai den
        grid.setColumnStretch(0, 66)
        grid.setColumnStretch(1, 34)
        return body

    def _build_footer(self):
        footer = QFrame()
        footer.setObjectName("bottomBar")
        footer.setFixedHeight(26)
        lay = QHBoxLayout(footer)
        lay.setContentsMargins(16, 0, 16, 0)
        hint = QLabel("SPACE  chạy / dừng      ESC  dừng ngay      Lăn chuột trên LiDAR  đổi tầm nhìn")
        hint.setObjectName("keyHint")
        self.footer_info = QLabel("")
        self.footer_info.setObjectName("bottomText")
        lay.addWidget(hint)
        lay.addStretch()
        lay.addWidget(self.footer_info)
        return footer

    # ----------------------------------------------------------------- hanh dong
    def toggle_run(self):
        # Chi cho bat chay khi fusion_node dang song
        if not self.hub.want_run and not self.hub.rates["FUSION"].alive():
            return
        self.hub.set_run(not self.hub.want_run)
        self._update_status_ui(force=True)

    # --------------------------------------------------------------- realtime
    def tick(self):
        hub = self.hub
        with hub.lock:
            images = dict(hub.images)
            seq = dict(hub.image_seq)
            t_img = dict(hub.image_t)
            scan, scan_seq = hub.scan, hub.scan_seq

        # Anh: chi ve lai khi co khung MOI
        t = now()
        for key, card in (("vis", self.cam_card), ("roi", self.roi_card), ("bin", self.bin_card)):
            if images[key] is not None and seq[key] != self.seen[key]:
                self.seen[key] = seq[key]
                card.view.set_frame(images[key])
                if key == "vis":
                    self.gui_rate.tick()
            card.view.set_stale(t_img[key] == 0 or t - t_img[key] > NO_SIGNAL_S)

        # LiDAR: chi khi co scan moi
        if scan is not None and scan_seq != self.seen["scan"]:
            self.seen["scan"] = scan_seq
            self.lidar.set_scan(*scan)

        if t - self.t_status_ui >= STATUS_UI_PERIOD_S:
            self._update_status_ui()

    def _update_status_ui(self, force=False):
        self.t_status_ui = now()
        hub = self.hub
        with hub.lock:
            st = dict(hub.status)
            want_run = hub.want_run
        for r in hub.rates.values():
            r.update()
        self.gui_rate.update()

        fusion_ok = hub.rates["FUSION"].alive()
        num = lambda k: parse_num(st.get(k)) if fusion_ok else None  # noqa: E731

        # ---- den ket noi ----
        fbage = num("fbage")
        esp_ok = fusion_ok and st.get("serial") == "open" and fbage is not None and 0 <= fbage <= 300
        cam_hz = num("fps")
        self.pills["FUSION"].set_state(fusion_ok, f"{hub.rates['FUSION'].hz:.0f} Hz" if fusion_ok else "")
        age = num("age")
        cam_ok = age is not None and 0 <= age < 300
        if not fusion_ok:
            cam_msg = "WAITING FOR FUSION NODE"
        elif age is not None and age < 0:
            cam_msg = "CAMERA CHƯA KẾT NỐI · đang tự thử lại mỗi 2 s"
        elif not cam_ok:
            cam_msg = "CAMERA MẤT TÍN HIỆU · đang tự kết nối lại"
        else:
            cam_msg = "WAITING FOR IMAGE"
        self.cam_card.view.set_placeholder(cam_msg)
        self.pills["CAMERA"].set_state(cam_ok, f"{cam_hz:.0f} fps" if cam_ok and cam_hz else "")
        lidar_rate = hub.rates["LIDAR"]
        self.pills["LIDAR"].set_state(lidar_rate.alive(), f"{lidar_rate.hz:.1f} Hz" if lidar_rate.alive() else "")
        self.pills["ESP32"].set_state(esp_ok, "UART" if esp_ok else "")

        # ---- trang thai xe ----
        run = num("run") == 1
        emg = num("emg") == 1
        track = st.get("track")
        c = self.control
        if not fusion_ok:
            c.set_state("OFFLINE", C_MUTED, "no /lane/status")
            c.button.set_mode("offline" if not want_run else "stop")
        else:
            if not run:
                c.set_state("STOPPED", C_TEXT_2, "bấm SPACE để chạy")
            elif emg:
                c.set_state("EMERGENCY", C_RED, "mất camera" if not cam_ok else "mất làn")
            elif esc is not None and esc <= 90 and (spd or 0) > 0:
                # Mini PC gui lenh chay nhung ESP32 van phat neutral
                why = ("ESP32 mất lệnh (watchdog)" if num("fwwd") == 1 else
                       "ESP32 chưa nhận gói lệnh" if num("fwcmd") == 0 else
                       "ESP32 đang EMG" if num("fwemg") == 1 else "ESP32 chưa ra ga")
                c.set_state("NO THROTTLE", C_RED, why)
            elif track == "two":
                c.set_state("FOLLOW LANE", C_GREEN, "2 vạch")
            elif track == "one":
                c.set_state("ONE LINE", C_ORANGE, "1 vạch · chạy chậm")
            else:
                c.set_state("LANE LOST", C_ORANGE, "giữ hướng")
            if want_run and run:
                c.button.set_mode("stop")
            elif want_run:
                c.button.set_mode("wait")
            else:
                c.button.set_mode("start")
        c.header.badges[0].set("RUNNING" if (fusion_ok and run) else "SAFE STOP",
                               "green" if (fusion_ok and run) else "muted")

        # ---- so lieu ----
        def show(key, value, fmt, color=None):
            c.m[key].set(fmt.format(value) if value is not None else "--", color)

        spd = num("spd")
        show("speed", spd, "{:.1f}")
        # Muc xung ESP32 THAT SU dang phat cho ESC (telemetry v2): 90 = dung,
        # > 90 = tien. -1 = firmware cu chua bao (can nap firmware moi).
        esc = num("esc")
        esc = esc if esc is not None and esc >= 0 else None
        show("esc", esc, "{:.0f}",
             None if esc is None else (C_GREEN if esc > 90 else C_TEXT_2))
        show("servo", num("servo"), "{:.1f}")
        dev = num("devm") if track in ("one", "two") else None
        show("dev", dev, "{:+.1f}",
             None if dev is None else (C_GREEN if abs(dev) < 3 else C_ORANGE if abs(dev) < 8 else C_RED))
        w = num("w")
        show("width", w if w is not None and w > 0 else None, "{:.0f}")
        show("curv", num("curv"), "{:+.2f}")
        show("fps", cam_hz, "{:.1f}")
        proc = num("proc")
        show("proc", proc, "{:.1f}")
        front = num("front")
        front = front if front is not None and front >= 0 else None
        show("front", front, "{:.0f}",
             None if front is None else (C_RED if front < 40 else C_ORANGE if front < 60 else C_GREEN))

        self.lidar.set_alert(st.get("alert") if fusion_ok else None)

        # ---- badge camera ----
        cam_size = st.get("cam", "")
        self.cam_card.header.set_subtitle(f"C++ overlay · {cam_size.replace('x', '×')}" if cam_size else "C++ overlay")
        badges = self.cam_card.header.badges
        badges[0].set(f"{cam_hz:.0f} FPS" if cam_hz else "", "blue")
        gate = num("gate") == 1
        badges[1].set(f"{proc:.1f} ms" + (" · GATED" if gate else "") if proc is not None else "",
                      "orange" if gate else "green")

        # ---- header / footer ----
        self.header_info.setText(f"{time.strftime('%H:%M:%S')}  ·  UP {fmt_uptime(now() - self.t_start)}")
        self.footer_info.setText(
            f"{QT_BINDING}  ·  GUI {self.gui_rate.hz:4.1f} fps  ·  "
            f"dev {st.get('dev', '--')} px  ·  age {st.get('age', '--')}"
            + (f"  ·  {self.worker.error}" if self.worker.error else ""))

    def resizeEvent(self, event):
        # Cua so hep: an phu de de tieu de va den ket noi khong de len nhau
        self.app_subtitle.setVisible(self.width() >= 1200)
        super().resizeEvent(event)

    def closeEvent(self, event):
        self.hub.set_run(False)
        self.timer.stop()
        event.accept()


# ============================================================================
# MAIN
# ============================================================================

def main():
    parser = argparse.ArgumentParser(description=APP_TITLE)
    parser.add_argument("--fps", type=int, default=60, help="Nhip kiem tra/ve GUI (Hz)")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=760)
    parser.add_argument("--lidar-offset", type=float, default=DEFAULT_LIDAR_OFFSET_DEG,
                        help="Goc lap LiDAR (do) khi chua nhan duoc lofs tu fusion_node")
    args = parser.parse_args()

    app = QApplication(sys.argv)
    app.setApplicationName(APP_TITLE)

    # Ctrl+C / kill: dong cua so dang hoang (gui lenh dung xe roi moi thoat).
    # Timer GUI chay 60 Hz nen Python kip xu ly tin hieu.
    import signal
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    signal.signal(signal.SIGTERM, lambda *_: app.quit())
    app.setFont(QFont(FONT_UI_FAMILY, 9))

    hub = Hub(args.lidar_offset)
    worker = RosWorker(hub)
    worker.start()

    window = AutoCarMonitor(hub, worker, fps=args.fps, width=args.width, height=args.height)
    window.show()

    code = app.exec() if hasattr(app, "exec") else app.exec_()
    hub.set_run(False)
    worker.stop()
    worker.join(timeout=1.0)
    sys.exit(code)


if __name__ == "__main__":
    main()
