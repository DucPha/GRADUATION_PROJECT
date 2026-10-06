#!/usr/bin/env python3
"""
Python + Matplotlib Dashboard for Lane Detection
White theme, polar LIDAR map, synchronized status
"""

import math
import sys
import threading
import time

from collections import deque
from datetime import timedelta

import matplotlib
matplotlib.use('QtAgg')  # Stable on Linux

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.gridspec import GridSpec

import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image, LaserScan

# BAT BUOC: nap PyQt5 + tao QApplication TRUOC cv_bridge.
# cv_bridge tu nap cv2 (ban pip trong ~/.local co Qt goi kem). Neu cv2 nap truoc,
# libQt5Core cua cv2 se chiem vao QApplication -> Qt tim plugin xcb trong thu muc
# cua cv2 -> "Could not load the Qt platform plugin xcb" va crash khi mo cua so.
from PyQt5.QtWidgets import QApplication
QApplication.instance() or QApplication([])

from cv_bridge import CvBridge


def _parse_status_line(line: str) -> dict:
    """Parse space-separated key=value status string from /lane/status"""
    res = {}
    if not line:
        return res
    # Split by whitespace
    parts = line.strip().split()
    for p in parts:
        if '=' not in p:
            continue
        k, v = p.split('=', 1)
        # Try numeric
        try:
            # Int or float? check if has dot or ms/cm suffix handled later? but store as string then convert
            # Remove common suffixes for numeric parse where appropriate
            res[k] = v
        except Exception:
            res[k] = v
    return res


def _to_float(v: str) -> float:
    if v is None:
        return -1.0
    s = str(v).strip()
    # Remove common units
    for suf in ('ms', 'cm', 'px', 'km/h', 'Hz'):
        if s.endswith(suf):
            s = s[:-len(suf)]
            break
    try:
        f = float(s)
        # treat -1 as invalid marker? keep as is
        return f
    except ValueError:
        return -1.0


def _to_int(v: str) -> int:
    f = _to_float(v)
    if f < 0:
        return -1
    return int(round(f))


def _fmt_cm(f: float) -> str:
    if f < 0.0:
        return '---'
    if f >= 199.5:
        return '>200 cm'
    return f'{int(round(f))} cm'


class DashboardMatplotlib(Node):
    def __init__(self):
        super().__init__('gui_matplotlib')

        self.bridge = CvBridge()

        # Latest data
        self.status_dict = {}
        self.status_str = ''
        self.status_ts = 0.0

        self.img_bgr = None
        self.img_ts = 0.0

        self.scan = None
        self.scan_ts = 0.0

        # Trends (time in seconds relative to now, -10s -> 0s)
        self.t_deque = deque(maxlen=600)  # ~10s @ 60
        self.dev_px_deque = deque(maxlen=600)
        self.dev_cm_deque = deque(maxlen=600)
        self.t0 = time.time()

        # Subscribers
        self.sub_status = self.create_subscription(
            String, '/lane/status', self._status_cb, 10)
        self.sub_vis = self.create_subscription(
            Image, '/lane/vis', self._vis_cb, 10)
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self._scan_cb, 10)

        # Timing
        self._last_anim_t = time.time()

        # Build figure
        self.fig = plt.figure(figsize=(10, 7), dpi=100)
        self.fig.patch.set_facecolor('white')

        gs = GridSpec(2, 2, figure=self.fig, hspace=0.32, wspace=0.28)

        # Axes
        self.ax_img = self.fig.add_subplot(gs[0, 0])
        self.ax_polar = self.fig.add_subplot(gs[0, 1], projection='polar')
        self.ax_trend = self.fig.add_subplot(gs[1, 0])
        self.ax_status = self.fig.add_subplot(gs[1, 1])

        # Style
        for ax in (self.ax_img, self.ax_polar, self.ax_trend, self.ax_status):
            ax.patch.set_facecolor('white')

        # Image
        self.ax_img.axis('off')
        self.im_artist = None

        # Polar
        # NOTE: _setup_polar() da tao scat_artist + txt_*_artist cho self,
        # khong duoc gan lai None o day (gây FuncAnimation crash: NoneType.set_animated)
        self._setup_polar()

        # Trend
        self._setup_trend()
        self.line_px, = self.ax_trend.plot([], [], color='#0072BD', lw=2, label='dev_px')
        self.line_cm, = self.ax_trend.plot([], [], color='#219947', lw=2, label='dev_cm')
        self.ax_trend.legend(loc='upper right', frameon=False, fontsize=8)

        # Status
        self._setup_status()
        self.txt_status = self.ax_status.text(
            0.05, 0.95, '', va='top', ha='left',
            fontsize=9, family='monospace', color='#141414',
            transform=self.ax_status.transAxes)

        # Animation with blitting
        self.anim = animation.FuncAnimation(
            self.fig, self._update_anim, init_func=self._init_anim,
            interval=100, blit=True, cache_frame_data=False)

        self.fig.canvas.manager.set_window_title('Autonomous Vehicle - Matplotlib Dashboard')
        plt.tight_layout()

    def _setup_polar(self):
        ax = self.ax_polar
        ax.set_theta_zero_location('N')  # 0° up = FRONT
        ax.set_theta_direction(-1)  # clockwise
        ax.grid(True, linestyle='--', linewidth=0.4, color='#f0f0f0', alpha=0.4)
        ax.set_ylim(0, 150)  # cm
        ax.set_yticks([20, 40, 60, 80, 100, 120, 150])
        ax.set_yticklabels(['20', '40', '60', '80', '100', '120', '150'])
        ax.tick_params(colors='#666666', labelsize=7)
        ax.title.set_text('LIDAR MAP')
        ax.title.set_color('#141414')
        ax.title.set_fontsize(9)

        # Range rings
        for r in (20, 40, 60, 80, 100, 120):
            circle = plt.Circle((0, 0), r, transform=ax.transData._b, fill=False,
                                linestyle='--', linewidth=0.5, color='#f0f0f0', alpha=0.4)
            ax.add_patch(circle)

        # Direction labels
        ax.text(math.radians(90), 165, 'FRONT', ha='center', va='center', fontsize=8, color='#141414')
        ax.text(math.radians(180), 165, 'LEFT', ha='center', va='center', fontsize=8, color='#141414')
        ax.text(math.radians(270), 165, 'REAR', ha='center', va='center', fontsize=8, color='#141414')
        ax.text(math.radians(0), 165, 'RIGHT', ha='center', va='center', fontsize=8, color='#141414')

        # Robot center
        tri = mpatches.RegularPolygon((0, 0), numVertices=3, radius=5, orientation=math.radians(90),
                                      facecolor='#219947', edgecolor='#219947')
        ax.add_patch(tri)

        self.scat_artist = ax.scatter([], [], s=8, c=[], cmap='cool_r', vmin=0, vmax=150, alpha=0.9)
        self.txt_f_artist = ax.text(math.radians(90), 135, 'F: ---', ha='center', va='center',
                                   fontsize=7, color='#141414', bbox=dict(boxstyle='round,pad=0.1',
                                                                           facecolor='white', edgecolor='#e0e0e0', alpha=0.9))
        self.txt_l_artist = ax.text(math.radians(180), 135, 'L: ---', ha='center', va='center',
                                   fontsize=7, color='#141414', bbox=dict(boxstyle='round,pad=0.1',
                                                                           facecolor='white', edgecolor='#e0e0e0', alpha=0.9))
        self.txt_r_artist = ax.text(math.radians(0), 135, 'R: ---', ha='center', va='center',
                                   fontsize=7, color='#141414', bbox=dict(boxstyle='round,pad=0.1',
                                                                           facecolor='white', edgecolor='#e0e0e0', alpha=0.9))
        self.txt_b_artist = ax.text(math.radians(270), 135, 'B: ---', ha='center', va='center',
                                   fontsize=7, color='#141414', bbox=dict(boxstyle='round,pad=0.1',
                                                                           facecolor='white', edgecolor='#e0e0e0', alpha=0.9))
        self.txt_lidar_state = ax.text(0, 0, '', ha='center', va='center', fontsize=10,
                                       color='#8C8C8C', transform=ax.transAxes)

    def _setup_trend(self):
        ax = self.ax_trend
        ax.set_facecolor('white')
        ax.grid(True, linestyle='--', linewidth=0.4, color='#f5f5f5')
        ax.set_xlim(-10.0, 0.0)
        ax.set_ylim(-80, 80)
        ax.tick_params(colors='#666666', labelsize=7)
        ax.set_xlabel('Time (s)', fontsize=8, color='#141414')
        ax.set_ylabel('Deviation', fontsize=8, color='#141414')
        ax.title.set_text('Lane Deviation (10s)')
        ax.title.set_color('#141414')
        ax.title.set_fontsize(9)

    def _setup_status(self):
        ax = self.ax_status
        ax.axis('off')
        ax.set_facecolor('white')

    def _status_cb(self, msg: String):
        self.status_str = msg.data
        self.status_dict = _parse_status_line(self.status_str)
        self.status_ts = time.time()

    def _vis_cb(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.img_bgr = img
            self.img_ts = time.time()
        except Exception:
            pass

    def _scan_cb(self, msg: LaserScan):
        self.scan = msg
        self.scan_ts = time.time()

    def _init_anim(self):
        artists = []
        if self.im_artist is None:
            dummy = np.zeros((10, 10, 3), dtype=np.uint8)
            self.im_artist = self.ax_img.imshow(dummy)
        artists.append(self.im_artist)

        artists.append(self.scat_artist)
        artists.append(self.txt_f_artist)
        artists.append(self.txt_l_artist)
        artists.append(self.txt_r_artist)
        artists.append(self.txt_b_artist)
        artists.append(self.txt_lidar_state)

        artists.append(self.line_px)
        artists.append(self.line_cm)

        artists.append(self.txt_status)
        return artists

    def _update_anim(self, frame):
        now = time.time()
        dt_rel = now - self.t0

        # Update trends
        dev_px_f = _to_float(self.status_dict.get('dev', ''))
        dev_cm_f = _to_float(self.status_dict.get('devm', ''))

        if abs(now - self._last_anim_t) >= 0.05:  # ~20Hz max push, but we sample at anim rate
            pass

        self.t_deque.append(dt_rel)
        self.dev_px_deque.append(dev_px_f if dev_px_f >= -5000 else -999)
        self.dev_cm_deque.append(dev_cm_f if dev_cm_f >= -500 else -999)

        t_arr = np.array(self.t_deque) - dt_rel  # relative -10..0
        px_arr = np.array(self.dev_px_deque, dtype=float)
        cm_arr = np.array(self.dev_cm_deque, dtype=float)

        px_arr = np.where(px_arr < -500, np.nan, px_arr)
        cm_arr = np.where(cm_arr < -500, np.nan, cm_arr)

        self.line_px.set_data(t_arr, px_arr)
        self.line_cm.set_data(t_arr, cm_arr)

        # Auto-scale trend gently
        allv = []
        for a in (px_arr, cm_arr):
            v = a[~np.isnan(a)]
            if len(v):
                allv.extend(v.tolist())
        if allv:
            lo = min(allv); hi = max(allv)
            rng = max(1.0, hi - lo)
            self.ax_trend.set_ylim(lo - rng*0.15, hi + rng*0.15)
        else:
            self.ax_trend.set_ylim(-60, 60)
        self.ax_trend.set_xlim(-10.0, 0.0)

        # Image
        if self.img_bgr is not None:
            try:
                img_rgb = self.img_bgr[..., ::-1]  # BGR->RGB
                if self.im_artist is None:
                    self.im_artist = self.ax_img.imshow(img_rgb)
                else:
                    self.im_artist.set_array(img_rgb)
                self.ax_img.set_xlim(0, img_rgb.shape[1]-1)
                self.ax_img.set_ylim(img_rgb.shape[0]-1, 0)
                self.txt_lidar_state.set_text('')
            except Exception:
                pass
        else:
            self.txt_lidar_state.set_text('')

        # Scan
        lidar_ok = (self.status_dict.get('lidar', 'none') == 'ok') and (now - self.scan_ts < 1.0)
        if lidar_ok and self.scan is not None:
            try:
                msg = self.scan
                angles = np.arange(msg.angle_min, msg.angle_max + msg.angle_increment * 0.5,
                                   msg.angle_increment)
                ranges_m = np.array(msg.ranges)
                n = min(len(angles), len(ranges_m))
                angles = angles[:n]
                ranges_m = ranges_m[:n]
                ranges_cm = ranges_m * 100.0
                # Filter
                mask = np.isfinite(ranges_cm)
                mask &= (ranges_cm >= 5.0)
                mask &= (ranges_cm <= 160.0)
                th = angles[mask]
                rcm = ranges_cm[mask]
                # Color by range
                if len(rcm):
                    c = rcm
                else:
                    c = np.array([])
                self.scat_artist.set_offsets(np.column_stack((th, rcm)))
                self.scat_artist.set_array(c)
                self.txt_lidar_state.set_text('')
            except Exception:
                self.txt_lidar_state.set_text('LIDAR: NO DATA')
        else:
            self.scat_artist.set_offsets(np.empty((0, 2)))
            self.txt_lidar_state.set_text('LIDAR: NO DATA')

        # Min 4 dirs
        f_d = _to_float(self.status_dict.get('front', ''))
        self.txt_f_artist.set_text(f'F: {_fmt_cm(f_d)}')
        l_d = -1.0; r_d = -1.0; b_d = -1.0
        if lidar_ok and self.scan is not None:
            try:
                msg = self.scan
                angles = np.arange(msg.angle_min, msg.angle_max + msg.angle_increment * 0.5,
                                   msg.angle_increment)
                ranges_m = np.array(msg.ranges)
                n = min(len(angles), len(ranges_m))
                for i in range(n):
                    a = angles[i] * 180.0 / math.pi
                    rm = ranges_m[i]
                    if not math.isfinite(rm) or rm < 0.05 or rm > 2.0:
                        continue
                    rc = rm * 100.0
                    if -45 <= a < 45:
                        if r_d < 0 or rc < r_d:
                            r_d = rc
                    elif 45 <= a < 135:
                        if l_d < 0 or rc < l_d:
                            l_d = rc
                    elif 135 <= a < 225:
                        if b_d < 0 or rc < b_d:
                            b_d = rc
                    elif 225 <= a < 315:
                        if f_d < 0 or rc < f_d:
                            f_d_cand = rc
                            if f_d_cand < f_d or f_d < 0:
                                pass
                    # front better from status? keep status front as primary if present
            except Exception:
                pass
        # prefer status front if valid
        f_d_use = _to_float(self.status_dict.get('front', ''))
        self.txt_f_artist.set_text(f'F: {_fmt_cm(f_d_use)}')
        self.txt_l_artist.set_text(f'L: {_fmt_cm(l_d)}')
        self.txt_r_artist.set_text(f'R: {_fmt_cm(r_d)}')
        self.txt_b_artist.set_text(f'B: {_fmt_cm(b_d)}')

        # Status text
        sd = self.status_dict
        two_l = sd.get('two_lanes', '0')
        dev_px_s = sd.get('dev', '0')
        emg_s = sd.get('emg', '0')
        age_s = sd.get('age', '-1')
        proc_s = sd.get('proc', '0.0')
        fps_s = sd.get('fps', '0.0')
        lidar_s = sd.get('lidar', 'none')
        front_s = sd.get('front', '-1')
        serial_s = sd.get('serial', 'closed')
        w_s = sd.get('w', '-1')
        devm_s = sd.get('devm', '-1')
        # Du lieu moi: lenh dong co, van toc thuc te tu ESP32, tuoi goi RX, canh bao LiDAR
        spd_s = sd.get('spd', '0.0')
        kmh_s = sd.get('kmh', '-1')
        fbage_s = sd.get('fbage', '-1')
        alert_s = sd.get('alert', 'CLEAR')

        lane_state = 'TRACKING' if two_l == '1' else 'LOST'
        lane_col = 'green' if two_l == '1' else 'red'
        emg_state = 'ACTIVE' if emg_s == '1' else 'OFF'
        emg_col = 'red' if emg_s == '1' else 'green'
        lidar_state_s = 'OK' if lidar_s == 'ok' else 'NONE'
        lidar_col = 'green' if lidar_s == 'ok' else 'red'
        ser_state = 'OPEN' if serial_s == 'open' else 'CLOSED'
        ser_col = 'green' if serial_s == 'open' else 'red'

        status_lines = []
        status_lines.append(f"LANE: {lane_state}")
        status_lines.append(f"EMG:  {emg_state}")
        status_lines.append(f"Dev(px): {dev_px_s}")
        status_lines.append(f"Dev(cm): {devm_s}")
        status_lines.append(f"Width(cm): {w_s}")
        status_lines.append(f"Age: {age_s}")
        status_lines.append(f"Proc: {proc_s}")
        status_lines.append(f"Cam FPS: {fps_s}")
        status_lines.append(f"LIDAR: {lidar_state_s}")
        status_lines.append(f"Front: {front_s}")
        status_lines.append(f"Serial: {ser_state}")
        # Du lieu dong co + LiDAR (chi them hang moi, hang cu giu nguyen)
        spd_f = _to_float(spd_s)
        kmh_f = _to_float(kmh_s)
        fbage_f = _to_float(fbage_s)
        spd_txt = '---' if spd_f < 0 else f'{spd_f:.1f}'
        kmh_txt = '---' if kmh_f < 0 else f'{kmh_f:.2f}'
        rx_txt = '---' if fbage_f < 0 else f'{int(fbage_f)} ms'
        status_lines.append(f"Speed cmd: {spd_txt} km/h")
        status_lines.append(f"Velocity: {kmh_txt} km/h")
        status_lines.append(f"ESP32 RX: {rx_txt}")
        status_lines.append(f"Lidar alert: {alert_s}")
        status_lines.append(f"Uptime: {str(timedelta(seconds=int(now - self.t0)))}")

        txt = '\n'.join(status_lines)
        self.txt_status.set_text(txt)

        self._last_anim_t = now
        return (self.im_artist, self.scat_artist, self.txt_f_artist, self.txt_l_artist,
                self.txt_r_artist, self.txt_b_artist, self.txt_lidar_state,
                self.line_px, self.line_cm, self.txt_status)


def main(args=None):
    rclpy.init(args=args)
    node = DashboardMatplotlib()
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=2.0)


if __name__ == '__main__':
    main()
