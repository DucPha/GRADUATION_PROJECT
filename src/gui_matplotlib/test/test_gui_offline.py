#!/usr/bin/env python3
"""
Offline test GUI (TkAgg, Windows) - MATLAB-style white theme, optimised for realtime.

Left   : camera, binary ROI, lane deviation trend
Centre : LIDAR polar map (12 m), fusion output
Right  : node panels (camera, LIDAR, ESP32) with status + result
ESC to exit.

Realtime design
  1. Data thread : each stream (camera / LIDAR / ESP32) overwrites a "latest value".
                   No queue -> the GUI can never fall behind the data.
  2. GUI timer   : reads the latest snapshot, redraws only what changed.
  3. Blitting    : two cached layers. Static layer = axes, grid, labels (drawn once).
                   Slow layer = text/chips (re-cached at TEXT_HZ or when a chip changes).
                   Per frame only the fast artists (images, lines, points, gauges) are drawn.
"""

import math
import threading
import time
import traceback
from collections import deque
from datetime import timedelta
from types import SimpleNamespace

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.colors import to_rgba
from matplotlib.gridspec import GridSpec
from matplotlib.patches import Rectangle
from matplotlib.transforms import Bbox
import numpy as np

# ------------------------------------------------------------------ theme (MATLAB defaults)
BG, CARD = '#F0F0F0', '#FFFFFF'
EDGE, GRID = '#262626', '#D9D9D9'
TEXT, MUTED = '#1A1A1A', '#6E6E6E'
BLUE, ORANGE, YELLOW = '#0072BD', '#D95319', '#EDB120'     # MATLAB colour order
GREEN, RED, AMBER = '#219947', '#A2142F', '#C98A00'        # status colours (AMBER = readable text)

R_MAX = 12.0         # LIDAR max range (m)
DANGER_M = 1.0       # red zone / STOP
WARN_M = 2.0         # amber zone / AVOID
STEER_MAX = 30       # steering gauge range (deg)
DEADBAND_CM = 1.5    # camera: |dev| below this = STRAIGHT
TREND_SEC = 10

ROI_Y0, ROI_H = 130, 100     # ROI rows in the camera image
IMG_H, IMG_W = 240, 320

TICK_MS = 25                         # GUI timer period (upper bound ~40 fps)
TEXT_HZ = 5                          # numeric text refresh rate
CAM_HZ, LIDAR_HZ, ESP_HZ = 30, 10, 20    # fake source rates
NICE = (5, 10, 20, 30, 50, 100, 200)     # trend axis limits (avoids constant rescaling)

plt.rcParams.update({
    'toolbar': 'None',
    'font.family': 'sans-serif',
    'font.sans-serif': ['Arial', 'Helvetica', 'Segoe UI', 'DejaVu Sans'],
    'axes.edgecolor': EDGE, 'axes.linewidth': 0.8,
    'axes.titlesize': 10, 'axes.titleweight': 'bold', 'axes.labelsize': 8.5,
    'xtick.direction': 'in', 'ytick.direction': 'in',
    'xtick.top': True, 'ytick.right': True,
    'xtick.labelsize': 8, 'ytick.labelsize': 8,
    'grid.color': EDGE, 'grid.alpha': 0.15, 'grid.linestyle': '-', 'grid.linewidth': 0.6,
    'legend.edgecolor': EDGE, 'legend.fancybox': False, 'legend.framealpha': 1.0,
})

DEG = np.arange(0, 360, 1)          # 0 = front, clockwise
TH = np.radians(DEG)
ROWS = np.arange(ROI_Y0, ROI_Y0 + ROI_H)
_rng = np.random.default_rng(0)


# ------------------------------------------------------------------ fake data
def fake_scan(w):
    """Corridor walls at +-4 m, one obstacle in front, one at +-60 deg. NaN = no return."""
    s = np.maximum(np.abs(np.sin(TH)), 1e-3)
    r = 4.0 / s
    for center, dist, half in ((0, w.front_m, 15), (300, w.obs_l, 8), (60, w.obs_r, 8)):
        ang = (DEG - center + 180) % 360 - 180
        m = np.abs(ang) <= half
        r[m] = np.minimum(r[m], dist / np.cos(np.radians(ang[m])))
    r = r + _rng.normal(0, 0.01, r.shape)
    r[r >= R_MAX] = np.nan
    return r


def lane_x(y, d):
    """Left/right lane x at image row y (fake perspective), d = deviation in px."""
    t = (IMG_H - 1 - y) / (IMG_H - 1 - 105)
    xl = (50 + d) + t * ((140 + 0.4 * d) - (50 + d))
    xr = (270 + d) + t * ((180 + 0.4 * d) - (270 + d))
    return xl, xr


def fake_binary(two_lanes, dev_px):
    b = np.zeros((ROI_H, IMG_W), np.uint8)
    b[_rng.random(b.shape) < 0.004] = 255                      # speckle noise
    if two_lanes:
        xl, xr = lane_x(ROWS, dev_px)
        idx = np.arange(ROI_H)
        for off in (-2, -1, 0, 1, 2):
            for x in (xl, xr):
                b[idx, np.clip((x + off).astype(int), 0, IMG_W - 1)] = 255
    return b


def create_fake_image():
    img = np.empty((IMG_H, IMG_W, 3), dtype=np.uint8)
    img[:105] = (206, 213, 222)      # sky
    img[105:] = (62, 66, 73)         # asphalt
    return img


class FakeSource(threading.Thread):
    """Stands in for the ROS subscribers.

    Each stream runs at its own rate and only overwrites the latest value.
    Real code: call put(...) from the subscriber callbacks, keep snapshot() as is.
    """

    def __init__(self):
        super().__init__(daemon=True)
        self.lock = threading.Lock()
        self.running = True
        self.t0 = time.time()
        self.world = SimpleNamespace(front_m=5.0, obs_l=6.0, obs_r=6.0)
        self.d = dict(
            t0=self.t0,
            cam=create_fake_image(), bin=np.zeros((ROI_H, IMG_W), np.uint8), cam_id=0,
            dev_px=0.0, dev_cm=0.0, two_lanes=1, w_cm=52.0,
            fps_cam=30.0, proc_ms=2.0, age_ms=10.0,
            scan=np.full(len(DEG), np.nan), scan_id=0, scan_hz=10.0, lidar_ok=1,
            serial_open=1, rx_age_ms=8.0, crc_err=0,
        )

    def snapshot(self):
        with self.lock:
            return dict(self.d)

    def put(self, **kw):
        with self.lock:
            self.d.update(kw)

    def _cam(self, t):
        dev_px = 12 * math.sin(0.8 * t)
        two = 0 if (t % 30) > 27 else 1                          # lane lost for a moment
        self.put(dev_px=dev_px, dev_cm=4.2 * math.sin(0.8 * t), two_lanes=two,
                 bin=fake_binary(two, dev_px), cam_id=self.d['cam_id'] + 1,
                 age_ms=10 + 5 * math.sin(t), proc_ms=2.0 + 0.3 * math.sin(1.2 * t),
                 fps_cam=29.5 + 0.8 * math.sin(0.7 * t))

    def _lidar(self, t):
        w = self.world
        w.front_m = 5.0 + 4.5 * math.sin(0.3 * t)                # 0.5 .. 9.5 m
        w.obs_l = 6.0 + 4.0 * math.sin(0.21 * t + 2)
        w.obs_r = 6.0 + 4.5 * math.sin(0.17 * t)
        self.put(scan=fake_scan(w), scan_id=self.d['scan_id'] + 1,
                 scan_hz=10.0 + 0.2 * math.sin(0.9 * t))

    def _esp(self, t):
        self.put(serial_open=0 if (t % 60) > 57 else 1,          # link drop for a moment
                 rx_age_ms=8.0 + 4.0 * math.sin(2 * t))

    def run(self):
        streams = [(self._cam, 1.0 / CAM_HZ), (self._lidar, 1.0 / LIDAR_HZ),
                   (self._esp, 1.0 / ESP_HZ)]
        nxt = [time.perf_counter()] * len(streams)
        while self.running:
            now = time.perf_counter()
            for i, (fn, period) in enumerate(streams):
                if now >= nxt[i]:
                    fn(time.time() - self.t0)
                    nxt[i] = max(nxt[i] + period, now - period)
            time.sleep(0.002)


# ------------------------------------------------------------------ decisions
def sector_min(r, center_deg):
    d = np.abs((DEG - center_deg + 180) % 360 - 180)
    v = r[(d <= 45) & ~np.isnan(r)]
    return float(v.min()) if v.size else R_MAX


def decide(fd, mins):
    F, L, R = mins['F'], mins['L'], mins['R']

    if not fd.two_lanes:
        cam = ('LANE LOST', RED)
    elif fd.dev_cm > DEADBAND_CM:
        cam = ('RIGHT', BLUE)
    elif fd.dev_cm < -DEADBAND_CM:
        cam = ('LEFT', BLUE)
    else:
        cam = ('STRAIGHT', GREEN)

    if F < DANGER_M:
        lid = ('STOP', RED)
    elif F < WARN_M:
        lid = ('AVOID LEFT', AMBER) if L > R else ('AVOID RIGHT', AMBER)
    else:
        lid = ('CLEAR', GREEN)

    if lid[0] == 'STOP':
        out, src, steer, speed = lid, 'LIDAR', 0.0, 0.0
    elif lid[0].startswith('AVOID'):
        out, src, speed = lid, 'LIDAR', 20.0
        steer = -20.0 if lid[0].endswith('LEFT') else 20.0
    elif not fd.two_lanes:
        out, src, steer, speed = cam, 'CAMERA', 0.0, 15.0
    else:
        out, src, speed = cam, 'CAMERA', 35.0
        steer = float(np.clip(2.0 * fd.dev_cm, -25, 25))

    if not fd.serial_open:
        esp = ('NO LINK', RED)
    elif speed == 0:
        esp = ('STOPPED', RED)
    elif steer < -1:
        esp = ('LEFT', BLUE)
    elif steer > 1:
        esp = ('RIGHT', BLUE)
    else:
        esp = ('STRAIGHT', GREEN)
    return dict(cam=cam, lid=lid, out=out, src=src, steer=steer, speed=speed, esp=esp)


# ------------------------------------------------------------------ widgets
class Blitter:
    """Two cached layers + a fast layer drawn every frame.

    bg0 = static figure, bg1 = bg0 + slow artists (text, chips).
    Z-order: static < slow < fast, so anything that must sit above an image
    (ROI box, overlay text) has to be a fast artist.
    """

    def __init__(self, fig, fast_axes):
        self.fig = fig
        self.canvas = fig.canvas
        self.fast_axes = fast_axes
        self.slow, self.fast = [], []
        self.bg0 = self.bg1 = self.fast_bbox = None
        self.canvas.mpl_connect('draw_event', self._on_draw)
        self.canvas.mpl_connect('resize_event', self._on_resize)

    def add_slow(self, *artists):
        for a in artists:
            a.set_animated(True)
            self.slow.append(a)

    def add_fast(self, *artists):               # add in z-order (back to front)
        for a in artists:
            a.set_animated(True)
            self.fast.append(a)

    def _draw(self, group):
        for a in group:
            self.fig.draw_artist(a)

    def _cache_slow(self):                      # canvas must hold bg0 right now
        self._draw(self.slow)
        self.bg1 = self.canvas.copy_from_bbox(self.fig.bbox)

    def _on_draw(self, _event):                 # after every full redraw (start, resize, limit change)
        self.bg0 = self.canvas.copy_from_bbox(self.fig.bbox)
        self._cache_slow()
        self._draw(self.fast)
        bb = Bbox.union([ax.bbox for ax in self.fast_axes])
        self.fast_bbox = Bbox.from_extents(bb.x0 - 2, bb.y0 - 2, bb.x1 + 2, bb.y1 + 2)

    def _on_resize(self, _event):
        self.bg0 = self.bg1 = None              # invalid until the next full draw

    def refresh(self, slow_changed):
        if self.bg0 is None:
            return
        if slow_changed:                        # rebuild slow layer, upload whole figure
            self.canvas.restore_region(self.bg0)
            self._cache_slow()
            region = self.fig.bbox
        else:                                   # normal frame: upload only the fast area
            self.canvas.restore_region(self.bg1)
            region = self.fast_bbox
        self._draw(self.fast)
        self.canvas.blit(region)


def chip():
    return dict(boxstyle='round,pad=0.3', fc=CARD, ec=EDGE, lw=0.9)


def set_chip(txt, text, color):
    key = (text, color)
    if getattr(txt, '_chip', None) == key:     # unchanged -> no re-layout
        return False
    txt._chip = key
    txt.set_text(text)
    txt.set_color(color)
    bb = txt.get_bbox_patch()
    bb.set_facecolor(to_rgba(color, 0.12))
    bb.set_edgecolor(color)
    return True


def make_card(ax, title, labels):
    """Node panel: title + status badge, key/value rows, RESULT chip."""
    ax.set_facecolor(CARD)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.set_autoscale_on(False)
    for s in ax.spines.values():
        s.set_edgecolor('#8C8C8C')
        s.set_linewidth(0.8)
    ax.text(0.04, 0.925, title, ha='left', va='center', fontsize=9.5,
            fontweight='bold', color=TEXT)
    badge = ax.text(0.96, 0.925, '', ha='right', va='center', fontsize=7.5,
                    fontweight='bold', bbox=chip())
    ax.plot([0.03, 0.97], [0.855, 0.855], color=GRID, lw=1)
    vals = []
    for y, lab in zip(np.linspace(0.78, 0.27, len(labels)), labels):
        ax.text(0.05, y, lab, ha='left', va='center', fontsize=8.5, color=MUTED)
        vals.append(ax.text(0.95, y, '', ha='right', va='center', fontsize=8.5,
                            fontweight='bold', color=TEXT))
    ax.plot([0.03, 0.97], [0.155, 0.155], color=GRID, lw=1)
    ax.text(0.05, 0.075, 'RESULT', ha='left', va='center', fontsize=8.5,
            fontweight='bold', color=TEXT)
    res = ax.text(0.95, 0.075, '', ha='right', va='center', fontsize=9,
                  fontweight='bold', bbox=chip())
    return badge, vals, res


def fit_lim(cur, peak, floor):
    """Trend axis limit with hysteresis: grows at once, shrinks only when far too big."""
    need = max(floor, 1.15 * peak)
    if need > cur or need < 0.4 * cur:
        return next((v for v in NICE if v >= need), NICE[-1])
    return cur


# ------------------------------------------------------------------ main
def main():
    src = FakeSource()
    fd = SimpleNamespace(**src.snapshot())      # latest snapshot, read by the row lambdas
    h, w = IMG_H, IMG_W

    fig = plt.figure(figsize=(15, 8.4), dpi=100, facecolor=BG)
    gs = GridSpec(3, 3, figure=fig, width_ratios=[0.95, 1.35, 0.85],
                  height_ratios=[1.3, 0.85, 0.95], hspace=0.40, wspace=0.20,
                  left=0.04, right=0.975, top=0.88, bottom=0.05)
    ax_img = fig.add_subplot(gs[0, 0])
    ax_bin = fig.add_subplot(gs[1, 0])
    ax_trend = fig.add_subplot(gs[2, 0])
    ax_polar = fig.add_subplot(gs[0:2, 1], projection='polar')
    ax_fus = fig.add_subplot(gs[2, 1])
    gs_c = gs[:, 2].subgridspec(3, 1, hspace=0.12, height_ratios=[1.15, 1.0, 1.0])
    ax_ncam = fig.add_subplot(gs_c[0])
    ax_nlid = fig.add_subplot(gs_c[1])
    ax_nesp = fig.add_subplot(gs_c[2])
    ax_cm = ax_trend.twinx()
    for ax in (ax_img, ax_bin, ax_polar, ax_trend):
        ax.set_facecolor(CARD)

    # ---------------- header
    fig.text(0.04, 0.945, 'AUTOCAR FUSION MONITOR', fontsize=16, fontweight='bold',
             color=TEXT, va='center')
    txt_sub = fig.text(0.04, 0.91, '', fontsize=9, color=MUTED, va='center')
    txt_overall = fig.text(0.975, 0.93, 'NORMAL', fontsize=12, fontweight='bold',
                           ha='right', va='center', bbox=chip())

    # ---------------- camera (image + overlay are all animated, drawn back to front)
    im_cam = ax_img.imshow(fd.cam, interpolation='nearest')
    ax_img.set_autoscale_on(False)
    ax_img.set_xticks([])
    ax_img.set_yticks([])
    ax_img.set_title('CAMERA', pad=6)
    roi_rect = Rectangle((0, ROI_Y0), w, ROI_H, fill=False, ec=YELLOW, lw=1.3, ls='--')
    ax_img.add_patch(roi_rect)
    roi_lbl = ax_img.text(5, ROI_Y0 - 3, 'ROI', color=YELLOW, fontsize=7.5,
                          fontweight='bold', va='bottom')
    car_center, = ax_img.plot([w / 2, w / 2], [h - 1, 105], color='#9AA5B1', lw=1.2, ls=':')
    ln_left, = ax_img.plot([], [], color='white', lw=2.4)
    ln_right, = ax_img.plot([], [], color='white', lw=2.4)
    ln_mid, = ax_img.plot([], [], color=GREEN, lw=1.8, ls='--')
    txt_cam = ax_img.text(w - 6, 8, '', ha='right', va='top', fontsize=8.5, fontweight='bold',
                          bbox=dict(boxstyle='round,pad=0.3', fc='white', ec=EDGE, lw=0.6, alpha=0.9))
    ax_img.set_xlim(0, w)
    ax_img.set_ylim(h, 0)

    # ---------------- binary ROI
    im_bin = ax_bin.imshow(fd.bin, cmap='gray', vmin=0, vmax=255, interpolation='nearest')
    ax_bin.set_xticks([])
    ax_bin.set_yticks([])
    ax_bin.set_title(f'ROI BINARY  ({w} x {ROI_H})', pad=6)

    # ---------------- LIDAR polar (0 deg = front, clockwise)
    ax_polar.set_theta_zero_location('N')
    ax_polar.set_theta_direction(-1)
    full = np.linspace(0, 2 * np.pi, 361)
    ax_polar.fill_between(full, 0, DANGER_M, color=RED, alpha=0.18, lw=0)
    ax_polar.fill_between(full, DANGER_M, WARN_M, color=YELLOW, alpha=0.20, lw=0)
    for ang in (45, 135, 225, 315):                                     # sector boundaries
        ax_polar.plot([math.radians(ang)] * 2, [0, R_MAX], color='#9A9A9A', lw=0.7, ls='--')
    # points by zone (red < DANGER, yellow < WARN, blue beyond): marker lines draw much
    # faster than a scatter and cache their polar transform between scans
    pt_lines = [ax_polar.plot([], [], ls='none', marker='o', ms=3, mew=0, color=c, zorder=5)[0]
                for c in (RED, YELLOW, BLUE)]
    ax_polar.plot([0], [0], ls='none', marker='^', ms=9, color=GREEN, zorder=10)   # vehicle
    ax_polar.set_ylim(0, R_MAX)
    ax_polar.set_rticks([2, 4, 6, 8, 10, 12])
    ax_polar.set_yticklabels(['2', '4', '6', '8', '10', '12 m'])
    ax_polar.set_rlabel_position(15)
    ax_polar.tick_params(axis='y', labelsize=7.5, labelcolor=MUTED)
    ax_polar.set_xticks(np.radians(np.arange(0, 360, 30)))
    ax_polar.set_xticklabels(['', '30°', '60°', '', '120°', '150°', '',
                              '210°', '240°', '', '300°', '330°'])
    ax_polar.tick_params(axis='x', labelsize=7.5, labelcolor=MUTED, pad=2)
    ax_polar.grid(True)
    ax_polar.spines['polar'].set_edgecolor(EDGE)
    ax_polar.set_title('LIDAR MAP  (max range 12 m)', pad=46)

    SECTORS = {'F': (0, 'FRONT', 0.5, 1.10), 'R': (90, 'RIGHT', 1.12, 0.5),
               'B': (180, 'REAR', 0.5, -0.10), 'L': (270, 'LEFT', -0.12, 0.5)}
    sector_txt = {}
    for k, (_, name, x, y) in SECTORS.items():
        sector_txt[k] = ax_polar.text(x, y, f'{name}\n---', transform=ax_polar.transAxes,
                                      ha='center', va='center', fontsize=9,
                                      fontweight='bold', color=MUTED, linespacing=1.3)

    # ---------------- trend (px left axis, cm right axis)
    lim = {'px': 10.0, 'cm': 5.0}
    ax_trend.set_title('LANE DEVIATION', loc='left', pad=6)
    ax_trend.set_xlim(-TREND_SEC, 0)
    ax_trend.set_ylim(-lim['px'], lim['px'])
    ax_cm.set_ylim(-lim['cm'], lim['cm'])
    ax_trend.grid(True)
    ax_trend.axhline(0, color=MUTED, lw=0.8)
    ax_trend.set_xlabel(f'Time (s)   [last {TREND_SEC} s]')
    ax_trend.set_ylabel('Deviation (px)', color=BLUE)
    ax_cm.set_ylabel('Deviation (cm)', color=ORANGE)
    ax_trend.tick_params(axis='y', labelcolor=BLUE)
    ax_cm.tick_params(axis='y', labelcolor=ORANGE)
    line_px, = ax_trend.plot([], [], color=BLUE, lw=1.5, label='dev px')
    line_cm, = ax_cm.plot([], [], color=ORANGE, lw=1.5, label='dev cm')
    ax_trend.legend(handles=[line_px, line_cm], loc='lower right', bbox_to_anchor=(1.0, 1.0),
                    fontsize=7.5, ncol=2, borderaxespad=0.3)           # static, above the axes
    t_dq = deque(maxlen=TREND_SEC * CAM_HZ + 30)
    px_dq = deque(maxlen=t_dq.maxlen)
    cm_dq = deque(maxlen=t_dq.maxlen)

    # ---------------- fusion output
    ax_fus.set_facecolor(CARD)
    ax_fus.set_xticks([])
    ax_fus.set_yticks([])
    ax_fus.set_xlim(0, 1)
    ax_fus.set_ylim(0, 1)
    ax_fus.set_autoscale_on(False)
    for s in ax_fus.spines.values():
        s.set_edgecolor('#8C8C8C')
        s.set_linewidth(0.8)
    ax_fus.text(0.03, 0.90, 'FUSION OUTPUT', ha='left', va='center', fontsize=9.5,
                fontweight='bold', color=TEXT)
    txt_out = ax_fus.text(0.19, 0.50, '', ha='center', va='center', fontsize=16,
                          fontweight='bold', bbox=dict(boxstyle='round,pad=0.5', fc=CARD, ec=EDGE, lw=1.2))
    txt_src = ax_fus.text(0.19, 0.14, '', ha='center', va='center', fontsize=8.5, color=MUTED)
    ax_fus.plot([0.38, 0.38], [0.08, 0.92], color=GRID, lw=1)

    GX, GW = 0.43, 0.54
    ax_fus.text(GX, 0.77, 'Steering (deg)', ha='left', va='center', fontsize=8.5, color=MUTED)
    txt_steer = ax_fus.text(GX + GW, 0.77, '', ha='right', va='center', fontsize=8.5,
                            fontweight='bold', color=TEXT)
    ax_fus.add_patch(Rectangle((GX, 0.53), GW, 0.14, fc='#E9E9E9', ec=EDGE, lw=0.6))
    steer_fill = Rectangle((GX + GW / 2, 0.53), 0, 0.14, fc=BLUE, ec='none')
    ax_fus.add_patch(steer_fill)
    steer_zero, = ax_fus.plot([GX + GW / 2] * 2, [0.50, 0.70], color=EDGE, lw=1)
    ax_fus.text(GX, 0.44, f'LEFT {STEER_MAX}', ha='left', va='center', fontsize=7, color=MUTED)
    ax_fus.text(GX + GW / 2, 0.44, '0', ha='center', va='center', fontsize=7, color=MUTED)
    ax_fus.text(GX + GW, 0.44, f'RIGHT {STEER_MAX}', ha='right', va='center', fontsize=7, color=MUTED)

    ax_fus.text(GX, 0.31, 'Speed (%)', ha='left', va='center', fontsize=8.5, color=MUTED)
    txt_speed = ax_fus.text(GX + GW, 0.31, '', ha='right', va='center', fontsize=8.5,
                            fontweight='bold', color=TEXT)
    ax_fus.add_patch(Rectangle((GX, 0.13), GW, 0.14, fc='#E9E9E9', ec=EDGE, lw=0.6))
    speed_fill = Rectangle((GX, 0.13), 0, 0.14, fc=GREEN, ec='none')
    ax_fus.add_patch(speed_fill)
    ax_fus.text(GX, 0.06, '0', ha='left', va='center', fontsize=7, color=MUTED)
    ax_fus.text(GX + GW, 0.06, '100', ha='right', va='center', fontsize=7, color=MUTED)

    # ---------------- node panels
    st = dict(mins={k: R_MAX for k in SECTORS}, valid=0.0, cam_id=-1, scan_id=-1,
              last_txt=0.0)
    st['res'] = decide(fd, st['mins'])
    cam_rows = [
        ('Resolution',   lambda: f'{w} x {h}'),
        ('Frame rate',   lambda: f'{fd.fps_cam:.1f} fps'),
        ('Frame age',    lambda: f'{fd.age_ms:.0f} ms'),
        ('Process time', lambda: f'{fd.proc_ms:.1f} ms'),
        ('Lanes found',  lambda: f'{2 if fd.two_lanes else 0} / 2'),
        ('Lane width',   lambda: f'{fd.w_cm:.0f} cm' if fd.two_lanes else '--'),
        ('Deviation',    lambda: f'{fd.dev_cm:+.1f} cm' if fd.two_lanes else '--'),
    ]
    lid_rows = [
        ('Scan rate',        lambda: f'{fd.scan_hz:.1f} Hz'),
        ('Points / scan',    lambda: f'{len(DEG)}'),
        ('Valid points',     lambda: f'{st["valid"]:.0f} %'),
        ('Max range',        lambda: f'{R_MAX:.1f} m'),
        ('Front min',        lambda: f'{st["mins"]["F"]:.2f} m'),
        ('Left / Right min', lambda: f'{st["mins"]["L"]:.2f} / {st["mins"]["R"]:.2f} m'),
    ]
    esp_rows = [
        ('Port / baud', lambda: 'COM3 / 115200'),
        ('TX rate',     lambda: '20 Hz' if fd.serial_open else '--'),
        ('RX rate',     lambda: '20 Hz' if fd.serial_open else '--'),
        ('Last RX',     lambda: f'{fd.rx_age_ms:.0f} ms' if fd.serial_open else '--'),
        ('Steer cmd',   lambda: f'{st["res"]["steer"]:+.1f} deg'),
        ('Speed cmd',   lambda: f'{st["res"]["speed"]:.0f} %'),
        ('CRC errors',  lambda: f'{fd.crc_err}'),
    ]
    cam_badge, cam_vals, cam_res = make_card(ax_ncam, 'CAMERA NODE', [r[0] for r in cam_rows])
    lid_badge, lid_vals, lid_res = make_card(ax_nlid, 'LIDAR NODE', [r[0] for r in lid_rows])
    esp_badge, esp_vals, esp_res = make_card(ax_nesp, 'ESP32 NODE', [r[0] for r in esp_rows])

    # ---------------- blitting: everything that moves, in z-order
    blit = Blitter(fig, [ax_img, ax_bin, ax_polar, ax_trend, ax_fus])
    blit.add_slow(*sector_txt.values(),
                  txt_out, txt_src, txt_steer, txt_speed,
                  cam_badge, cam_res, lid_badge, lid_res, esp_badge, esp_res,
                  *cam_vals, *lid_vals, *esp_vals,
                  txt_sub, txt_overall)
    blit.add_fast(im_cam, roi_rect, roi_lbl, car_center, ln_left, ln_right, ln_mid, txt_cam,
                  im_bin,
                  *pt_lines,
                  line_px, line_cm,
                  steer_fill, steer_zero, speed_fill)

    def fill(vals, rows):
        for t, (_, fn) in zip(vals, rows):
            t.set_text(fn())

    def update_numbers(now):
        """Numeric text - runs at TEXT_HZ only."""
        mins, res = st['mins'], st['res']
        for k, (_, name, _, _) in SECTORS.items():
            sector_txt[k].set_text(f'{name}\n{mins[k]:.2f} m')
            sector_txt[k].set_color(RED if mins[k] < DANGER_M else
                                    AMBER if mins[k] < WARN_M else GREEN)
        fill(cam_vals, cam_rows)
        fill(lid_vals, lid_rows)
        fill(esp_vals, esp_rows)
        txt_steer.set_text(f'{res["steer"]:+.1f}')
        txt_speed.set_text(f'{res["speed"]:.0f}')
        if fd.two_lanes:
            txt_cam.set_text(f'dev {fd.dev_cm:+.1f} cm')
            txt_cam.set_color(GREEN)
        else:
            txt_cam.set_text('LANE LOST')
            txt_cam.set_color(RED)
        txt_sub.set_text(f'Offline simulation   |   Uptime {timedelta(seconds=int(now - fd.t0))}'
                         f'   |   ESC to exit')
        # trend axis limits: rare change -> full redraw
        if t_dq:
            new_px = fit_lim(lim['px'], np.abs(px_dq).max(), 10.0)
            new_cm = fit_lim(lim['cm'], np.abs(cm_dq).max(), 5.0)
            if (new_px, new_cm) != (lim['px'], lim['cm']):
                lim.update(px=new_px, cm=new_cm)
                ax_trend.set_ylim(-new_px, new_px)
                ax_cm.set_ylim(-new_cm, new_cm)
                fig.canvas.draw_idle()

    def _tick():
        now = time.time()
        snap = src.snapshot()
        fd.__dict__.update(snap)
        dirty = slow = False

        if snap['scan_id'] != st['scan_id']:                 # new LIDAR scan
            st['scan_id'] = snap['scan_id']
            r = snap['scan']
            ok = ~np.isnan(r)
            th_ok, r_ok = TH[ok], r[ok]
            zone = np.digitize(r_ok, (DANGER_M, WARN_M))          # 0 red, 1 yellow, 2 blue
            for z, ln in enumerate(pt_lines):
                m = zone == z
                ln.set_data(th_ok[m], r_ok[m])
            st['mins'] = {k: sector_min(r, c[0]) for k, c in SECTORS.items()}
            st['valid'] = 100.0 * float(ok.mean())
            dirty = True

        if snap['cam_id'] != st['cam_id']:                   # new camera frame
            st['cam_id'] = snap['cam_id']
            im_cam.set_data(snap['cam'])                     # real camera: put the decoded frame here
            im_bin.set_data(snap['bin'])
            if fd.two_lanes:
                xl0, xr0 = lane_x(h - 1, fd.dev_px)
                xl1, xr1 = lane_x(105, fd.dev_px)
                ln_left.set_data([xl0, xl1], [h - 1, 105])
                ln_right.set_data([xr0, xr1], [h - 1, 105])
                ln_mid.set_data([(xl0 + xr0) / 2, (xl1 + xr1) / 2], [h - 1, 105])
            else:
                for ln in (ln_left, ln_right, ln_mid):
                    ln.set_data([], [])
            t_dq.append(now - fd.t0)
            px_dq.append(fd.dev_px)
            cm_dq.append(fd.dev_cm)
            t_arr = np.array(t_dq) - (now - fd.t0)
            line_px.set_data(t_arr, np.array(px_dq))
            line_cm.set_data(t_arr, np.array(cm_dq))
            dirty = True

        txt_due = now - st['last_txt'] >= 1.0 / TEXT_HZ
        if dirty or txt_due:
            res = decide(fd, st['mins'])
            st['res'] = res

            half = GW / 2                                    # gauges
            v = float(np.clip(res['steer'], -STEER_MAX, STEER_MAX)) / STEER_MAX
            steer_fill.set_x(GX + half if v >= 0 else GX + half + v * half)
            steer_fill.set_width(abs(v) * half)
            speed_fill.set_width(GW * res['speed'] / 100.0)

            src_text = f'Source: {res["src"]}'                # chips: slow layer, rebuilt only on change
            if txt_src.get_text() != src_text:
                txt_src.set_text(src_text)
                slow = True
            slow |= set_chip(txt_out, *res['out'])
            slow |= set_chip(cam_badge, *(('ONLINE', GREEN) if fd.fps_cam > 0 else ('OFFLINE', RED)))
            slow |= set_chip(lid_badge, *(('ONLINE', GREEN) if fd.lidar_ok else ('NO DATA', RED)))
            slow |= set_chip(esp_badge, *(('CONNECTED', GREEN) if fd.serial_open
                                          else ('DISCONNECTED', RED)))
            slow |= set_chip(cam_res, *res['cam'])
            slow |= set_chip(lid_res, *res['lid'])
            slow |= set_chip(esp_res, *res['esp'])
            if res['out'][0] == 'STOP':
                lvl = ('EMERGENCY', RED)
            elif (res['out'][0] != res['cam'][0] or res['cam'][0] == 'LANE LOST'
                  or not fd.serial_open or not fd.lidar_ok):
                lvl = ('WARNING', AMBER)
            else:
                lvl = ('NORMAL', GREEN)
            slow |= set_chip(txt_overall, *lvl)
            dirty = True

        if txt_due:
            st['last_txt'] = now
            update_numbers(now)
            slow = True

        if dirty or slow:
            blit.refresh(slow)

    def tick():
        try:
            _tick()
        except Exception:                # a Tk timer dies silently on exceptions
            traceback.print_exc()

    fig.canvas.mpl_connect('key_press_event',
                           lambda e: plt.close(fig) if e.key == 'escape' else None)
    fig.canvas.mpl_connect('close_event', lambda e: setattr(src, 'running', False))
    fig.canvas.manager.set_window_title('Autocar Fusion Monitor - OFFLINE TEST')

    src.start()
    timer = fig.canvas.new_timer(interval=TICK_MS)
    timer.add_callback(tick)
    timer.start()
    plt.show()


if __name__ == '__main__':
    main()