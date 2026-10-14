#!/usr/bin/env python3
# ============================================================================
# calibrate_pitch.py - DO GOC CUI CAMERA (chay moi lan chinh camera)
# ----------------------------------------------------------------------------
# Detector chieu anh xuong mat san theo goc cui camera. Goc sai vai do ->
# khoang cach / be rong lan sai -> vach thu 2 bi loai (chay 1 vach), xe vao
# cua som / muon. Tu hieu chinh khi dang chay KHONG on dinh tren duong gap
# khuc (13/10: troi 4-6 do), nen do 1 lan bang cong cu nay:
#
#   1. Tat ./run.sh (camera dang ban). Dat xe tren BAN LAB, mui xe huong doc
#      theo 2 canh dai cua ban (2 canh THANG, SONG SONG that). KHONG dung vach
#      duong: bang cao su dan tay khong song song (13/10: 1 doan duong thang
#      cho 52 do thay vi ~35) - cung la ly do tu hieu chinh tren duong bi troi.
#   2. python3 tools/calibrate_pitch.py           (chup tu camera)
#      python3 tools/calibrate_pitch.py anh.jpg   (dung anh co san)
#   3. Kiem tra anh tools/calibrate_pitch_check.jpg (2 duong do phai nam dung
#      tren 2 canh). Goc duoc luu vao ~/.autocar/camera_pitch_deg; lan chay
#      ./run.sh sau tu dung (hoac ghi de: ./run.sh camera_pitch_deg:=34).
#
# Cach tinh: 2 duong song song tren san hoi tu tai diem tu (u, v) tren anh.
# Camera cui goc p, tieu cu f, tam anh cy: tan(p) = (cy - v) / f.
# f = (H/2) / tan(vfov/2), vfov = 51 do (do 08/10, giong camera_vfov_deg).
# ============================================================================

import math
import os
import sys

import cv2
import numpy as np

VFOV_DEG = 51.0
OUT_DIR = os.path.expanduser("~/.autocar")
OUT_FILE = os.path.join(OUT_DIR, "camera_pitch_deg")
CHECK_JPG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibrate_pitch_check.jpg")


def grab_frame():
    """Chup 1 anh 1920x1080 MJPG tu camera (bo vai frame dau cho on dinh phoi sang)."""
    for idx in range(4):
        cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if not cap.isOpened():
            continue
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        frame = None
        for _ in range(20):
            ok, f = cap.read()
            if ok:
                frame = f
        cap.release()
        if frame is not None:
            return frame
    sys.exit("Khong mo duoc camera (./run.sh con dang chay?)")


def find_edges(img):
    """Cac doan thang dai trong anh: (do dai, x1, y1, x2, y2, he so goc dx/dy)."""
    g = cv2.GaussianBlur(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    e = cv2.Canny(g, 30, 90)
    h = img.shape[0]
    lines = cv2.HoughLinesP(e, 1, np.pi / 360, 120, minLineLength=h // 5, maxLineGap=25)
    out = []
    for x1, y1, x2, y2 in (lines[:, 0] if lines is not None else []):
        ang = abs(math.degrees(math.atan2(y2 - y1, x2 - x1)))
        if 20 < ang < 88 or 92 < ang < 160:  # bo duong ngang / dung
            out.append((math.hypot(x2 - x1, y2 - y1), x1, y1, x2, y2, (x2 - x1) / (y2 - y1)))
    return sorted(out, reverse=True)


def vanishing_point(edges, w, h):
    """Cap canh dai nhat co 2 he so goc trai dau, giao nhau TREN anh, quanh giua."""
    best = None
    for i in range(min(len(edges), 12)):
        for j in range(i + 1, min(len(edges), 12)):
            a, b = edges[i], edges[j]
            if a[5] * b[5] >= 0:
                continue  # cung chieu nghieng: khong phai 2 canh 2 ben
            # x = x1 + k * (y - y1) cho moi duong -> giao diem
            ka, kb = a[5], b[5]
            v = (b[1] - a[1] + ka * a[2] - kb * b[2]) / (ka - kb)
            u = a[1] + ka * (v - a[2])
            if v >= 0 or not (-w < u < 2 * w):
                continue
            score = a[0] + b[0]
            if best is None or score > best[0]:
                best = (score, u, v, a, b)
    return best


def main():
    img = cv2.imread(sys.argv[1]) if len(sys.argv) > 1 else grab_frame()
    if img is None:
        sys.exit("Khong doc duoc anh")
    h, w = img.shape[:2]
    best = vanishing_point(find_edges(img), w, h)
    if best is None:
        sys.exit("Khong tim thay 2 canh song song hoi tu phia truoc - dat xe thang huong 2 canh / 2 vach")
    _, u, v, a, b = best
    f = (h / 2.0) / math.tan(math.radians(VFOV_DEG / 2.0))
    pitch = math.degrees(math.atan((h / 2.0 - v) / f))

    vis = img.copy()
    for e in (a, b):
        x1, y1, k = e[1], e[2], e[5]
        cv2.line(vis, (int(x1 + k * (h - y1)), h), (int(x1 + k * (0 - y1)), 0), (0, 0, 255), 3)
    cv2.putText(vis, "pitch %.1f deg" % pitch, (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 255), 4)
    cv2.imwrite(CHECK_JPG, cv2.resize(vis, (w // 2, h // 2)))

    if not 15.0 <= pitch <= 60.0:
        sys.exit("Goc do duoc %.1f do khong hop ly - kiem tra %s" % (pitch, CHECK_JPG))
    os.makedirs(OUT_DIR, exist_ok=True)
    with open(OUT_FILE, "w") as fh:
        fh.write("%.1f\n" % pitch)
    print("Goc cui camera: %.1f do (diem tu v = %.0f px). Da luu %s" % (pitch, v, OUT_FILE))
    print("Kiem tra anh: %s" % CHECK_JPG)


if __name__ == "__main__":
    main()
