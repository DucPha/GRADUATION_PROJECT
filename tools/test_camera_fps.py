#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Measure real capture FPS of the USB camera across resolutions.

Bypass the ROS node so we isolate the camera from the image pipeline.
Usage: python3 tools/test_camera_fps.py [index] [seconds]
"""
import glob
import subprocess
import sys
import time

import cv2

RESOLUTIONS = [(352, 288), (320, 240), (176, 144), (640, 480)]


def probe(idx):
    """Return True if idx is a UVC node that yields real frames."""
    try:
        r = subprocess.run(
            ["v4l2-ctl", "-d", f"/dev/video{idx}", "--all"],
            capture_output=True, text=True, timeout=5,
        ).stdout
        if "Video Capture" not in r or "uvcvideo" not in r:
            return False
        cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            return False
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 352)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 288)
        ok, frame = cap.read()
        cap.release()
        return bool(ok and frame is not None and not frame.empty())
    except Exception:
        return False


def detect_camera(retries=10, delay=1.0):
    """Find a /dev/videoN node that is a real UVC capture device.

    The index is NOT stable: this camera drops off the USB bus repeatedly and
    the kernel hands out a different number each time (Device 013 -> 016, then
    video0 <-> video1). Probing is the only reliable way, and it has to be
    retried because a re-enumeration in flight leaves every node unreadable
    for a moment.
    """
    for attempt in range(retries):
        for node in sorted(glob.glob("/dev/video*")):
            try:
                idx = int(node[-1])
            except ValueError:
                continue
            if probe(idx):
                if attempt:
                    print(f"camera found on retry {attempt + 1}")
                return idx
        print(f"no camera on attempt {attempt + 1}/{retries}, waiting...")
        time.sleep(delay)
    return None


def measure(idx, w, h, seconds, fps_req=30):
    cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
    if not cap.isOpened():
        return None
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS, fps_req)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    gw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    gh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if gw < 32 or gh < 32:
        cap.release()
        return None

    for _ in range(10):          # warm up: auto-exposure needs a few frames
        cap.read()

    n = 0
    t0 = time.time()
    while time.time() - t0 < seconds:
        if cap.read()[0]:
            n += 1
    el = time.time() - t0
    cap.release()
    return gw, gh, n / el


def main():
    idx = int(sys.argv[1]) if len(sys.argv) > 1 else -1
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0

    if idx < 0:
        idx = detect_camera()
        if idx is None:
            print("ERROR: no usable UVC capture node found")
            return 1
        print(f"auto-detected camera at /dev/video{idx}")
    else:
        print(f"using /dev/video{idx}")

    print(f"{'requested':>12} {'actual':>12} {'fps':>8}   note")
    for w, h in RESOLUTIONS:
        res = measure(idx, w, h, seconds)
        if res is None:
            print(f"{w}x{h:<7} {'FAIL':>12} {'-':>8}   node lost / format rejected")
            # Camera may have fallen off the bus mid-test; re-detect once.
            idx2 = detect_camera()
            if idx2 is None:
                print("   camera gone from USB bus - stop testing")
                break
            print(f"   re-detected at /dev/video{idx2}, retrying")
            idx = idx2
            continue

        gw, gh, fps = res
        note = "OK" if fps >= 25 else ("USABLE" if fps >= 15 else "TOO SLOW")
        print(f"{w}x{h:<7} {gw}x{gh:<7} {fps:>8.2f}   {note}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
