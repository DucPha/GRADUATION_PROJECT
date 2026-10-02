#!/usr/bin/env python3
"""
Check USB camera supported formats and FPS.
Run: python3 tools/check_camera_formats.py [device_index]
"""
import subprocess
import sys

def run_v4l2_ctl(device):
    """Run v4l2-ctl --list-formats-ext and parse output."""
    try:
        result = subprocess.run(
            ['v4l2-ctl', '-d', device, '--list-formats-ext'],
            capture_output=True, text=True, timeout=5
        )
        return result.stdout
    except FileNotFoundError:
        return "ERROR: v4l2-ctl not installed. Run: sudo apt install v4l-utils"
    except subprocess.TimeoutExpired:
        return "ERROR: v4l2-ctl timeout"

def parse_formats(output):
    """Parse v4l2-ctl output for human-readable summary."""
    lines = output.split('\n')
    current_fmt = None
    results = []

    for line in lines:
        line = line.strip()
        if line.startswith('[') and ']:' in line:
            # Format line: [0]: 'MJPG' (Motion-JPEG, compressed)
            current_fmt = line
            results.append({'format': current_fmt, 'resolutions': []})
        elif current_fmt and 'Size:' in line and 'fps' in line.lower():
            # Resolution line: Size: Discrete 640x480
            #                  Interval: Discrete 0.033s (30.000 fps)
            results[-1]['resolutions'].append(line)

    return results

def main():
    device_idx = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    device = f'/dev/video{device_idx}'

    print(f"Checking camera formats for {device}...\n")
    output = run_v4l2_ctl(device)
    print(output)
    print("\n" + "="*60)
    print("SUMMARY - Look for MJPG @ 640x480 @ 30/60fps:")
    print("="*60)

    parsed = parse_formats(output)
    for fmt in parsed:
        print(f"\n{fmt['format']}")
        for res in fmt['resolutions']:
            if '640x480' in res or '30.000' in res or '60.000' in res or '120.000' in res:
                print(f"  >>> {res}  <-- GOOD FOR LANE KEEPING")
            else:
                print(f"      {res}")

if __name__ == '__main__':
    main()