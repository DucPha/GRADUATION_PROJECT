#!/bin/bash
# Docker entrypoint cho autonomous vehicle

set -e

# Source ROS 2
source /opt/ros/${ROS_DISTRO}/setup.bash
source /opt/ros/${ROS_DISTRO}/install/setup.bash 2>/dev/null || true

# Set default model directories if not provided via environment
# Điều này giúp node Python tìm model trong package share directory
export TRAFFIC_MODEL_DIR=${TRAFFIC_MODEL_DIR:-/opt/ros/${ROS_DISTRO}/install/traffic_light_detector/share/traffic_light_detector/models}
export TURN_MODEL_DIR=${TURN_MODEL_DIR:-/opt/ros/${ROS_DISTRO}/install/turn_detector/share/turn_detector/models}

# Quyền truy cập thiết bị.
# Container chạy với user KHÔNG phải root nên `sudo` không tồn tại (và cũng
# không cần): quyền được cấp lúc build bằng cách thêm user vào group
# `video`/`dialout` (xem Dockerfile). Cách cũ gọi sudo trong container là
# luôn thất bại rồi bị nuốt bằng `|| true`, nên quyền không bao giờ được đặt.
if [ -e /dev/video0 ] && [ ! -w /dev/video0 ]; then
    echo "[entrypoint] WARNING: /dev/video0 không mở được cho user hiện tại." >&2
    echo "[entrypoint]          Chạy container với --device=/dev/video0 và user thuộc group 'video'." >&2
fi

if [ -e /dev/ttyUSB0 ] && [ ! -w /dev/ttyUSB0 ]; then
    echo "[entrypoint] WARNING: /dev/ttyUSB0 không ghi được (cần group 'dialout')." >&2
fi

if [ -e /dev/ttyACM0 ] && [ ! -w /dev/ttyACM0 ]; then
    echo "[entrypoint] WARNING: /dev/ttyACM0 không ghi được (cần group 'dialout')." >&2
fi

exec "$@"