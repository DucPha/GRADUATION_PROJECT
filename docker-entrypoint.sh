#!/bin/bash
# Docker entrypoint for autonomous vehicle

set -e

# Source ROS 2
source /opt/ros/${ROS_DISTRO}/setup.bash
source /opt/ros/${ROS_DISTRO}/install/setup.bash 2>/dev/null || true

# Setup hardware access
if [ -e /dev/video0 ]; then
    sudo chmod 666 /dev/video* 2>/dev/null || true
fi

if [ -e /dev/ttyUSB0 ]; then
    sudo chmod 666 /dev/ttyUSB* 2>/dev/null || true
fi

if [ -e /dev/ttyACM0 ]; then
    sudo chmod 666 /dev/ttyACM* 2>/dev/null || true
fi

# Execute command
exec "$@"