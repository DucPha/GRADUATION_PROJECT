#!/bin/bash
# Chay toan bo he thong. Moi cong serial va camera deu tu do, nen khong can
# truyen tham so. Lan sau doi thi them tham so, vi du:
#   ./run_all.sh speed_x10:=30
set -e

echo "==== CAP QUYEN TRUY CAP PHAN CUNG ===="
sudo chmod 666 /dev/ttyACM* /dev/ttyUSB* /dev/video* 2>/dev/null || true

echo "==== LOAD ROS 2 MOI TRUONG ===="
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "==== KHOI DONG XE (camera 2 lan + ESP32 + LiDAR) ===="
ros2 launch fusion_node fusion.launch.py "$@"