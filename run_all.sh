#!/bin/bash

echo "==== CẤP QUYỀN TRUY CẬP PHẦN CỨNG ===="
sudo chmod 666 /dev/ttyUSB0 /dev/ttyUSB1 /dev/video*

echo "==== LOAD ROS 2 MÔI TRƯỜNG ===="
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "==== 1. KHỞI ĐỘNG LIDAR (ttyUSB1) ===="
ros2 run rplidar_ros rplidar_composition --ros-args \
  -p serial_port:=/dev/ttyUSB1 \
  -p serial_baudrate:=115200 \
  -p frame_id:=laser \
  -p inverted:=false \
  -p angle_compensate:=true &
PID_LIDAR=$!
sleep 2

echo "==== 2. KHỞI ĐỘNG CORE XE (ttyUSB0 + video0) ===="
ros2 launch autonomous_vehicle bringup.launch.py \
  serial_port:=/dev/ttyUSB0 \
  camera_index:=0 \
  enable_traffic:=false \
  enable_turn:=false &
PID_CORE=$!
sleep 2

echo "==== 3. KHỞI ĐỘNG GUI ===="
ros2 run autonomous_vehicle_gui autonomous_vehicle_gui_node &
PID_GUI=$!

# Bắt sự kiện Ctrl+C để dọn dẹp sạch sẽ các node chạy ngầm
trap "echo -e '\n[HỆ THỐNG] Đang tắt toàn bộ tiến trình...'; kill $PID_LIDAR $PID_CORE $PID_GUI; exit" INT

# Giữ script chạy để chờ Ctrl+C
wait
