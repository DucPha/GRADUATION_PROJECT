#!/bin/bash
# Build + chay toan bo he thong xe tu hanh.
#
#   ./run.sh                      # build neu can, roi chay
#   ./run.sh --no-build           # chi chay, khong build lai
#   ./run.sh --gui                # them dashboard matplotlib
#   ./run.sh speed_x10:=30        # goc tham so launch truyen nguyen
set -e

cd "$(dirname "$0")"

BUILD=1
GUI=0
declare -a ARGS=()

for arg in "$@"; do
	case "$arg" in
		--no-build) BUILD=0 ;;
		--gui)      GUI=1 ;;
		*)          ARGS+=("$arg") ;;
	esac
done

echo "==== CAP QUYEN TRUY CAP PHAN CUNG ===="
sudo chmod 666 /dev/ttyACM* /dev/ttyUSB* /dev/video* 2>/dev/null || true

echo "==== LOAD ROS 2 MOI TRUONG ===="
# shellcheck disable=SC1091
source /opt/ros/"${ROS_DISTRO:-jazzy}"/setup.bash

if [ "$BUILD" -eq 1 ]; then
	echo "==== COLCON BUILD ===="
	rosdep install --from-paths src --ignore-src -r -y
	colcon build --symlink-install
fi

if [ ! -f install/setup.bash ]; then
	echo "LOI: chua co install/setup.bash. Hay chay './run.sh' khong co --no-build." >&2
	exit 1
fi

# shellcheck disable=SC1091
source install/setup.bash

echo "==== KHOI DONG XE (camera 2 lan + ESP32 + LiDAR) ===="
ros2 launch fusion_node fusion.launch.py "${ARGS[@]}" &
VEHICLE_PID=$!

if [ "$GUI" -eq 1 ]; then
	echo "==== KHOI DONG DASHBOARD ===="
	ros2 run gui_matplotlib gui_matplotlib
fi

wait "$VEHICLE_PID"
