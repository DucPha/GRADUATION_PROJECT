#!/bin/bash
# Build + chay toan bo he thong xe tu hanh.
#
#   ./run.sh                      # build, roi chay xe + dashboard
#   ./run.sh --no-build           # chi chay, khong build lai
#   ./run.sh --no-gui             # khong mo dashboard (xe se DUNG cho lenh,
#                                 # them require_start:=false de tu chay)
#   ./run.sh speed_x10:=30        # tham so launch truyen nguyen
#
# Xe chi chay khi bam SPACE (hoac nut CHAY XE) tren dashboard.
# Dong dashboard hoac Ctrl+C -> xe dung.
set -e

cd "$(dirname "$0")"

BUILD=1
GUI=1
declare -a ARGS=()

for arg in "$@"; do
	case "$arg" in
		--no-build) BUILD=0 ;;
		--no-gui)   GUI=0 ;;
		--gui)      GUI=1 ;;   # giu tuong thich lenh cu
		*)          ARGS+=("$arg") ;;
	esac
done

echo "==== QUYEN TRUY CAP PHAN CUNG ===="
# Chi goi sudo khi user chua co quyen (chua vao nhom dialout/video)
for dev in /dev/ttyUSB* /dev/ttyACM* /dev/video*; do
	if [ -e "$dev" ] && [ ! -w "$dev" ]; then
		sudo chmod 666 /dev/ttyUSB* /dev/ttyACM* /dev/video* 2>/dev/null || true
		break
	fi
done

echo "==== LOAD MOI TRUONG ROS 2 ===="
# shellcheck disable=SC1091
source /opt/ros/"${ROS_DISTRO:-jazzy}"/setup.bash

if [ "$BUILD" -eq 1 ]; then
	echo "==== COLCON BUILD ===="
	rosdep install --from-paths src --ignore-src -r -y ||
		echo "Canh bao: rosdep loi (mat mang?), van tiep tuc build."
	colcon build --symlink-install
fi

if [ ! -f install/setup.bash ]; then
	echo "LOI: chua co install/setup.bash. Hay chay './run.sh' khong co --no-build." >&2
	exit 1
fi

# shellcheck disable=SC1091
source install/setup.bash

GUI_PID=""
cleanup() {
	[ -n "$GUI_PID" ] && kill "$GUI_PID" 2>/dev/null || true
	[ -n "$VEHICLE_PID" ] && kill -INT "$VEHICLE_PID" 2>/dev/null || true
	wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "==== KHOI DONG XE (camera + ESP32 /dev/ttyUSB0 + LiDAR /dev/ttyUSB1) ===="
ros2 launch fusion_node fusion.launch.py "${ARGS[@]}" &
VEHICLE_PID=$!

if [ "$GUI" -eq 1 ]; then
	echo "==== KHOI DONG DASHBOARD (SPACE = chay/dung, ESC = dung) ===="
	python3 src/gui/gui.py &
	GUI_PID=$!
fi

wait "$VEHICLE_PID"
