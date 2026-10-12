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
# Dong dashboard (nut X tren cua so) hoac Ctrl+C -> tat CA chuong trinh:
# node gui lenh dung (EMG, dev 0) -> ESP32 nha ga, servo ve 90 do.
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
VEHICLE_PID=""
cleanup() {
	trap - EXIT INT TERM
	[ -n "$GUI_PID" ] && kill "$GUI_PID" 2>/dev/null || true
	if [ -n "$VEHICLE_PID" ] && kill -0 "$VEHICLE_PID" 2>/dev/null; then
		# SIGINT cho CA NHOM launch (fusion_node, lidar): fusion_node thoat dang
		# hoang, gui EMG + dev 0 (servo 90) truoc khi dong serial; het node thi
		# launch tu thoat. Gui ca nhom vi ros2 launch co the dang lo SIGINT
		# (12/10 10:46: dong dashboard xong xe van chay). Doi toi 10 s; chua
		# thoat thi TERM ca nhom (fusion_node bat ca SIGTERM, van gui EMG).
		kill -INT -- "-$VEHICLE_PID" 2>/dev/null || kill -INT "$VEHICLE_PID" 2>/dev/null || true
		for _ in $(seq 100); do
			kill -0 "$VEHICLE_PID" 2>/dev/null || break
			sleep 0.1
		done
		kill -TERM -- "-$VEHICLE_PID" 2>/dev/null || true
	fi
	wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# Job control: moi tien trinh nen co nhom rieng (PGID = PID) de cleanup() gui
# tin hieu cho ca nhom launch. Ctrl+C toi run.sh -> cleanup() chuyen tiep.
set -m

echo "==== KHOI DONG XE (camera + ESP32 /dev/ttyUSB0 + LiDAR /dev/ttyUSB1 + Foxglove) ===="
# Laptop cung WiFi: Foxglove -> Open connection -> Foxglove WebSocket -> dia chi duoi
for ip in $(hostname -I); do
	case "$ip" in *:*) ;; *) echo "  Foxglove: ws://$ip:8765" ;; esac
done
ros2 launch fusion_node fusion.launch.py "${ARGS[@]}" &
VEHICLE_PID=$!

if [ "$GUI" -eq 1 ]; then
	echo "==== KHOI DONG DASHBOARD (SPACE = chay/dung, ESC = dung, nut X cua so = thoat) ===="
	python3 src/gui/gui.py &
	GUI_PID=$!
fi

if [ -n "$GUI_PID" ]; then
	# Thoat khi 1 trong 2 dung: dong dashboard -> tat xe; xe loi -> tat dashboard
	wait -n "$VEHICLE_PID" "$GUI_PID" || true
	echo "==== DUNG CHUONG TRINH (servo ve 90) ===="
else
	wait "$VEHICLE_PID" || true
fi
