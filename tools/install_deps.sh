#!/bin/bash
# Cai toan bo thu vien Ubuntu 24.04 + ROS 2 Jazzy can cho xe va dashboard.
#
#   ./tools/install_deps.sh
#
# Chay lai nhieu lan van an toan (apt/pip bo qua goi da co).
set -e

ROS_DISTRO="${ROS_DISTRO:-jazzy}"

echo "==== GOI APT (build C++, camera, serial, ROS 2, LiDAR) ===="
sudo apt update
sudo apt install -y \
	build-essential cmake git v4l-utils \
	libopencv-dev libjpeg-turbo8-dev \
	python3-opencv python3-numpy python3-matplotlib python3-serial python3-pip \
	python3-colcon-common-extensions python3-rosdep \
	python3-pyside2.qtcore python3-pyside2.qtgui python3-pyside2.qtwidgets \
	"ros-${ROS_DISTRO}-desktop" \
	"ros-${ROS_DISTRO}-rplidar-ros" \
	"ros-${ROS_DISTRO}-cv-bridge"

echo "==== PYSIDE6 CHO DASHBOARD (pip, chi cho user hien tai) ===="
# Ubuntu 24.04 khong co PySide6 trong apt. Neu pip loi, dashboard tu dung
# PySide2 (da cai o tren) nen van chay duoc.
pip3 install --user --break-system-packages PySide6-Essentials ||
	echo "Canh bao: khong cai duoc PySide6, dashboard se dung PySide2."

echo "==== ROSDEP ===="
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
	sudo rosdep init
fi
rosdep update

echo "==== QUYEN CAMERA + SERIAL ===="
if ! id -nG "$USER" | grep -qw dialout || ! id -nG "$USER" | grep -qw video; then
	sudo usermod -aG dialout,video "$USER"
	echo "Da them $USER vao nhom dialout, video: DANG XUAT va dang nhap lai."
fi

echo "==== XONG ===="
