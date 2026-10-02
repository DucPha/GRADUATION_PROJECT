# Multi-stage build cho Autonomous Vehicle (ROS 2 workspace)
#
# ROS_DISTRO mặc định theo máy host: Jazzy trên Ubuntu 24.04. Đổi qua
# --build-arg ROS_DISTRO=humble nếu dựng trên Ubuntu 22.04.
ARG ROS_DISTRO=jazzy

# ============================================================================
# Stage 1: base - ROS 2 + phụ thuộc hệ thống
# ============================================================================
FROM ros:${ROS_DISTRO}-ros-base AS base

ARG ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=${ROS_DISTRO}

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    libopencv-dev python3-opencv \
    v4l-utils udev usbutils \
    build-essential cmake git \
    && rm -rf /var/lib/apt/lists/*

# ncnn cho Python binding: KHÔNG cài `opencv-python` bằng pip.
#
# opencv-python cài vào site-packages sẽ đè lên `cv2` do gói `python3-opencv`
# của hệ thống cung cấp. Module cv2 của ROS (opencv4 cho Ament) được build
# theo đúng ABI mà python3-opencv mang; bản pip mang ABI riêng, nên `import
# cv2` trong container sẽ trỏ nhầm sang bản pip và node nhận diện AI chết
# với lỗi kiểu dữ liệu numpy. python3-opencv đã cài ở trên là đủ.
#
# Lưu ý: gói pip tên là 'ncnn' (không phải 'ncnn-vulkan') - bản vulkan được
# build sẵn trong wheel. Xem https://pypi.org/project/ncnn/
RUN pip3 install --no-cache-dir --break-system-packages \
    ncnn numpy

# ============================================================================
# Stage 2: builder - cài dependency + colcon build
# ============================================================================
FROM base AS builder

WORKDIR /workspace
COPY src/ ./src/
COPY rosdep.yaml /workspace/rosdep.yaml

# rosdep install chạy với --from-paths src nên rosdep.yaml phải nằm trong
# src/, không phải ở gốc workspace.
# rosdep.yaml đã được copy vào /workspace/rosdep.yaml, copy vào src/
RUN cp /workspace/rosdep.yaml ./src/rosdep.yaml && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep update --rosdistro ${ROS_DISTRO} && \
    rosdep install --from-paths src --ignore-src -r -y

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# ============================================================================
# Stage 3: runtime
# ============================================================================
FROM base AS runtime

COPY --from=builder /workspace/install /opt/ros/${ROS_DISTRO}/install

COPY docker-entrypoint.sh /docker-entrypoint.sh
RUN chmod +x /docker-entrypoint.sh

# User không phải root, thuộc group video (camera) và dialout (serial).
# Quyền truy cập thiết bị đến từ group này, không phải từ chmod/sudo lúc chạy.
ARG UID=1000
ARG GID=1000
RUN groupadd -g ${GID} autocar && \
    useradd -u ${UID} -g ${GID} -m -s /bin/bash autocar && \
    usermod -a -G video,dialout autocar

USER autocar
WORKDIR /home/autocar

# Không thể cấp quyền định kỳ cho /dev/* khi build: thiết bị chỉ xuất hiện
# lúc container chạy (--device=...). Khi chạy cần:
#   docker run --device=/dev/video0 --device=/dev/ttyUSB0 ...
ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["ros2", "launch", "autonomous_vehicle", "bringup.launch.py"]