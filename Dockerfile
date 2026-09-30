# Multi-stage build for Autonomous Vehicle ROS 2 workspace
# Stage 1: Base with ROS 2 and dependencies
FROM ros:humble-ros-base-jammy AS base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-venv python3-dev \
    libopencv-dev python3-opencv \
    v4l-utils udev usbutils \
    libncnn-dev ncnn-vulkan-tools \
    build-essential cmake git \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install --no-cache-dir \
    ncnn-vulkan opencv-python numpy

# Stage 2: Build workspace
FROM base AS builder

WORKDIR /workspace

# Copy source
COPY src/ ./src/

# Install rosdep dependencies
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# Build
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Stage 3: Runtime
FROM base AS runtime

ENV ROS_DISTRO=humble

COPY --from=builder /workspace/install /opt/ros/${ROS_DISTRO}/install
COPY --from=builder /workspace/src /opt/ros/${ROS_DISTRO}/src

# Setup entrypoint
COPY docker-entrypoint.sh /docker-entrypoint.sh
RUN chmod +x /docker-entrypoint.sh

# Create non-root user
ARG UID=1000
ARG GID=1000
RUN groupadd -g ${GID} autocar && \
    useradd -u ${UID} -g ${GID} -m -s /bin/bash autocar && \
    usermod -a -G video,dialout autocar

USER autocar
WORKDIR /home/autocar

ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["ros2", "launch", "autonomous_vehicle", "bringup.launch.py"]