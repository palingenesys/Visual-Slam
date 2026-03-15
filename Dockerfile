# =============================================================================
# Visual SLAM — Unitree Go2 Deployment Image
# =============================================================================
#
# Target hardware : Unitree Go2 EDU/Pro — NVIDIA Jetson Orin NX (aarch64)
# ROS2 distro     : Foxy  (Ubuntu 20.04, matches Go2 Jetson JetPack 5.x)
# DDS middleware  : CycloneDDS
# Camera          : Intel RealSense D435i (USB 3.0)
# SLAM backend    : RTAB-Map (rgbd_odometry + rtabmap nodes)
#
# NOTE: The Go2 Jetson already has ROS2 Foxy installed natively.
#       If all required packages are present on the host, you can skip Docker
#       entirely and build/run the workspace natively (see README).
#       Docker is useful for isolation and reproducibility.
#
# ── Build ─────────────────────────────────────────────────────────────────────
#   docker build -t go2-slam:foxy .
#
# ── Run (headless, DDS odometry publishing) ──────────────────────────────────
#   docker run --rm -it \
#     --privileged \
#     --network host \
#     -e ROS_DOMAIN_ID=0 \
#     -e DDS_INTERFACE=eth0 \
#     -v /dev:/dev \
#     -v ~/.ros:/root/.ros \
#     go2-slam:foxy
#
# ── Run (with RViz2 via X11 / NoMachine) ─────────────────────────────────────
#   docker run --rm -it \
#     --privileged \
#     --network host \
#     -e ROS_DOMAIN_ID=0 \
#     -e DDS_INTERFACE=eth0 \
#     -e DISPLAY=$DISPLAY \
#     -v /tmp/.X11-unix:/tmp/.X11-unix \
#     -v /dev:/dev \
#     -v ~/.ros:/root/.ros \
#     go2-slam:foxy
#
# ── Remote RViz2 from laptop ─────────────────────────────────────────────────
#   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#   export ROS_DOMAIN_ID=0
#   rviz2 -d <path>/config/d435i_slam.rviz
#
# =============================================================================

FROM ros:foxy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# ── System dependencies ───────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    build-essential \
    cmake \
    git \
    libssl-dev \
    usbutils \
    libgl1-mesa-glx \
    libglu1-mesa \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# ── Intel RealSense SDK (librealsense2) ───────────────────────────────────────
# Intel's apt repo provides ARM64 packages for Ubuntu 20.04 (focal).
RUN mkdir -p /etc/apt/keyrings && \
    curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp \
        | tee /etc/apt/keyrings/librealsense.pgp > /dev/null && \
    echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] \
        https://librealsense.intel.com/Debian/apt-repo \
        $(lsb_release -cs) main" \
        | tee /etc/apt/sources.list.d/librealsense.list && \
    apt-get update && apt-get install -y --no-install-recommends \
        librealsense2 \
        librealsense2-dev \
        librealsense2-utils \
    && rm -rf /var/lib/apt/lists/*

# ── ROS2 Foxy packages ────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    # RealSense ROS2 driver
    ros-foxy-realsense2-camera \
    ros-foxy-realsense2-description \
    # RTAB-Map: odometry + SLAM backend
    ros-foxy-rtabmap-ros \
    # IMU fusion
    ros-foxy-imu-filter-madgwick \
    # CycloneDDS RMW (Foxy defaults to FastDDS; we override to CycloneDDS)
    ros-foxy-rmw-cyclonedds-cpp \
    ros-foxy-cyclonedds \
    # Navigation & TF
    ros-foxy-nav-msgs \
    ros-foxy-geometry-msgs \
    ros-foxy-tf2-ros \
    ros-foxy-tf2-tools \
    # Visualization
    ros-foxy-rviz2 \
    # Lifecycle / composable nodes
    ros-foxy-rclcpp-lifecycle \
    ros-foxy-rclcpp-components \
    && rm -rf /var/lib/apt/lists/*

# ── Copy workspace source ─────────────────────────────────────────────────────
WORKDIR /ros2_ws
COPY ros2_ws/src ./src

# ── Build workspace ───────────────────────────────────────────────────────────
RUN rosdep init 2>/dev/null || true && rosdep update 2>/dev/null || true

RUN /bin/bash -c "\
    source /opt/ros/foxy/setup.bash && \
    colcon build \
        --packages-select anubi_slam point_cloud \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --event-handlers console_direct+ \
"

# ── DDS configuration ─────────────────────────────────────────────────────────
RUN mkdir -p /config
COPY cyclonedds_go2.xml /config/cyclonedds_go2.xml

# ── Entrypoint ────────────────────────────────────────────────────────────────
COPY docker-entrypoint.sh /docker-entrypoint.sh
RUN chmod +x /docker-entrypoint.sh

# ── Environment defaults ──────────────────────────────────────────────────────
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///config/cyclonedds_go2.xml
ENV ROS_DOMAIN_ID=0
ENV HOME=/root

ENTRYPOINT ["/docker-entrypoint.sh"]

# Default: full SLAM pipeline. Override examples:
#   Pure VIO only:       docker run ... go2-slam:foxy ros2 launch anubi_slam slam_go2.launch.py use_slam:=false rviz:=false
#   Resume previous map: docker run ... go2-slam:foxy ros2 launch anubi_slam slam_go2.launch.py delete_db:=false
CMD ["ros2", "launch", "anubi_slam", "slam_go2.launch.py"]
