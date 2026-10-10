# ============================================================
# OrbbecSDK_ROS2 Docker Image
# ROS2 Humble + Orbbec camera driver + capture script
# ============================================================
FROM osrf/ros:humble-desktop

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# System dependencies (from README_CN)
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgflags-dev \
    nlohmann-json3-dev \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-compressed-image-transport \
    ros-humble-image-publisher \
    ros-humble-camera-info-manager \
    ros-humble-diagnostic-updater \
    ros-humble-diagnostic-msgs \
    ros-humble-statistics-msgs \
    ros-humble-xacro \
    ros-humble-backward-ros \
    ros-humble-cv-bridge \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-sensor-msgs \
    ros-humble-tf2-msgs \
    libdw-dev \
    libssl-dev \
    mesa-utils \
    libgl1 \
    libgoogle-glog-dev \
    python3-pip \
    python3-venv \
    && rm -rf /var/lib/apt/lists/*

# Create colcon workspace
RUN mkdir -p /ros2_ws/src

# Copy OrbbecSDK_ROS2 source
COPY . /ros2_ws/src/OrbbecSDK_ROS2/

# Fix Git LFS symlinks (Windows filesystem breaks Linux symlinks)
RUN cd /ros2_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/x64 && \
    rm -f libOrbbecSDK.so libOrbbecSDK.so.2 && \
    ln -s libOrbbecSDK.so.2.8.6 libOrbbecSDK.so.2 && \
    ln -s libOrbbecSDK.so.2 libOrbbecSDK.so

# Build
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd /ros2_ws && \
    colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Python venv for capture script (numpy<2 for ROS2 Humble cv_bridge compatibility)
RUN python3 -m venv /opt/capture_venv && \
    /opt/capture_venv/bin/pip install --no-cache-dir -i https://pypi.tuna.tsinghua.edu.cn/simple "numpy<2" opencv-python

# Entrypoint
COPY docker_entrypoint.sh /docker_entrypoint.sh
RUN chmod +x /docker_entrypoint.sh

ENTRYPOINT ["/docker_entrypoint.sh"]
