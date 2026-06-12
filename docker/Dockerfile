FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    dbus-x11 \
    git \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    mesa-utils \
    novnc \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    tigervnc-standalone-server \
    websockify \
    x11-utils \
    xfce4 \
    xfce4-terminal \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace/vision_guided_robot_ws
COPY . .

RUN rosdep init 2>/dev/null || true \
    && rosdep update \
    && source /opt/ros/humble/setup.bash \
    && rosdep install --from-paths src --ignore-src -r -y \
    && colcon build --symlink-install

ENV DISPLAY=:1
ENV ROS_DOMAIN_ID=42
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV QT_X11_NO_MITSHM=1
ENV GZ_SIM_RESOURCE_PATH=/workspace/vision_guided_robot_ws/install/vision_guided_robot/share/vision_guided_robot/models

EXPOSE 6080

ENTRYPOINT ["/bin/bash", "/workspace/vision_guided_robot_ws/docker/entrypoint.sh"]
