FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV PIP_DISABLE_PIP_VERSION_CHECK=1

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    python3-pip \
    python3-rosdep \
    python3-catkin-tools \
    ros-noetic-geometry-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-std-msgs \
    ros-noetic-robot-state-publisher \
    ros-noetic-tf2-ros \
    libgl1-mesa-glx \
    libglfw3 \
    libosmesa6 \
    libx11-6 \
    libxcursor1 \
    libxinerama1 \
    libxrandr2 \
    libxi6 \
    libxrender1 \
    libxxf86vm1 \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir mujoco numpy

WORKDIR /ws

CMD ["bash"]
