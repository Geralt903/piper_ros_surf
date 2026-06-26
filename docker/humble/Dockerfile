FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    can-utils \
    ethtool \
    iproute2 \
    python3-colcon-common-extensions \
    python3-pip \
    ros-humble-controller-manager \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-rviz2 \
    ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir python-can scipy piper_sdk

WORKDIR /ws/piper_ros

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
