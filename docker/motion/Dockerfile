ARG ROS_BASE_IMAGE=osrf/ros:humble-desktop-full
FROM ${ROS_BASE_IMAGE}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG UBUNTU_MIRROR=https://mirrors.aliyun.com/ubuntu
ARG ROS2_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu

RUN find -L /etc/apt -maxdepth 2 -type f \
      \( -name '*.list' -o -name '*.sources' \) -exec sed -i \
      -e "s|http://archive.ubuntu.com/ubuntu|${UBUNTU_MIRROR}|g" \
      -e "s|http://security.ubuntu.com/ubuntu|${UBUNTU_MIRROR}|g" \
      -e "s|http://packages.ros.org/ros2/ubuntu|${ROS2_MIRROR}|g" \
      -e "s|https://packages.ros.org/ros2/ubuntu|${ROS2_MIRROR}|g" \
      -e "s|^Types: deb deb-src$|Types: deb|g" {} + \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
      build-essential \
      ca-certificates \
      git \
      libboost-all-dev \
      libeigen3-dev \
      libnlopt-cxx-dev \
      nlohmann-json3-dev \
      python3-colcon-common-extensions \
      python3-pip \
      python3-rosdep \
      python3-vcstool \
      ros-humble-control-msgs \
      ros-humble-moveit \
      ros-humble-rmw-fastrtps-cpp \
      ros-humble-ros2-controllers \
      ros-humble-tf2-eigen-kdl \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir rerun-sdk==0.33.1

COPY docker/motion/entrypoint.sh /motion-entrypoint.sh
RUN chmod +x /motion-entrypoint.sh

ENV ROS_DOMAIN_ID=42 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fastdds_udp_only.xml \
    FASTDDS_DEFAULT_PROFILES_FILE=/etc/fastdds_udp_only.xml \
    MOTION_SOURCE_ROOT=/repo \
    MOTION_WORKSPACE=/workspace \
    MOTION_OUTPUT_ROOT=/motion_data \
    MOTION_ROLE=server \
    MOTION_RT_MODE=external \
    MOTION_DRY_RUN=true

WORKDIR /workspace
ENTRYPOINT ["/motion-entrypoint.sh"]
