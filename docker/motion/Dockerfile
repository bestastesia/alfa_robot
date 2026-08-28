ARG CONTRACT_RUNTIME_IMAGE=robot/contract-runtime:interfaces-92d6ff2-20260827

FROM ${CONTRACT_RUNTIME_IMAGE} AS motion-deps

USER root
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update \
    && apt-get install -y --no-install-recommends --no-upgrade \
      python3-numpy \
      python3-pip \
      ros-humble-moveit \
      ros-humble-ros2-controllers \
    && python3 -m pip install --no-cache-dir rerun-sdk==0.33.1 \
    && rm -rf /var/lib/apt/lists/*

FROM motion-deps AS builder

USER root
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update \
    && apt-get install -y --no-install-recommends --no-upgrade \
      build-essential \
      libeigen3-dev \
      libnlopt-cxx-dev \
      nlohmann-json3-dev \
      python3-colcon-common-extensions \
      ros-humble-rosidl-default-generators \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /motion_build
COPY ros2_ws/src/ src/
COPY tools/demonstration0720/armmotion/ros2_ws/src/armmotion_demo/ src/armmotion_demo/

RUN source /opt/ros/humble/setup.bash \
    && source /opt/robot_contract/setup.bash \
    && colcon build \
      --merge-install \
      --install-base /opt/motion \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTING=OFF \
        -DPYTHON_EXECUTABLE=/usr/bin/python3 \
        -DPython3_EXECUTABLE=/usr/bin/python3 \
      --packages-up-to armmotion_demo

FROM motion-deps AS runtime

ARG MOTION_VERSION=dev
ARG MOTION_GIT_SHA=unknown
ARG ROBOT_INTERFACES_SHA=92d6ff2ed0b45684d7da2170d96703ca8be569f4
ARG CONTRACT_RUNTIME_IMAGE=robot/contract-runtime:interfaces-92d6ff2-20260827

USER root
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

COPY --from=builder /opt/motion /opt/motion
COPY docker/motion/entrypoint.sh /usr/local/bin/motion-entrypoint
COPY docker/motion/healthcheck.sh /usr/local/bin/motion-healthcheck

RUN chmod 0755 /usr/local/bin/motion-entrypoint /usr/local/bin/motion-healthcheck \
    && mkdir -p /opt/motion-release /var/lib/robot/motion /var/log/robot/motion \
    && printf '%s\n' \
      "schema_version: 1" \
      "motion_version: ${MOTION_VERSION}" \
      "motion_git_sha: ${MOTION_GIT_SHA}" \
      "robot_interfaces_sha: ${ROBOT_INTERFACES_SHA}" \
      "contract_runtime_image: ${CONTRACT_RUNTIME_IMAGE}" \
      "ros_distro: humble" \
      "rmw_implementation: rmw_fastrtps_cpp" \
      "fastdds_profiles: /etc/robot/fastdds.xml" \
      "stage_action: /motion/execute_stage" \
      "readiness_topic: /motion/readiness" \
      "trajectory_action: /whole_body_jtc/follow_joint_trajectory" \
      > /opt/motion-release/manifest.yaml \
    && chown -R 1000:1000 /var/lib/robot/motion /var/log/robot/motion

LABEL org.opencontainers.image.title="ALFA Motion Domain" \
      org.opencontainers.image.version="${MOTION_VERSION}" \
      org.opencontainers.image.revision="${MOTION_GIT_SHA}" \
      org.opencontainers.image.source="https://github.com/kkozia188/alfa_robot" \
      robot.interfaces.revision="${ROBOT_INTERFACES_SHA}"

ENV ROS_DOMAIN_ID=7 \
    ROS_LOCALHOST_ONLY=0 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    FASTRTPS_DEFAULT_PROFILES_FILE=/etc/robot/fastdds.xml \
    FASTDDS_DEFAULT_PROFILES_FILE=/etc/robot/fastdds.xml \
    ROS_LOG_DIR=/var/log/robot/motion \
    ARMMOTION_OUTPUT_ROOT=/var/lib/robot/motion \
    ARMMOTION_RUNTIME_WORKDIR=/var/lib/robot/motion \
    ARMMOTION_REQUIRE_INSTALLED_RUNTIME=1 \
    ALFA_RUNTIME_ROOT=/var/lib/robot/motion \
    ALFA_RUNTIME_WORKDIR=/var/lib/robot/motion \
    ALFA_ROS_SETUP=/opt/motion/setup.bash \
    MOTION_USE_MOCK_RT_CONTROL=false \
    MOTION_DRY_RUN=false

USER 1000:1000
WORKDIR /var/lib/robot/motion

ENTRYPOINT ["/usr/local/bin/motion-entrypoint"]
CMD ["server"]

HEALTHCHECK --interval=10s --timeout=6s --start-period=90s --retries=3 \
  CMD ["/usr/local/bin/motion-healthcheck"]
