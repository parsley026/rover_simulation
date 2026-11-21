# syntax=docker/dockerfile:1
#
# Digital-shadow dev image: Ubuntu 24.04 (Noble) + ROS 2 Jazzy + Gazebo Harmonic.
#
# Jazzy targets Noble, and Gazebo Harmonic (gz-sim8, as pinned in
# rover_description/package.xml) is the pairing for Jazzy, so 24.04 is the
# right base. The repo's own setup scripts are reused as-is so this image
# stays in step with how the rover computer is provisioned.
#
# NOTE: the apt package names in the "extra dependencies" layer were written
# without network access and have not been built. If the build fails there,
# `rosdep install` (see compose.yaml) is the authoritative source.

FROM ubuntu:24.04

ARG ROS_DISTRO=jazzy
ENV ROS_DISTRO=${ROS_DISTRO} \
    DEBIAN_FRONTEND=noninteractive \
    TZ=Etc/UTC \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

# The setup scripts require root AND call `sudo`; ubuntu:24.04 ships without it.
RUN apt-get update \
    && apt-get install -y --no-install-recommends sudo ca-certificates curl \
    && rm -rf /var/lib/apt/lists/*

# Only the three setup scripts are copied (see ../.dockerignore); the source
# tree is bind-mounted at runtime so edits do not trigger an image rebuild.
WORKDIR /opt/setup
COPY setup_ros2.sh setup_gazebo.sh setup_others.sh ./
RUN chmod +x ./*.sh

# One layer per script so a failure points at the right stage and earlier
# (large) layers stay cached.
RUN ./setup_ros2.sh   ${ROS_DISTRO}
RUN ./setup_gazebo.sh ${ROS_DISTRO}
RUN ./setup_others.sh ${ROS_DISTRO}

# Extra dependencies declared in src/*/package.xml but not covered by the
# setup scripts (ros2_control + gz_ros2_control, nav2, rtabmap, rosbridge, ...).
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-ros2-control \
        ros-${ROS_DISTRO}-ros2-controllers \
        ros-${ROS_DISTRO}-gz-ros2-control \
        ros-${ROS_DISTRO}-navigation2 \
        ros-${ROS_DISTRO}-nav2-bringup \
        ros-${ROS_DISTRO}-rtabmap-ros \
        ros-${ROS_DISTRO}-robot-localization \
        ros-${ROS_DISTRO}-rosbridge-suite \
        libeigen3-dev \
        libyaml-cpp-dev \
        python3-numpy \
        python3-opencv \
        python3-pil \
        python3-yaml \
        mesa-utils \
        x11-apps \
    && rm -rf /var/lib/apt/lists/*

# rosdep is installed by ros-dev-tools; `init` needs to happen once as root.
# `rosdep update` is left to the user (per-user cache, needs network).
RUN rosdep init || true

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /ws
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
