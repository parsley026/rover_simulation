#!/bin/bash

set -e

if [[ $EUID -ne 0 ]]; then
   echo "root needed" 
   exit 1
fi

ROS_DISTRO=${1:-jazzy}

echo "stage 1"
sudo apt update && sudo apt upgrade -y

echo "stage 2"
sudo apt update && sudo apt install -y \
    ros-${ROS_DISTRO}-ament-cmake \
    ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-rclcpp \
    ros-${ROS_DISTRO}-rclpy \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-realtime-tools \
    ros-${ROS_DISTRO}-tf2 \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-urdf \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-ament-index-cpp \
    python3-colcon-common-extensions \
    libceres-dev \
    libpaho-mqttpp-dev \
    libpaho-mqtt-dev \
    ros-${ROS_DISTRO}-can-msgs

echo "installation complete"