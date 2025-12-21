#!/bin/bash
# setup_ros2_jazzy.bash
# source https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

set -e

if [[ $EUID -ne 0 ]]; then
   echo "root needed" 
   exit 1
fi

ROS_DISTRO=${1:-jazzy}

echo "stage 1"
locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale

echo "stage 2"
sudo apt install -y software-properties-common
sudo add-apt-repository universe

echo "stage 3"
sudo apt install -y curl
ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb \
"https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

echo "stage 4"
sudo apt update && sudo apt install -y ros-dev-tools

echo "stage 5"
sudo apt update
sudo apt upgrade -y

echo "stage 6"
sudo apt install -y ros-${ROS_DISTRO}-desktop

echo "stage 7"
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
source /opt/ros/${ROS_DISTRO}/setup.bash

echo "installation complete"
