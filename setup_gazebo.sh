#!/bin/bash
# setup_gazebo.bash
# https://gazebosim.org/docs/harmonic/ros_installation/

set -e

if [[ $EUID -ne 0 ]]; then
   echo "root needed" 
   exit 1
fi

ROS_DISTRO=${1:-jazzy}

echo "stage 1"
sudo apt update && sudo apt upgrade -y


echo "stage 2"
sudo apt-get install -y ros-${ROS_DISTRO}-ros-gz

echo "installation complete"
