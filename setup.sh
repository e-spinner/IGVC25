#!/usr/bin/env bash
set -e

# === Install ros === 2 #

locale

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb



sudo apt update
sudo apt upgrade -y

sudo apt install -y \
    ros-humble-ros-base \
    ros-dev-tools \
    ros-humble-rviz2 \
    ros-humble-rqt-graph \
    ros-humble-imu-tools

sudo apt install -y \
    ros-humble-nmea-navsat-driver \
    ros-humble-phidgets-drivers

# === Rosdep === #
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# === Build === #
colcon build --symlink-install
echo "Done. Source install to Run:"
echo ""
echo "      source ./install/setup.bash"
echo "   or source ./install/setup.zsh"


