#!/usr/bin/env bash
set -e

# === Install ros2 humble === #

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
    ros-humble-imu-tools \
    ros-humble-xacro \
    xterm

sudo apt install -y \
    ros-humble-nmea-navsat-driver \
    ros-humble-phidgets-drivers

sudo apt install -y \
    python3-pip \
    clang-format

pip install --user \
    irobot_edu_sdk

# === Install Gazebo Ignition Fortress === #
sudo apt-get install lsb-release gnupg
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install ignition-fortress

sudo apt install -y \
    ros-humble-ros2-controllers \
    ros-humble-gz-ros2-control \
    ros-humble-ros-gz \
    ros-humble-ros-gz-bridge \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-teleop-twist-keyboard \
    ros-humble-twist-stamper \
    ros-humble-twist-mux \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup

# === Rosdep === #
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y

source /opt/ros/humble/setup.bash

# === Build === #
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
echo ""
echo "Done. Ignore cmake compile stderr. Source install to Run:"
echo ""
echo "      source ./install/setup.bash"
echo "   or source ./install/setup.zsh"


