#!/usr/bin/env bash
set -e

sudo apt update
sudo apt upgrade -y

sudo apt install -y \
    ros-humble-ros-base \
    ros-dev-tools \
    ros-humble-rviz2 \
    ros-humble-rqt-graph \
    ros-humble-imu-tools \
    ros-humble-xacro \
    xterm \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-teleop-twist-keyboard \
    ros-humble-teleop-twist-joy \
    ros-humble-twist-stamper \
    ros-humble-twist-mux \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-robot-localization \
    ros-humble-nmea-navsat-driver \
    ros-humble-phidgets-drivers \
    ros-humble-velodyne \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-interfaces \
    ros-humble-gz-ros2-control