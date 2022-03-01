#!/bin/bash

sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
apt update -y
DEBIAN_FRONTEND="noninteractive" apt install -y ros-noetic-desktop-full
DEBIAN_FRONTEND="noninteractive" apt install -y ros-noetic-pcl-ros
DEBIAN_FRONTEND="noninteractive" apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool \
                                                ros-noetic-geographic-msgs ros-noetic-geodesy ros-noetic-vision-msgs libgeographic-dev \
                                                ros-noetic-ros-numpy ros-noetic-visualization-msgs ros-noetic-diagnostic-updater \
                                                ros-noetic-eigen-conversions ros-noetic-tf2-geometry-msgs ros-noetic-roslint \
                                                ros-noetic-cv-bridge ros-noetic-image-geometry ros-noetic-ackermann-msgs \
						                        ros-noetic-derived-object-msgs ros-noetic-rqt-gui ros-noetic-rqt-gui-py \
                                                ros-noetic-rviz ros-noetic-roslint

rosdep init
rosdep update
