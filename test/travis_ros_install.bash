#!/bin/bash -xve

# Install ROS
# Reference : http://wiki.ros.org/ja/indigo/Installation/Ubuntu

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'

wget http://packages.ros.org/ros.key -O - | sudo apt-key add -

sudo apt-get update 

sudo apt-get -y install ros-indigo-ros-base

sudo rosdep init
rosdep update

sudo apt-get -y install python-rosinstall


# Setup catkin
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
source /opt/ros/indigo/setup.bash
catkin_init_workspace
cd ~/catkin_ws
catkin_make
