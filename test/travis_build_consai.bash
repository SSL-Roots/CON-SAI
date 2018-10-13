#!/bin/bash -xve

# Install packages

# Google Protobuf
sudo apt-get -y install libprotobuf-dev libprotoc-dev protobuf-compiler
sudo pip2 install protobuf

# ROS Packages
sudo apt-get -y install ros-$ROS_DISTRO-navigation ros-$ROS_DISTRO-bfl

# pygraphviz
sudo apt-get -y install graphviz libgraphviz-dev pkg-config
sudo pip2 install pygraphviz --install-option="--include-path=/usr/include/graphviz" --install-option="--library-path=/usr/lib/graphviz/"


# Build CON-SAI
rsync -av ./ ~/catkin_ws/src/CON-SAI

cd ~/catkin_ws/src/CON-SAI
git submodule init
git submodule update

cd ~/catkin_ws
catkin_make
