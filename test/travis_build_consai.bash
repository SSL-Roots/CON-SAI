#!/bin/bash -xve

# Install packages

# Google Protobuf ver > 2.6
sudo apt-get install -y libprotobuf-dev libprotoc-dev protobuf-compiler
sudo pip2 install protobuf==2.6.1

# ROS Navigation
sudo apt-get install -y ros-indigo-navigation

# pygraphviz
sudo apt-get install -y graphviz libgraphviz-dev pkg-config
sudo pip install pygraphviz --install-option="--include-path=/usr/include/graphviz" --install-option="--library-path=/usr/lib/graphviz/"


# Build CON-SAI
git clone https://github.com/SSL-Roots/CON-SAI ~/catkin_ws/src/CON-SAI

cd ~/catkin_ws/src/CON-SAI
git submodule init
git submodule update

cd ~/catkin_ws
catkin_make
