#!/bin/bash -xve

cd ~/catkin_ws

catkin_make test    # Always returns 0
catkin_test_results # Output previous test results
