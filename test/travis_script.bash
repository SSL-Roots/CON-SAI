#!/bin/bash -xve

cd ~/catkin_ws

catkin_make run_tests    # Always returns 0
catkin_test_results # Output previous test results
