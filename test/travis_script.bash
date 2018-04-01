#!/bin/bash -xve

cd ~/catkin_ws

# Code level tests
catkin_make run_tests # Always returns 0
catkin_test_results   # Output previous test results

# Node level tests
rostest ai_core test_ai_core.test
rostest ai_core test_simulator.test
rostest ai_core test_robot.test ai_name:="/" number:=0
