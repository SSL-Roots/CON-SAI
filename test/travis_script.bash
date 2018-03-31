#!/bin/bash -xve

catkin_make run_tests

rostest ai_core test_simulator.launch
rostest ai_core test_robot.launch ai_name:="/" number:=0
rostest ai_core test_ai_core.launch
