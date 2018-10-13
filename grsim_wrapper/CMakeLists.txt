cmake_minimum_required(VERSION 2.8.3)
project(grsim_wrapper)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  consai_msgs
  roscpp
  rospy
  std_msgs
  nav_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_add_nosetests(scripts/tests/test_format_converter.py)
