cmake_minimum_required(VERSION 2.8.3)
project(decision_making)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
