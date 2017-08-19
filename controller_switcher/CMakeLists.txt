cmake_minimum_required(VERSION 2.8.3)
project(controller_switcher)

find_package(catkin REQUIRED COMPONENTS
  consai_msgs
  roscpp
  rospy
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
