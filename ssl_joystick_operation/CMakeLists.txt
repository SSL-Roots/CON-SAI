cmake_minimum_required(VERSION 2.8.3)
project(ssl_joystick_operation)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  consai_msgs
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
