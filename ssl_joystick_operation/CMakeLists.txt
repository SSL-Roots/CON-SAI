cmake_minimum_required(VERSION 2.8.3)
project(ssl_joystick_operation)

find_package(catkin REQUIRED COMPONENTS
  joy
  rospy
  sensor_msgs
  std_msgs
  consai_msgs
)

catkin_package(
  CATKIN_DEPENDS joy rospy sensor_msgs std_msgs consai_msgs
    
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
