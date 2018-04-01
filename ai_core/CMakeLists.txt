cmake_minimum_required(VERSION 2.8.3)
project(ai_core)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(tests/test_ai_core.test)
  add_rostest(tests/test_simulator.test)
  add_rostest(tests/test_robot.test ARGS ai_name:="/" number:=0)
endif()

