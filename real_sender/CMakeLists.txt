cmake_minimum_required(VERSION 2.8.3)
project(real_sender)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  serial
  consai_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  include/${PROJECT_NAME}
)

add_library(real_sender
  src/serializer/serializer.cpp
)

add_executable(real_sender_node
src/real_sender_node.cpp)

target_link_libraries(real_sender_node
    real_sender
    ${catkin_LIBRARIES}
)

add_dependencies(real_sender_node consai_msgs_generate_messages_cpp)
