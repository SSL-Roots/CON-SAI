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

add_executable(${PROJECT_NAME}_node
    src/real_sender_node.cpp
)

add_library(serializer
  src/serializer.cpp
)

target_link_libraries(${PROJECT_NAME}_node
    serializer
    ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_node consai_msgs_generate_messages_cpp)

if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(utest src/tests/test_serializer.cpp)
    target_link_libraries(utest serializer ${catkin_LIBRARIES})
endif()
