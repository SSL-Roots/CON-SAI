cmake_minimum_required(VERSION 2.8.3)
project(trapezoidal_control)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  roscpp
  rospy
  tf
  consai_msgs
)

generate_dynamic_reconfigure_options(
  cfg/controller.cfg
)

catkin_package()

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_compile_options(-std=c++11)

add_executable(${PROJECT_NAME}_node 
    src/trapezoidal_control_node.cpp
)

add_library(controller
    src/trapezoidal_controller.cpp)

target_link_libraries(${PROJECT_NAME}_node
    controller
    ${catkin_LIBRARIES}
)

add_dependencies(trapezoidal_control_node consai_msgs_generate_messages_cpp)

add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)

if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(utest src/tests/test_trapezoidal_controller.cpp)
    target_link_libraries(utest controller ${catkin_LIBRARIES})
endif()
