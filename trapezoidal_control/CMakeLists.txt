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

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/trapezoidal_control_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
add_dependencies(trapezoidal_control_node consai_msgs_generate_messages_cpp)

add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)
