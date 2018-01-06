cmake_minimum_required(VERSION 2.8.3)
project(avoiding_point_generator)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  roscpp
  rospy
  tf
  consai_msgs
)

generate_dynamic_reconfigure_options(
  cfg/point_generator.cfg
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_compile_options(-std=c++11)

add_executable(${PROJECT_NAME}_node src/avoiding_point_generator_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
add_dependencies(avoiding_point_generator_node consai_msgs_generate_messages_cpp)

add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)
