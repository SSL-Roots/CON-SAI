cmake_minimum_required(VERSION 2.8.3)
project(world_observer)

find_package(PkgConfig)

if("$ENV{ROS_DISTRO}" STRGREATER "lunar")
    pkg_check_modules(BFL REQUIRED orocos-bfl)
else()
    pkg_check_modules(BFL REQUIRED bfl)
endif()


message("BFL include dirs:" ${BFL_INCLUDE_DIRS})
message("BFL library dirs:" ${BFL_LIBRARY_DIRS})

include_directories(${BFL_INCLUDE_DIRS})
link_directories(${BFL_LIBRARY_DIRS})

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  consai_msgs
)

catkin_package()

include_directories(
  "include"
  ${catkin_INCLUDE_DIRS}
)

add_compile_options(-std=c++11)

add_executable(world_observer
  src/world_observer.cpp
  src/estimator/estimator.cpp
  src/estimator/enemy_estimator.cpp
  src/estimator/ball_estimator.cpp
)

target_link_libraries(world_observer
 ${catkin_LIBRARIES}
 ${BFL_LIBRARIES}
)

add_dependencies(world_observer consai_msgs_generate_messages_cpp)
