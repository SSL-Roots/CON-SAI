cmake_minimum_required(VERSION 2.8.3)
project(decision_making)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_add_nosetests(scripts/tests/test_world_model.py)
catkin_add_nosetests(scripts/tests/test_observer.py)
catkin_add_nosetests(scripts/tests/test_tactics_run.py)
catkin_add_nosetests(scripts/tests/test_tool.py)
catkin_add_nosetests(scripts/tests/test_state.py)
catkin_add_nosetests(scripts/tests/test_coordinate.py)
catkin_add_nosetests(scripts/tests/test_assignor.py)
catkin_add_nosetests(scripts/tests/test_formation.py)
