cmake_minimum_required(VERSION 2.8.3)
project(behavior_tree_visualizer)

find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/rqt_test_button
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
