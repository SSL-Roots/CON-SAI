cmake_minimum_required(VERSION 2.8.3)
project(sai_visualizer)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui
  rqt_gui_py
)

catkin_python_setup()

catkin_package()

install(PROGRAMS
  scripts/sai_visualizer
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY 
  resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
