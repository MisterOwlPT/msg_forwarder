cmake_minimum_required(VERSION 3.0.2)
project(msg_forwarder)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  rospy_message_converter
)

catkin_python_setup()

catkin_package(
 CATKIN_DEPENDS rospy rospy_message_converter
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(
  PROGRAMS scripts/forwarder.py 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)