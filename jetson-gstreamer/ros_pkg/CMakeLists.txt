cmake_minimum_required(VERSION 2.8.3)
project(ros_pkg)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/ros-gstreamer-node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
