cmake_minimum_required(VERSION 2.8.3)
project(lcsr_replay)

# FIND DEPENDENCIES
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs tf roscpp rosbag
)

# PACKAGE SETTINGS
catkin_package(
  CATKIN_DEPENDS geometry_msgs tf roscpp rosbag lcsr_barrett lcsr_collab lcsr_spacenav
  DEPENDS system_lib
)

include_directories( ${catkin_INCLUDE_DIRS} )
link_directories( ${catkin_LIBRARY_DIRS} )
