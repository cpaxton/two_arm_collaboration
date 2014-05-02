cmake_minimum_required(VERSION 2.8.3)
project(lcsr_replay)

SET(CMAKE_CXX_FLAGS "-std=c++0x")

# FIND DEPENDENCIES
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs tf roscpp rosbag oro_barrett_msgs
)

# PACKAGE SETTINGS
catkin_package(
  CATKIN_DEPENDS geometry_msgs tf roscpp rosbag lcsr_barrett lcsr_collab lcsr_spacenav oro_barrett_msgs
  DEPENDS system_lib
)

include_directories( ${catkin_INCLUDE_DIRS} )
link_directories( ${catkin_LIBRARY_DIRS} )

add_executable (lcsr_replay_test src/test.cpp src/read.hpp)
target_link_libraries(lcsr_replay_test ${catkin_LIBRARIES})

add_executable (rrec src/run_record.cpp src/record.hpp)
target_link_libraries(rrec ${catkin_LIBRARIES})

set_target_properties(lcsr_replay_test
                      PROPERTIES OUTPUT_NAME test
                      PREFIX "")
