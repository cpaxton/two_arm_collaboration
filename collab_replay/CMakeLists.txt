cmake_minimum_required(VERSION 2.8.3)
project(collab_replay)

SET(CMAKE_CXX_FLAGS "-std=c++0x")

# FIND DEPENDENCIES
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs tf roscpp rosbag
  oro_barrett_msgs
  dmp
)

find_package(OpenCV)

find_package(Boost REQUIRED COMPONENTS system)

add_message_files(FILES Features.msg Segment.msg)
generate_messages(DEPENDENCIES geometry_msgs)

# PACKAGE SETTINGS
catkin_package(
  CATKIN_DEPENDS geometry_msgs tf roscpp rosbag collab_env collab_spacenav oro_barrett_msgs dmp
  DEPENDS system_lib
)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories( ${catkin_INCLUDE_DIRS} )
link_directories( ${catkin_LIBRARY_DIRS} )

add_executable (collab_replay_test src/test.cpp src/read.hpp)
add_dependencies(collab_replay_test ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(collab_replay_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable (rrec src/run_record.cpp src/record.hpp src/transform.hpp)
add_dependencies(rrec ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(rrec ${catkin_LIBRARIES})

add_executable (b2play src/barrett_play.cpp src/read.hpp src/features.hpp)
add_dependencies(b2play ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(b2play ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable (registered_b2play src/register_play.cpp src/read.hpp src/features.hpp)
add_dependencies(registered_b2play ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(registered_b2play ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

set_target_properties(collab_replay_test
                      PROPERTIES OUTPUT_NAME test
                      PREFIX "")

