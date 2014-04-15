cmake_minimum_required(VERSION 2.8.3)
project(lcsr_construction_plugin)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  sensor_msgs
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(ModelJointStatePublisher src/plugin.cpp)
target_link_libraries(ModelJointStatePublisher ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

catkin_package(
  LIBRARIES ModelJointStatePublisher
  DEPENDS
    roscpp
    gazebo_ros
    sensor_msgs
)
