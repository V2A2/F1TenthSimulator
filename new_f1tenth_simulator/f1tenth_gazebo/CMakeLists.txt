cmake_minimum_required(VERSION 3.0.2)
project(f1tenth_gazebo)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rospy
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(world_controls_plugin SHARED plugins/world_controls_plugin.cc)
target_link_libraries(world_controls_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
