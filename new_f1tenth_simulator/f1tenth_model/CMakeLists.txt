cmake_minimum_required(VERSION 3.0.2)
project(f1tenth_model)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
catkin_package(
	CATKIN_DEPENDS roscpp
)
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(wheel_plugin SHARED plugins/wheel_plugin.cc)
target_link_libraries(wheel_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
add_library(test_plugin SHARED plugins/test_plugin.cc)
target_link_libraries(test_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

