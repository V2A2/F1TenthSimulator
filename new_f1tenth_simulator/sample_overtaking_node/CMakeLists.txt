cmake_minimum_required(VERSION 3.0.2)
project(sample_overtaking_node)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs sensor_msgs road_description)

catkin_package(
	CATKIN_DEPENDS roscpp rospy
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)
add_executable(sample_overtaking_node src/sample_overtaking_node.cpp)
add_dependencies(sample_overtaking_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sample_overtaking_node ${catkin_LIBRARIES} ) 
