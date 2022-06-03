cmake_minimum_required(VERSION 3.0.2)
project(f1tenth_input_handler)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)

catkin_package(
	CATKIN_DEPENDS roscpp rospy
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)
add_executable(f1tenth_input_handler src/input_handler.cpp)
add_dependencies(f1tenth_input_handler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(f1tenth_input_handler ${catkin_LIBRARIES} ) 
