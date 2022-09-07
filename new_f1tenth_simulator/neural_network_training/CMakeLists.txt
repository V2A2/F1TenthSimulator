cmake_minimum_required(VERSION 3.0.2)

project(neural_network_training)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs sensor_msgs OpenCV cv_bridge image_transport)

catkin_package(
	CATKIN_DEPENDS roscpp rospy
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(semantic_segmentation src/semantic_segmentation/semantic_segmentation.cpp)
add_dependencies(semantic_segmentation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(semantic_segmentation ${catkin_LIBRARIES}) 
