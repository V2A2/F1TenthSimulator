cmake_minimum_required(VERSION 3.0.2)
project(f1tenth_semantic_segmentation)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs cv_bridge image_transport)

catkin_package(
	CATKIN_DEPENDS roscpp rospy
)
find_package(OpenCV REQUIRED)
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
add_executable(f1tenth_semantic_segmentation src/f1tenth_semantic_segmentation.cpp)
add_dependencies(f1tenth_semantic_segmentation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(f1tenth_semantic_segmentation ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}) 
