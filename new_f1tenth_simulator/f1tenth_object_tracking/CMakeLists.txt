cmake_minimum_required(VERSION 3.0.2)
project(f1tenth_object_tracking)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs cv_bridge image_transport)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
catkin_package(
	CATKIN_DEPENDS roscpp rospy
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${EIGEN3_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
)
add_executable(f1tenth_object_tracking src/f1tenth_object_tracking.cpp)
add_dependencies(f1tenth_object_tracking ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(f1tenth_object_tracking ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}) 
