
cmake_minimum_required(VERSION 3.0.2)

project(road_description)

find_package(catkin REQUIRED COMPONENTS roscpp)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

catkin_package(
    INCLUDE_DIRS include
	CATKIN_DEPENDS roscpp
)
install(DIRECTORY include/road_description/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN "*.h")