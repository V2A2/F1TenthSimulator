cmake_minimum_required(VERSION 3.0.2)
project(black_car_model)
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

# Build the stand-alone test program
#add_executable(vel vel.cc)

#if (${gazebo_VERSION_MAJOR} LESS 6)
#  include(FindBoost)
#  find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)
#  target_link_libraries(vel ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
#else()
#  target_link_libraries(vel ${GAZEBO_LIBRARIES})
#endif()
