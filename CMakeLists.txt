cmake_minimum_required(VERSION 2.8.3)
project(laser_collision_detection)

#find_path(FCL_INCLUDE_DIRS
#NAMES fcl/collision.h
#HINTS ${PC_FCL_INCLUDEDIR}
#PATHS "${CMAKE_INSTALL_PREFIX}/include")

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  rospy
  std_msgs
  sensor_msgs
  visualization_msgs
  tf
  laser_geometry
  geometry_msgs
)

find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)
find_package(octomap REQUIRED)
pkg_check_modules(LIBFCL REQUIRED fcl)
find_library(LIBFCL_LIBRARIES_FULL ${LIBFCL_LIBRARIES} ${LIBFCL_LIBRARY_DIRS})
set(LIBFCL_LIBRARIES "${LIBFCL_LIBRARIES_FULL}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

message ("=========== FCL LIB ${LIBFCL_LIBRARIES}")
link_libraries(${OCTOMAP_LIBRARIES})
include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  #INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs
)

add_executable(laser_obstacle_detect src/laser_obstacle_detect.cpp)
target_link_libraries(laser_obstacle_detect
   ${LIBFCL_LIBRARIES}
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   ${OCTOMAP_LIBRARIES}
   ${PCL_INCLUDE_DIRS}
   ${PCL_LIBRARIES}
)
