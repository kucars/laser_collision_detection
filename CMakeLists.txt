cmake_minimum_required(VERSION 2.8.3)
project(laser_collision_detection)

find_path(FCL_INCLUDE_DIRS
NAMES fcl/collision.h
HINTS ${PC_FCL_INCLUDEDIR}
PATHS "${CMAKE_INSTALL_PREFIX}/include")

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
find_package(fcl REQUIRED)
find_package(octomap REQUIRED)

link_libraries(${OCTOMAP_LIBRARIES})
include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS system_lib
)

add_executable(laser_obstacle_detect src/laser_obstacle_detect.cpp)
target_link_libraries(laser_obstacle_detect
   ${fcl_LIBRARIES}
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   ${OCTOMAP_LIBRARIES}
   ${PCL_INCLUDE_DIRS}
   ${PCL_LIBRARIES}
)
