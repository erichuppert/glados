cmake_minimum_required(VERSION 2.8.3)
project(orc_pub)
cmake_policy(SET CMP0054 NEW)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  orc_utils
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES orc_pub_odometry orc_pub_status
  CATKIN_DEPENDS roscpp rospy orc_utils
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(orc_pub_odometry src/odometry.cpp)

add_library(orc_pub_status src/status.cpp)

add_executable(orc_pub src/main.cpp)

add_dependencies(orc_pub ${catkin_EXPORTED_TARGETS})

target_link_libraries(orc_pub
  orc_pub_odometry
  orc_pub_status
  ${catkin_LIBRARIES}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
