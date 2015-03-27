cmake_minimum_required(VERSION 2.8.3)
project(orc)
cmake_policy(SET CMP0054 NEW)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES orc_utils orc_odometry orc_status
  CATKIN_DEPENDS roscpp rospy
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(orc_odometry src/odometry.cpp)

add_library(orc_status src/status.cpp)

add_library(orc_utils
  src/udp_util.c
  src/varray.c
  src/uorc.c
)

add_executable(orc src/main.cpp)

add_dependencies(orc ${catkin_EXPORTED_TARGETS})

target_link_libraries(orc
  orc_utils
  orc_odometry
  orc_status
  pthread
  ${catkin_LIBRARIES}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu99")
