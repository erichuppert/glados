cmake_minimum_required(VERSION 2.8.3)
project(orc_utils)
cmake_policy(SET CMP0054 NEW)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES orc_utils
  CATKIN_DEPENDS roscpp rospy
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(orc_utils
  udp_util.c
  varray.c
  uorc.c
)

## Specify libraries to link a library or executable target against
target_link_libraries(orc_utils
  pthread
  ${catkin_LIBRARIES}
)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu99")
