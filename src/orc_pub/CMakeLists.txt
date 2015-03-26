cmake_minimum_required(VERSION 2.8.3)
project(orc_pub)
cmake_policy(SET CMP0054 NEW)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  orc_utils
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES orc_pub
  CATKIN_DEPENDS roscpp rospy orc_utils
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

# add_library(orc_pub
#   src/${PROJECT_NAME}/orc_pub.cpp
# )

add_executable(orc_run src/main.cpp)

add_dependencies(orc_run ${catkin_EXPORTED_TARGETS} orc_utils)

target_link_libraries(orc_run
  ${catkin_LIBRARIES}
)
