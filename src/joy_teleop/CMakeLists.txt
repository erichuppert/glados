cmake_minimum_required(VERSION 2.8.3)
project(joy_teleop)

find_package(catkin REQUIRED COMPONENTS
  joy
  orc
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES joy_teleop
#  CATKIN_DEPENDS joy roscpp
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(joy_teleop_node src/teleop_cmd_vel.cpp)

# add_dependencies(joy_teleop_node joy_teleop_generate_messages_cpp)

target_link_libraries(joy_teleop_node
  ${catkin_LIBRARIES}
)
