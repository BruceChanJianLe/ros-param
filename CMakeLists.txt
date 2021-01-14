cmake_minimum_required(VERSION 3.0.2)
project(ros-param)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  rospy
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ros-param
#  CATKIN_DEPENDS dynamic_reconfigure roscpp rospy
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ros_param_node
  src/ros_param_node.cpp
  src/ros_param.cpp
)
target_link_libraries(ros_param_node ${catkin_LIBRARIES})