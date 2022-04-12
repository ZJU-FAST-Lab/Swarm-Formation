cmake_minimum_required(VERSION 2.8.3)
project(swarm_graph)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  tf
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES swarm_graph
   CATKIN_DEPENDS message_runtime roscpp std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  SYSTEM
  include
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(swarm_graph
  src/swarm_graph.cpp
)






