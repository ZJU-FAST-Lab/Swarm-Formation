cmake_minimum_required(VERSION 3.0.2)
project(vrb_planner)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  quadrotor_msgs
  plan_env
  path_searching
  traj_opt
  traj_utils
  message_generation
  cv_bridge
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES vrb_planner
 CATKIN_DEPENDS plan_env path_searching traj_opt traj_utils
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_executable(vrb_planner_node
  src/vrb_planner_node.cpp
  src/vrb_planner.cpp
  src/virtual_structure.cpp
)

target_link_libraries(vrb_planner_node 
  ${catkin_LIBRARIES}
)



