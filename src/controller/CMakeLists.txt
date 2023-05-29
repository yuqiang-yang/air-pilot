cmake_minimum_required(VERSION 2.8.3)
project(so3_control)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  quadrotor_msgs
  tf
  nodelet
  cmake_utils
)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

catkin_package(
)


find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})


include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(irobot_msgs
#   src/${PROJECT_NAME}/irobot_msgs.cpp
# )

## Declare a cpp executable
#add_executable(odom_visualization src/odom_visualization.cpp)
add_library(SO3Control src/SO3Control.cpp)
add_library(so3_control_nodelet src/so3_control_nodelet.cpp)

target_link_libraries(so3_control_nodelet 
    ${catkin_LIBRARIES}
    SO3Control
)


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(multi_map_visualization multi_map_server_messages_cpp)

## Specify libraries to link a library or executable target against
#target_link_libraries(odom_visualization
#   ${catkin_LIBRARIES}
#   ${ARMADILLO_LIBRARIES}
#   pose_utils
#)

