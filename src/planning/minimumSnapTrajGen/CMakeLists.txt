cmake_minimum_required(VERSION 2.8.3)
project(minimumSnapTrajGen)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  bspline_opt
  path_searching
  roscpp
  std_msgs
  cv_bridge
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES minimumSnapTrajGen
 CATKIN_DEPENDS path_searching bspline_opt
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library( minimumSnapTrajGen
    src/planning_visualization.cpp 
    src/polynomial_traj.cpp 
    )
target_link_libraries( minimumSnapTrajGen
    ${catkin_LIBRARIES} 
    )  
