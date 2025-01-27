cmake_minimum_required(VERSION 2.8.3)
project(air_pilot)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++17 )
ADD_COMPILE_OPTIONS(-std=c++17 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  quadrotor_msgs
  mapping
  path_searching
  bspline_opt
  minimumSnapTrajGen
  message_generation
  cv_bridge
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Bspline.msg
  DataDisp.msg
  )
  
# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# catkin_package(CATKIN_DEPENDS message_runtime)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES air_pilot
 CATKIN_DEPENDS mapping path_searching bspline_opt minimumSnapTrajGen message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)


add_executable(air_pilot_node
  src/air_pilot_node.cpp 
  src/ReplanFSM.cpp
  src/PlannerManager.cpp
  )
target_link_libraries(air_pilot_node 
  ${catkin_LIBRARIES}
  )
add_dependencies(air_pilot_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(trajectory_publisher src/trajectory_publisher.cpp)
target_link_libraries(trajectory_publisher ${catkin_LIBRARIES})
add_dependencies(trajectory_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS})


