cmake_minimum_required(VERSION 3.0.2)
project(obj_traj_est)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  nav_msgs
  std_msgs
  message_generation
  visualization_msgs
  apriltag_msgs
  ${MESSAGE_DEPENDENCIES}
)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_message_files(
  FILES
  traj_msg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(INCLUDE_DIRS 
  include
  LIBRARIES traj
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs message_runtime visualization_msgs apriltag_msgs
  #DEPENDS Eigen3
)



include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


add_library(k_filter src/k_filter.cpp)
add_library(traj src/traj.cpp)



add_executable(obj_traj src/obj_traj.cpp)
add_dependencies(obj_traj ${catkin_EXPORTED_TARGETS})
target_link_libraries(obj_traj k_filter traj ${catkin_LIBRARIES})

install(
  TARGETS traj
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)